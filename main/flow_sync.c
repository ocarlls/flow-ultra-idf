#include "flow_sync.h"

#include <string.h>

void flow_sync_reset_parent(flow_parent_t *parent)
{
    if (parent == NULL) {
        return;
    }
    memset(parent, 0, sizeof(*parent));
    parent->valid = false;
    parent->my_rank = FLOW_RANK_INVALID;
    parent->radio = FLOW_UPLINK_NONE;
    parent->best_rssi = INT16_MIN;
}

void flow_sync_build_beacon(flow_beacon_t *out,
                            const uint8_t self_id[6],
                            const uint8_t root_id[6],
                            uint8_t my_rank,
                            uint32_t sync_seq,
                            int64_t root_epoch_ms,
                            uint32_t next_collect_in_ms,
                            uint32_t next_sync_in_ms)
{
    if (out == NULL) {
        return;
    }
    flow_beacon_init_empty(out);
    if (root_id != NULL) {
        memcpy(out->root_id, root_id, 6);
    }
    if (self_id != NULL) {
        memcpy(out->sender_id, self_id, 6);
    }
    out->hop_from_root = my_rank;
    out->sync_seq = sync_seq;
    out->root_epoch_ms = root_epoch_ms;
    out->next_collect_in_ms = next_collect_in_ms;
    out->next_sync_in_ms = next_sync_in_ms;
    flow_beacon_update_crc32(out);
}

// ---------------------------------------------------------------------------
// Deriva adaptativa
//
// Modelo: o nó dorme sleep_ms pela regua local e acorda esperando o beacon do
// pai apos "guard" ms de escuta. Se o relogio local for LENTO, o nó acorda
// mais tarde no tempo real e o beacon chega ANTES do previsto (err_ms < 0);
// se for RAPIDO, acorda cedo e o beacon chega DEPOIS (err_ms > 0). A razao
// err_ms/slept_ms e a deriva relativa; guardamos EWMA em ppm. Na hora de
// dormir de novo, esticamos/encolhemos o sleep na proporcao inversa.
// ---------------------------------------------------------------------------

#define FLOW_DRIFT_SAMPLE_MAX_PPM 50000  /* rejeita amostras > 5% (lixo) */
#define FLOW_DRIFT_MIN_SLEEP_MS   5000   /* sleep curto demais nao mede nada */

void flow_drift_reset(flow_drift_t *d)
{
    if (d == NULL) {
        return;
    }
    d->ppm = 0;
    d->samples = 0;
}

void flow_drift_update(flow_drift_t *d, int32_t err_ms, uint32_t slept_ms)
{
    if (d == NULL || slept_ms < FLOW_DRIFT_MIN_SLEEP_MS) {
        return;
    }
    const int64_t sample_ppm64 = ((int64_t)err_ms * 1000000LL) / (int64_t)slept_ms;
    if (sample_ppm64 > FLOW_DRIFT_SAMPLE_MAX_PPM ||
        sample_ppm64 < -FLOW_DRIFT_SAMPLE_MAX_PPM) {
        return; /* outlier: beacon atrasado por outra causa, nao por deriva */
    }
    const int32_t sample_ppm = (int32_t)sample_ppm64;
    /* O sleep anterior JA foi corrigido por d->ppm (flow_drift_apply), logo a
     * amostra e um RESIDUO: acumula (controle integral, ganho 1/2). Na
     * primeira amostra nao havia correcao aplicada => absorve inteira. */
    if (d->samples == 0) {
        d->ppm = sample_ppm;
    } else {
        d->ppm += (sample_ppm * 3) / 4;
    }
    if (d->ppm > FLOW_DRIFT_SAMPLE_MAX_PPM) {
        d->ppm = FLOW_DRIFT_SAMPLE_MAX_PPM;
    } else if (d->ppm < -FLOW_DRIFT_SAMPLE_MAX_PPM) {
        d->ppm = -FLOW_DRIFT_SAMPLE_MAX_PPM;
    }
    if (d->samples < 255) {
        d->samples++;
    }
}

uint32_t flow_drift_apply(const flow_drift_t *d, uint32_t sleep_ms)
{
    if (d == NULL || d->samples == 0) {
        return sleep_ms;
    }
    /* err = sleep * ppm/1e6 e o atraso previsto do beacon; compensamos
     * dormindo sleep + err (mesmo sinal: relogio rapido => beacon chega
     * "depois" na regua local => dorme mais para acordar rente a ele). */
    const int64_t corr = ((int64_t)sleep_ms * d->ppm) / 1000000LL;
    int64_t adjusted = (int64_t)sleep_ms + corr;
    if (adjusted < 1000) {
        adjusted = 1000;
    }
    return (uint32_t)adjusted;
}

uint32_t flow_drift_uncertainty_ppm(const flow_drift_t *d,
                                    uint32_t init_ppm, uint32_t residual_ppm)
{
    if (d == NULL || d->samples == 0) {
        return init_ppm;
    }
    /* Apos convergencia, usa |ppm| real como incerteza (com piso residual). */
    const uint32_t abs_ppm = (d->ppm < 0)
                                 ? (uint32_t)(-d->ppm)
                                 : (uint32_t)d->ppm;
    return (abs_ppm > residual_ppm) ? abs_ppm : residual_ppm;
}

uint32_t flow_sync_compute_guard(uint32_t uncertainty_ppm, uint32_t sleep_ms,
                                 uint8_t rank, uint8_t consecutive_misses,
                                 uint32_t hop_jitter_ms,
                                 uint32_t guard_min_ms, uint32_t guard_max_ms)
{
    uint64_t guard = ((uint64_t)sleep_ms * uncertainty_ppm) / 1000000ULL;

    /* Jitter de cascata: o burst do pai comeca com um atraso variavel em
     * relacao ao do avo (captura + processamento), acumulado por salto. */
    if (rank != FLOW_RANK_INVALID && rank > 0U) {
        guard += (uint64_t)rank * hop_jitter_ms;
    }

    /* Escalada por miss: dobra a cada perda consecutiva (ate o teto). */
    for (uint8_t i = 0; i < consecutive_misses && i < 8U; ++i) {
        guard *= 2U;
        if (guard >= guard_max_ms) {
            break;
        }
    }

    if (guard < guard_min_ms) {
        guard = guard_min_ms;
    }
    if (guard > guard_max_ms) {
        guard = guard_max_ms;
    }
    return (uint32_t)guard;
}

bool flow_sync_consider_beacon(flow_parent_t *parent,
                               const flow_beacon_t *b,
                               flow_uplink_radio_t radio,
                               int16_t rssi)
{
    if (parent == NULL || !flow_beacon_validate_crc32(b)) {
        return false;
    }

    // Um pai utilizável precisa ter um caminho finito ao root (rank < 254),
    // senão rank+1 estouraria o limite e não levaria ao root.
    if (b->hop_from_root >= (FLOW_RANK_INVALID - 1U)) {
        return false;
    }
    const uint8_t candidate_rank = (uint8_t)(b->hop_from_root + 1U);

    // Critério de seleção: menor rank vence; empate → maior RSSI.
    bool better = false;
    if (!parent->valid) {
        better = true;
    } else if (candidate_rank < parent->my_rank) {
        better = true;
    } else if (candidate_rank == parent->my_rank && rssi > parent->best_rssi) {
        better = true;
    }

    if (!better) {
        // Mantém o pai atual, mas acompanha o sync_seq mais recente dele.
        if (parent->valid && memcmp(parent->parent_id, b->sender_id, 6) == 0) {
            parent->sync_seq = b->sync_seq;
        }
        return false;
    }

    parent->valid = true;
    memcpy(parent->parent_id, b->sender_id, 6);
    memcpy(parent->root_id, b->root_id, 6);
    parent->my_rank = candidate_rank;
    parent->radio = radio;
    parent->best_rssi = rssi;
    parent->sync_seq = b->sync_seq;
    parent->root_epoch_ms = b->root_epoch_ms;
    parent->next_collect_in_ms = b->next_collect_in_ms;
    parent->next_sync_in_ms = b->next_sync_in_ms;
    return true;
}
