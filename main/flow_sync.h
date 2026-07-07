#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "flow_packet.h"

#ifdef __cplusplus
extern "C" {
#endif

// Rádio pelo qual o uplink (rumo ao root) acontece, descoberto via beacon.
typedef enum {
    FLOW_UPLINK_NONE = 0,
    FLOW_UPLINK_ESPNOW = 1,
    FLOW_UPLINK_LORA = 2,
} flow_uplink_radio_t;

// Pai escolhido pela descoberta dinâmica (gradiente por rank/hop_from_root).
typedef struct {
    bool                valid;
    uint8_t             parent_id[6];   // MAC do pai (sender_id do melhor beacon)
    uint8_t             root_id[6];     // MAC do root anunciado no beacon
    uint8_t             my_rank;        // = rank do pai + 1
    flow_uplink_radio_t radio;          // rádio em que o melhor beacon chegou
    int16_t             best_rssi;      // RSSI do melhor beacon (desempate)
    uint32_t            sync_seq;       // último sync_seq do pai
    // Agenda carregada do beacon escolhido (p/ programar o deep sleep):
    int64_t             root_epoch_ms;       // tempo do root no beacon
    uint32_t            next_collect_in_ms;  // ms até a próxima coleta
    uint32_t            next_sync_in_ms;     // ms até o próximo sync
} flow_parent_t;

#define FLOW_RANK_INVALID 0xFFU

// ---------------------------------------------------------------------------
// Estimador de deriva do relogio (RC interno, sem cristal 32 kHz).
// O nó persiste esta struct em RTC RAM; a cada resync mede o erro entre
// "quando esperava o beacon do pai" e "quando ele chegou" e refina a
// estimativa. Com isso a guarda converge de minutos para segundos.
// Convencao de sinal: ppm > 0 = relogio local RAPIDO (acorda cedo no tempo
// real; o beacon chega DEPOIS do previsto na regua local) => dormir mais.
// ---------------------------------------------------------------------------
typedef struct {
    int32_t ppm;      // deriva estimada (EWMA das amostras)
    uint8_t samples;  // amostras absorvidas (satura em 255)
} flow_drift_t;

void flow_drift_reset(flow_drift_t *d);

// Absorve uma medicao: err_ms = atraso observado do beacon em relacao ao
// previsto (positivo = chegou DEPOIS = relogio rapido); slept_ms = duracao do
// sleep pela regua local. Amostras absurdas (>5% ou sleep curto) sao descartadas.
void flow_drift_update(flow_drift_t *d, int32_t err_ms, uint32_t slept_ms);

// Corrige uma duracao alvo de sleep pela deriva estimada (compensacao ativa).
uint32_t flow_drift_apply(const flow_drift_t *d, uint32_t sleep_ms);

// Incerteza residual da estimativa (ppm): comeca em init_ppm (pior caso do RC)
// e cai pela metade a cada amostra ate um piso (residual_ppm).
uint32_t flow_drift_uncertainty_ppm(const flow_drift_t *d,
                                    uint32_t init_ppm, uint32_t residual_ppm);

// Guarda de escuta para a proxima acordada:
//   incerteza_ppm x sleep_ms + margem de jitter da cascata (por rank),
//   dobrando por miss consecutivo, clampada em [guard_min, guard_max].
uint32_t flow_sync_compute_guard(uint32_t uncertainty_ppm, uint32_t sleep_ms,
                                 uint8_t rank, uint8_t consecutive_misses,
                                 uint32_t hop_jitter_ms,
                                 uint32_t guard_min_ms, uint32_t guard_max_ms);

// Constrói um beacon para (re)transmitir, dado o estado deste node.
void flow_sync_build_beacon(flow_beacon_t *out,
                            const uint8_t self_id[6],
                            const uint8_t root_id[6],
                            uint8_t my_rank,
                            uint32_t sync_seq,
                            int64_t root_epoch_ms,
                            uint32_t next_collect_in_ms,
                            uint32_t next_sync_in_ms);

// Avalia um beacon recebido para seleção de pai. Atualiza *parent se for melhor
// (menor rank; desempate por maior RSSI). Retorna true se o pai foi (re)definido.
// Beacons com CRC inválido ou rank inutilizável são ignorados.
bool flow_sync_consider_beacon(flow_parent_t *parent,
                               const flow_beacon_t *b,
                               flow_uplink_radio_t radio,
                               int16_t rssi);

// Zera a seleção de pai (no início de uma janela de sync).
void flow_sync_reset_parent(flow_parent_t *parent);

#ifdef __cplusplus
}
#endif
