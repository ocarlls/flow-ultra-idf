#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "driver/spi_master.h"
#include "driver/rtc_io.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "soc/soc_caps.h"

#include "as6031_remote.h"
#include "espnow_test_module.h"

#include "sdkconfig.h"

// --- Configurações de Hardware ---
#define PIN_CS           10
#define PIN_MOSI         11
#define PIN_MISO         13
#define PIN_SCK          12
#define PIN_INT_AS6031   11

// --- Constantes Físicas ---
const double CO = 1482.0;    // Velocidade do som (m/s)
const double L  = 0.07;      // Distância entre transdutores (m)
const double D  = 0.027;     // Diâmetro interno (m)
const double PI = 3.1415926535;

static const char *TAG = "WATER_METER";

#define DEVICE_ROLE_NODE        1

// Se configurado via menuconfig, respeita o valor; senao usa default 1.
#ifdef CONFIG_AS6031_INTERNAL_TOTALIZER
#define AS6031_INTERNAL_TOTALIZER CONFIG_AS6031_INTERNAL_TOTALIZER
#else
#define AS6031_INTERNAL_TOTALIZER 1
#endif

#ifdef CONFIG_AS6031_VALIDATE_MODE
#define AS6031_VALIDATE_MODE CONFIG_AS6031_VALIDATE_MODE
#else
#define AS6031_VALIDATE_MODE 0
#endif

#ifdef CONFIG_AS6031_VALIDATE_INTERVAL_MS
#define AS6031_VALIDATE_INTERVAL_MS CONFIG_AS6031_VALIDATE_INTERVAL_MS
#else
#define AS6031_VALIDATE_INTERVAL_MS 1000
#endif

#ifdef CONFIG_AS6031_VALIDATE_NO_FLOW_THRESHOLD_MLPM
#define AS6031_VALIDATE_NO_FLOW_THRESHOLD_MLPM CONFIG_AS6031_VALIDATE_NO_FLOW_THRESHOLD_MLPM
#else
#define AS6031_VALIDATE_NO_FLOW_THRESHOLD_MLPM 100
#endif

#ifdef CONFIG_AS6031_VALIDATE_NEGATIVE_GLITCH_ML
#define AS6031_VALIDATE_NEGATIVE_GLITCH_ML CONFIG_AS6031_VALIDATE_NEGATIVE_GLITCH_ML
#else
#define AS6031_VALIDATE_NEGATIVE_GLITCH_ML 2
#endif

#ifdef CONFIG_AS6031_MAINTENANCE_MODE
#define AS6031_MAINTENANCE_MODE CONFIG_AS6031_MAINTENANCE_MODE
#else
#define AS6031_MAINTENANCE_MODE 0
#endif

#ifdef CONFIG_AS6031_MAINTENANCE_INTERVAL_MS
#define AS6031_MAINTENANCE_INTERVAL_MS CONFIG_AS6031_MAINTENANCE_INTERVAL_MS
#else
#define AS6031_MAINTENANCE_INTERVAL_MS 1000
#endif

#ifdef CONFIG_AS6031_MAX_DAILY_DELTA_LITERS
#define AS6031_MAX_DAILY_DELTA_LITERS CONFIG_AS6031_MAX_DAILY_DELTA_LITERS
#else
#define AS6031_MAX_DAILY_DELTA_LITERS 5000
#endif

#ifdef CONFIG_AS6031_DAILY_WAKEUP_SECONDS
#define AS6031_DAILY_WAKEUP_SECONDS CONFIG_AS6031_DAILY_WAKEUP_SECONDS
#else
#define AS6031_DAILY_WAKEUP_SECONDS 86400
#endif

#ifdef CONFIG_AS6031_TOTALIZER_COUNTER_DECREASING
#define AS6031_TOTALIZER_COUNTER_DECREASING CONFIG_AS6031_TOTALIZER_COUNTER_DECREASING
#else
#define AS6031_TOTALIZER_COUNTER_DECREASING 1
#endif

// Limites de migracao para recuperar ancora legada de escala/formato antigo.
#define AS6031_MIGRATION_MAX_TOTAL_LITERS 10000000.0
#define AS6031_MIGRATION_REANCHOR_RATIO   4.0

// Quando os enderecos do totalizador interno nao sao conhecidos, este modo faz uma varredura
// da RAM do AS6031 (RAA) para ajudar a descobrir quais celulas mudam com o fluxo.
// Mantem o ESP acordado e loga os enderecos que mudaram entre duas amostras.

// Discovery: pode ser controlado via menuconfig.
#ifdef CONFIG_AS6031_DISCOVERY_MODE
#define AS6031_DISCOVERY_MODE CONFIG_AS6031_DISCOVERY_MODE
#else
#define AS6031_DISCOVERY_MODE 0
#endif


#ifdef CONFIG_AS6031_DISCOVERY_INTERVAL_MS
#define AS6031_DISCOVERY_INTERVAL_MS CONFIG_AS6031_DISCOVERY_INTERVAL_MS
#else
#define AS6031_DISCOVERY_INTERVAL_MS 5000
#endif

#ifdef CONFIG_AS6031_DISCOVERY_SAMPLES
#define AS6031_DISCOVERY_SAMPLES CONFIG_AS6031_DISCOVERY_SAMPLES
#else
#define AS6031_DISCOVERY_SAMPLES 2
#endif

#ifdef CONFIG_AS6031_DISCOVERY_TOPK
#define AS6031_DISCOVERY_TOPK CONFIG_AS6031_DISCOVERY_TOPK
#else
#define AS6031_DISCOVERY_TOPK 20
#endif

#ifdef CONFIG_AS6031_DISCOVERY_REPEAT_COUNT
#define AS6031_DISCOVERY_REPEAT_COUNT CONFIG_AS6031_DISCOVERY_REPEAT_COUNT
#else
#define AS6031_DISCOVERY_REPEAT_COUNT 0
#endif


#ifdef CONFIG_AS6031_DISCOVERY_INCLUDE_NVRAM
#define AS6031_DISCOVERY_INCLUDE_NVRAM CONFIG_AS6031_DISCOVERY_INCLUDE_NVRAM
#else
#define AS6031_DISCOVERY_INCLUDE_NVRAM 0
#endif

#ifdef CONFIG_AS6031_FORCE_SYS_INIT_ON_BOOT
#define AS6031_FORCE_SYS_INIT_ON_BOOT CONFIG_AS6031_FORCE_SYS_INIT_ON_BOOT
#else
#define AS6031_FORCE_SYS_INIT_ON_BOOT 0
#endif

#ifdef CONFIG_AS6031_COMMISSIONING_TOF_MODE
#define AS6031_COMMISSIONING_TOF_MODE CONFIG_AS6031_COMMISSIONING_TOF_MODE
#else
#define AS6031_COMMISSIONING_TOF_MODE 0
#endif

#ifdef CONFIG_AS6031_COMMISSIONING_INTERVAL_MS
#define AS6031_COMMISSIONING_INTERVAL_MS CONFIG_AS6031_COMMISSIONING_INTERVAL_MS
#else
#define AS6031_COMMISSIONING_INTERVAL_MS 500
#endif

#ifdef CONFIG_AS6031_COMMISSIONING_FLOW_THRESHOLD_MLPS
#define AS6031_COMMISSIONING_FLOW_THRESHOLD_MLPS CONFIG_AS6031_COMMISSIONING_FLOW_THRESHOLD_MLPS
#else
#define AS6031_COMMISSIONING_FLOW_THRESHOLD_MLPS 20
#endif

#define AS6031_DISCOVERY_RAM_START 0x000u
#define AS6031_DISCOVERY_RAM_END   0x0AFu  // RAM 0x000..0x0AF (datasheet)

#define AS6031_DISCOVERY_NVRAM_START 0x100u
#define AS6031_DISCOVERY_NVRAM_END   0x1FFu // NVRAM 0x100..0x1FF (datasheet)

#ifndef DISCOVERY_REPORT_MAX_LEN
#define DISCOVERY_REPORT_MAX_LEN 4096
#endif

#if AS6031_DISCOVERY_MODE
// Persistencia de resumo do discovery para leitura posterior em campo.
#define NVS_NS_DISC "disc"
#define NVS_KEY_DISC_VALID "d_valid"
#define NVS_KEY_DISC_LABEL "d_label"
#define NVS_KEY_DISC_CHANGED "d_changed"
#define NVS_KEY_DISC_SAMPLES "d_samples"
#define NVS_KEY_DISC_INTERVAL "d_int_ms"
#define NVS_KEY_DISC_U32_OK "d_u32_ok"
#define NVS_KEY_DISC_U32_ADDR "d_u32_addr"
#define NVS_KEY_DISC_U32_NET "d_u32_net"
#define NVS_KEY_DISC_P64_OK "d_p64_ok"
#define NVS_KEY_DISC_P64_ADDR "d_p64_addr"
#define NVS_KEY_DISC_P64_NET "d_p64_net"
#define NVS_KEY_DISC_REPORT "d_report"

static void as6031_discovery_store_summary(uint8_t label_id,
                                           uint32_t changed_cells,
                                           int samples,
                                           int interval_ms,
                                           bool has_u32,
                                           uint16_t u32_addr,
                                           int64_t u32_net,
                                           bool has_p64,
                                           uint16_t p64_addr,
                                           int64_t p64_net,
                                           const char *report_text)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS_DISC, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DISCOVERY: falha ao abrir NVS (%s)", esp_err_to_name(err));
        return;
    }

    err = nvs_set_u8(h, NVS_KEY_DISC_VALID, 1);
    if (err == ESP_OK) err = nvs_set_u8(h, NVS_KEY_DISC_LABEL, label_id);
    if (err == ESP_OK) err = nvs_set_u32(h, NVS_KEY_DISC_CHANGED, changed_cells);
    if (err == ESP_OK) err = nvs_set_u32(h, NVS_KEY_DISC_SAMPLES, (uint32_t)samples);
    if (err == ESP_OK) err = nvs_set_u32(h, NVS_KEY_DISC_INTERVAL, (uint32_t)interval_ms);

    if (err == ESP_OK) err = nvs_set_u8(h, NVS_KEY_DISC_U32_OK, has_u32 ? 1 : 0);
    if (err == ESP_OK && has_u32) {
        err = nvs_set_u16(h, NVS_KEY_DISC_U32_ADDR, u32_addr);
        if (err == ESP_OK) err = nvs_set_i64(h, NVS_KEY_DISC_U32_NET, u32_net);
    }

    if (err == ESP_OK) err = nvs_set_u8(h, NVS_KEY_DISC_P64_OK, has_p64 ? 1 : 0);
    if (err == ESP_OK && has_p64) {
        err = nvs_set_u16(h, NVS_KEY_DISC_P64_ADDR, p64_addr);
        if (err == ESP_OK) err = nvs_set_i64(h, NVS_KEY_DISC_P64_NET, p64_net);
    }

    if (err == ESP_OK && report_text != NULL) {
        const size_t report_len = strnlen(report_text, DISCOVERY_REPORT_MAX_LEN - 1) + 1;
        err = nvs_set_blob(h, NVS_KEY_DISC_REPORT, report_text, report_len);
    }

    if (err == ESP_OK) {
        err = nvs_commit(h);
    }

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DISCOVERY: falha ao persistir resumo (%s)", esp_err_to_name(err));
    }

    nvs_close(h);
}

static void as6031_discovery_log_persisted_summary(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS_DISC, NVS_READONLY, &h);
    if (err != ESP_OK) {
        return;
    }

    uint8_t valid = 0;
    if (nvs_get_u8(h, NVS_KEY_DISC_VALID, &valid) != ESP_OK || valid == 0) {
        nvs_close(h);
        return;
    }

    uint8_t label_id = 0;
    uint32_t changed = 0;
    uint32_t samples = 0;
    uint32_t interval_ms = 0;
    uint8_t u32_ok = 0;
    uint16_t u32_addr = 0;
    int64_t u32_net = 0;
    uint8_t p64_ok = 0;
    uint16_t p64_addr = 0;
    int64_t p64_net = 0;

    (void)nvs_get_u8(h, NVS_KEY_DISC_LABEL, &label_id);
    (void)nvs_get_u32(h, NVS_KEY_DISC_CHANGED, &changed);
    (void)nvs_get_u32(h, NVS_KEY_DISC_SAMPLES, &samples);
    (void)nvs_get_u32(h, NVS_KEY_DISC_INTERVAL, &interval_ms);
    (void)nvs_get_u8(h, NVS_KEY_DISC_U32_OK, &u32_ok);
    if (u32_ok) {
        (void)nvs_get_u16(h, NVS_KEY_DISC_U32_ADDR, &u32_addr);
        (void)nvs_get_i64(h, NVS_KEY_DISC_U32_NET, &u32_net);
    }
    (void)nvs_get_u8(h, NVS_KEY_DISC_P64_OK, &p64_ok);
    if (p64_ok) {
        (void)nvs_get_u16(h, NVS_KEY_DISC_P64_ADDR, &p64_addr);
        (void)nvs_get_i64(h, NVS_KEY_DISC_P64_NET, &p64_net);
    }

    const char *label = (label_id == 1) ? "NVRAM" : "RAM";
    ESP_LOGW(TAG,
             "DISCOVERY(NVS): ultimo resumo [%s], changed=%lu, samples=%lu, interval=%lums",
             label,
             (unsigned long)changed,
             (unsigned long)samples,
             (unsigned long)interval_ms);

    if (u32_ok) {
        ESP_LOGW(TAG,
                 "DISCOVERY(NVS): melhor u32 addr=0x%03X net=%lld",
                 (unsigned)u32_addr,
                 (long long)u32_net);
    }
    if (p64_ok) {
        ESP_LOGW(TAG,
                 "DISCOVERY(NVS): melhor par 32.32 addr=0x%03X/0x%03X net=%lld",
                 (unsigned)p64_addr,
                 (unsigned)(p64_addr + 1),
                 (long long)p64_net);
    }

    size_t report_len = 0;
    if (nvs_get_blob(h, NVS_KEY_DISC_REPORT, NULL, &report_len) == ESP_OK && report_len > 1 && report_len <= DISCOVERY_REPORT_MAX_LEN) {
        char *report = (char *)calloc(1, report_len);
        if (report != NULL) {
            if (nvs_get_blob(h, NVS_KEY_DISC_REPORT, report, &report_len) == ESP_OK) {
                report[report_len - 1] = '\0';
                ESP_LOGW(TAG, "DISCOVERY(NVS): relatorio completo:\n%s", report);
            }
            free(report);
        }
    }

    nvs_close(h);
}
#endif

static esp_err_t as6031_read_range(spi_device_handle_t dev, uint16_t start_addr, uint16_t end_addr, uint32_t *out)
{
    if (start_addr > end_addr || end_addr > 0x1FFu || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const size_t count = (size_t)(end_addr - start_addr + 1);
    size_t off = 0;
    while (off < count) {
        const size_t batch = (count - off) > 32 ? 32 : (count - off);
        esp_err_t err = as6031_raard_dwords(dev, (uint16_t)(start_addr + off), &out[off], batch, false, NULL);
        if (err != ESP_OK) {
            return err;
        }
        off += batch;
    }
    return ESP_OK;
}

static void as6031_discovery_scan_range(spi_device_handle_t dev, uint16_t start, uint16_t end, const char *label)
{
    const size_t count = (size_t)(end - start + 1);

    const int samples = (AS6031_DISCOVERY_SAMPLES < 2) ? 2 : (int)AS6031_DISCOVERY_SAMPLES;
    const int topk = (AS6031_DISCOVERY_TOPK < 1) ? 1 : (int)AS6031_DISCOVERY_TOPK;

    uint32_t *snaps = (uint32_t *)calloc((size_t)samples * count, sizeof(uint32_t));
    if (!snaps) {
        ESP_LOGE(TAG, "Discovery: sem memoria para snapshots (%u dwords x %d)", (unsigned)count, samples);
        return;
    }

    ESP_LOGW(TAG, "DISCOVERY: enderecos do totalizador nao configurados (AS6031_ADDR_RAM_FLOW_VOLUME_* = 0xFFFF)");
    ESP_LOGW(TAG, "DISCOVERY: varrendo %s 0x%03X..0x%03X (%d snapshots, %ums)",
             (label ? label : "RAA"), start, end, samples, (unsigned)AS6031_DISCOVERY_INTERVAL_MS);
    ESP_LOGW(TAG, "DISCOVERY: gere fluxo (agua passando) durante a espera para ver contadores mudando");

    esp_err_t err = ESP_OK;
    for (int s = 0; s < samples; s++) {
        uint32_t *snap = &snaps[(size_t)s * count];
        err = as6031_read_range(dev, start, end, snap);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Discovery: falha ao ler snapshot %d/%d: %s", s + 1, samples, esp_err_to_name(err));
            goto done;
        }
        if (s + 1 < samples) {
            vTaskDelay(pdMS_TO_TICKS(AS6031_DISCOVERY_INTERVAL_MS));
        }
    }

    typedef struct {
        uint16_t addr;
        uint32_t first;
        uint32_t last;
        int inc;
        int dec;
        uint64_t abs_delta_sum;
        int64_t net;
    } cand32_t;

    typedef struct {
        uint16_t addr;
        uint64_t first;
        uint64_t last;
        int inc;
        int dec;
        uint64_t abs_delta_sum;
        int64_t net;
    } cand64_t;

    cand32_t *c32 = (cand32_t *)calloc(count, sizeof(cand32_t));
    cand64_t *c64 = (cand64_t *)calloc((count > 1 ? (count - 1) : 0), sizeof(cand64_t));
    if (!c32 || (!c64 && count > 1)) {
        ESP_LOGE(TAG, "Discovery: sem memoria para ranking");
        free(c32);
        free(c64);
        goto done;
    }

    size_t changed_cells = 0;
    for (size_t i = 0; i < count; i++) {
        cand32_t c = {
            .addr = (uint16_t)(start + i),
            .first = snaps[i],
            .last = snaps[(size_t)(samples - 1) * count + i],
            .inc = 0,
            .dec = 0,
            .abs_delta_sum = 0,
            .net = (int64_t)((int64_t)c.last - (int64_t)c.first),
        };
        for (int s = 1; s < samples; s++) {
            uint32_t prev = snaps[(size_t)(s - 1) * count + i];
            uint32_t cur = snaps[(size_t)s * count + i];
            int64_t d = (int64_t)((int64_t)cur - (int64_t)prev);
            if (d > 0) c.inc++;
            else if (d < 0) c.dec++;
            c.abs_delta_sum += (uint64_t)(d < 0 ? -d : d);
        }
        c32[i] = c;
        if (c.first != c.last) {
            changed_cells++;
        }
    }

    size_t pair_count = (count > 1 ? (count - 1) : 0);
    for (size_t i = 0; i < pair_count; i++) {
        uint64_t first64 = ((uint64_t)snaps[i] << 32) | (uint64_t)snaps[i + 1];
        uint64_t last64 = ((uint64_t)snaps[(size_t)(samples - 1) * count + i] << 32) | (uint64_t)snaps[(size_t)(samples - 1) * count + i + 1];
        cand64_t c = {
            .addr = (uint16_t)(start + i),
            .first = first64,
            .last = last64,
            .inc = 0,
            .dec = 0,
            .abs_delta_sum = 0,
            .net = (int64_t)(last64 - first64),
        };
        for (int s = 1; s < samples; s++) {
            uint64_t prev64 = ((uint64_t)snaps[(size_t)(s - 1) * count + i] << 32) | (uint64_t)snaps[(size_t)(s - 1) * count + i + 1];
            uint64_t cur64 = ((uint64_t)snaps[(size_t)s * count + i] << 32) | (uint64_t)snaps[(size_t)s * count + i + 1];
            int64_t d = (int64_t)(cur64 - prev64);
            if (d > 0) c.inc++;
            else if (d < 0) c.dec++;
            c.abs_delta_sum += (uint64_t)(d < 0 ? -d : d);
        }
        c64[i] = c;
    }

    ESP_LOGI(TAG, "DISCOVERY: %s: %u/%u celulas mudaram (first != last)",
             (label ? label : "RAA"), (unsigned)changed_cells, (unsigned)count);

    char *discovery_report = (char *)calloc(1, DISCOVERY_REPORT_MAX_LEN);
    size_t report_used = 0;
#define APPEND_REPORT(...) do { \
    if (discovery_report != NULL && report_used < DISCOVERY_REPORT_MAX_LEN) { \
        int _n = snprintf(discovery_report + report_used, DISCOVERY_REPORT_MAX_LEN - report_used, __VA_ARGS__); \
        if (_n > 0) { \
            size_t _nn = (size_t)_n; \
            if (_nn >= (DISCOVERY_REPORT_MAX_LEN - report_used)) report_used = DISCOVERY_REPORT_MAX_LEN - 1; \
            else report_used += _nn; \
        } \
    } \
} while (0)

    APPEND_REPORT("DISCOVERY REPORT [%s]\n", (label ? label : "RAA"));
    APPEND_REPORT("range=0x%03X..0x%03X samples=%d interval_ms=%u changed=%u/%u\n",
                  start,
                  end,
                  samples,
                  (unsigned)AS6031_DISCOVERY_INTERVAL_MS,
                  (unsigned)changed_cells,
                  (unsigned)count);

    // Ordena candidatos 32-bit: preferir monotonicamente crescente (dec==0), net>0 e maior net.
    for (size_t i = 0; i < count; i++) {
        for (size_t j = i + 1; j < count; j++) {
            const bool i_good = (c32[i].dec == 0 && c32[i].net > 0);
            const bool j_good = (c32[j].dec == 0 && c32[j].net > 0);
            bool swap = false;
            if (j_good && !i_good) swap = true;
            else if (j_good == i_good) {
                if (c32[j].net > c32[i].net) swap = true;
                else if (c32[j].net == c32[i].net && c32[j].inc > c32[i].inc) swap = true;
                else if (c32[j].net == c32[i].net && c32[j].inc == c32[i].inc && c32[j].abs_delta_sum > c32[i].abs_delta_sum) swap = true;
            }
            if (swap) {
                cand32_t tmp = c32[i];
                c32[i] = c32[j];
                c32[j] = tmp;
            }
        }
    }

    // Ordena candidatos 64-bit (pares 32.32): mesma logica.
    for (size_t i = 0; i < pair_count; i++) {
        for (size_t j = i + 1; j < pair_count; j++) {
            const bool i_good = (c64[i].dec == 0 && c64[i].net > 0);
            const bool j_good = (c64[j].dec == 0 && c64[j].net > 0);
            bool swap = false;
            if (j_good && !i_good) swap = true;
            else if (j_good == i_good) {
                if (c64[j].net > c64[i].net) swap = true;
                else if (c64[j].net == c64[i].net && c64[j].inc > c64[i].inc) swap = true;
                else if (c64[j].net == c64[i].net && c64[j].inc == c64[i].inc && c64[j].abs_delta_sum > c64[i].abs_delta_sum) swap = true;
            }
            if (swap) {
                cand64_t tmp = c64[i];
                c64[i] = c64[j];
                c64[j] = tmp;
            }
        }
    }

    ESP_LOGI(TAG, "DISCOVERY: %s: TOP %d (u32) candidatos (preferencia: dec=0, net>0)", (label ? label : "RAA"), topk);
    APPEND_REPORT("TOP u32:\n");
    int printed = 0;
    for (size_t i = 0; i < count && printed < topk; i++) {
        if (c32[i].net == 0 && c32[i].abs_delta_sum == 0) {
            continue;
        }
        ESP_LOGI(TAG, "DISCOVERY: u32 addr 0x%03X: 0x%08lX -> 0x%08lX net=%lld inc=%d dec=%d abs=%llu",
                 c32[i].addr,
                 (unsigned long)c32[i].first,
                 (unsigned long)c32[i].last,
                 (long long)c32[i].net,
                 c32[i].inc,
                 c32[i].dec,
                 (unsigned long long)c32[i].abs_delta_sum);
        APPEND_REPORT("u32 addr 0x%03X: 0x%08lX -> 0x%08lX net=%lld inc=%d dec=%d abs=%llu\n",
                      c32[i].addr,
                      (unsigned long)c32[i].first,
                      (unsigned long)c32[i].last,
                      (long long)c32[i].net,
                      c32[i].inc,
                      c32[i].dec,
                      (unsigned long long)c32[i].abs_delta_sum);
        printed++;
    }

    ESP_LOGI(TAG, "DISCOVERY: %s: TOP %d (pares 32.32) candidatos", (label ? label : "RAA"), topk);
    APPEND_REPORT("TOP pair 32.32:\n");
    printed = 0;
    for (size_t i = 0; i < pair_count && printed < topk; i++) {
        if (c64[i].net <= 0 || c64[i].abs_delta_sum == 0) {
            continue;
        }
        const uint32_t first_hi = (uint32_t)(c64[i].first >> 32);
        const uint32_t first_lo = (uint32_t)(c64[i].first & 0xFFFFFFFFu);
        const uint32_t last_hi = (uint32_t)(c64[i].last >> 32);
        const uint32_t last_lo = (uint32_t)(c64[i].last & 0xFFFFFFFFu);

        const double m3_first = (double)first_hi + ((double)first_lo / 4294967296.0);
        const double m3_last = (double)last_hi + ((double)last_lo / 4294967296.0);

        ESP_LOGI(TAG, "DISCOVERY: par 0x%03X/0x%03X: %.9f -> %.9f m3 (%.3f -> %.3f L) inc=%d dec=%d",
                 c64[i].addr, (uint16_t)(c64[i].addr + 1),
                 m3_first, m3_last,
                 m3_first * 1000.0, m3_last * 1000.0,
                 c64[i].inc, c64[i].dec);
        APPEND_REPORT("pair 0x%03X/0x%03X: %.9f -> %.9f m3 (%.3f -> %.3f L) inc=%d dec=%d\n",
                      c64[i].addr,
                      (uint16_t)(c64[i].addr + 1),
                      m3_first,
                      m3_last,
                      m3_first * 1000.0,
                      m3_last * 1000.0,
                      c64[i].inc,
                      c64[i].dec);
        printed++;
    }

#if AS6031_DISCOVERY_MODE
    // Persiste um resumo do melhor candidato para consulta posterior, mesmo sem monitor serial no momento do teste.
    bool has_u32 = false;
    uint16_t best_u32_addr = 0;
    int64_t best_u32_net = 0;
    for (size_t i = 0; i < count; i++) {
        if (c32[i].dec == 0 && c32[i].net > 0) {
            has_u32 = true;
            best_u32_addr = c32[i].addr;
            best_u32_net = c32[i].net;
            break;
        }
    }

    bool has_p64 = false;
    uint16_t best_p64_addr = 0;
    int64_t best_p64_net = 0;
    for (size_t i = 0; i < pair_count; i++) {
        if (c64[i].dec == 0 && c64[i].net > 0) {
            has_p64 = true;
            best_p64_addr = c64[i].addr;
            best_p64_net = c64[i].net;
            break;
        }
    }

    if (changed_cells > 0) {
        const uint8_t label_id = (label && strcmp(label, "NVRAM") == 0) ? 1 : 0;
        as6031_discovery_store_summary(label_id,
                                       (uint32_t)changed_cells,
                                       samples,
                                       AS6031_DISCOVERY_INTERVAL_MS,
                                       has_u32,
                                       best_u32_addr,
                                       best_u32_net,
                                       has_p64,
                                       best_p64_addr,
                                       best_p64_net,
                                       discovery_report);
        if (has_u32 || has_p64) {
            ESP_LOGW(TAG, "DISCOVERY: resumo persistido em NVS para leitura posterior");
        } else {
            ESP_LOGW(TAG, "DISCOVERY: houve mudanca, mas sem candidato monotônico; resumo bruto persistido em NVS");
        }
    }
#endif

    ESP_LOGW(TAG, "DISCOVERY: se algum par incrementou com fluxo, configure os enderecos no menuconfig (AS6031 / Flow Meter)");

    free(c32);
    free(c64);
    free(discovery_report);
#undef APPEND_REPORT

done:
    free(snaps);
}

static void as6031_discovery_scan(spi_device_handle_t dev)
{
    as6031_discovery_scan_range(dev, AS6031_DISCOVERY_RAM_START, AS6031_DISCOVERY_RAM_END, "RAM");
#if AS6031_DISCOVERY_INCLUDE_NVRAM
    as6031_discovery_scan_range(dev, AS6031_DISCOVERY_NVRAM_START, AS6031_DISCOVERY_NVRAM_END, "NVRAM");
#endif
}

#if AS6031_VALIDATE_MODE
static void as6031_validate_totalizer_loop(spi_device_handle_t dev)
{
    ESP_LOGW(TAG, "VALIDACAO: lendo totalizador continuamente (intervalo=%ums)", (unsigned)AS6031_VALIDATE_INTERVAL_MS);
    ESP_LOGW(TAG,
             "VALIDACAO: limiar sem fluxo=%.3f L/min, tolerancia glitch negativo=%.3f L",
             ((double)AS6031_VALIDATE_NO_FLOW_THRESHOLD_MLPM) / 1000.0,
             ((double)AS6031_VALIDATE_NEGATIVE_GLITCH_ML) / 1000.0);
    ESP_LOGW(TAG, "VALIDACAO: interrompa o fluxo para ver taxa=0; gere fluxo para ver taxa>0");

    double last_liters = NAN;
    double baseline_liters = NAN;
    double positive_accum_liters = 0.0;
    int64_t last_t_us = 0;

    while (1) {
        double liters = 0.0;
        esp_err_t err = as6031_read_flow_volume_liters(dev, &liters);
        int64_t t_us = esp_timer_get_time();

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "VALIDACAO: falha ao ler totalizador: %s", esp_err_to_name(err));
        } else {
            if (!isfinite(baseline_liters)) {
                baseline_liters = liters;
            }

            if (isfinite(last_liters) && last_t_us > 0 && t_us > last_t_us) {
                const double dt_s = (double)(t_us - last_t_us) / 1000000.0;
                const double dL_raw = liters - last_liters;
                double dL_used = dL_raw;
                bool neg_glitch_clamped = false;

                const double neg_glitch_l = ((double)AS6031_VALIDATE_NEGATIVE_GLITCH_ML) / 1000.0;
                if (dL_used < 0.0 && fabs(dL_used) <= neg_glitch_l) {
                    // Em alguns mapas de memoria, a atualizacao INT/FRACTION pode nao ser atomica.
                    dL_used = 0.0;
                    neg_glitch_clamped = true;
                }

                double lpm_raw = 0.0;
                double lpm = 0.0;
                if (dt_s > 0) {
                    lpm_raw = (dL_raw / dt_s) * 60.0;
                    lpm = (dL_used / dt_s) * 60.0;
                }

                const double no_flow_lpm = ((double)AS6031_VALIDATE_NO_FLOW_THRESHOLD_MLPM) / 1000.0;
                const bool no_flow = fabs(lpm) < no_flow_lpm;
                if (no_flow) {
                    lpm = 0.0;
                }

                if (dL_used > 0.0) {
                    positive_accum_liters += dL_used;
                }

                double test_delta_l = liters - baseline_liters;
                if (test_delta_l < 0.0 && fabs(test_delta_l) <= neg_glitch_l) {
                    test_delta_l = 0.0;
                }

                ESP_LOGI(TAG,
                         "VALIDACAO: total=%.6f L  teste=%.6f L  teste_pos=%.6f L  delta_raw=%.6f L  delta=%.6f L  dt=%.3fs  taxa_raw=%.3f L/min  taxa=%.3f L/min  estado=%s%s",
                         liters,
                         test_delta_l,
                         positive_accum_liters,
                         dL_raw,
                         dL_used,
                         dt_s,
                         lpm_raw,
                         lpm,
                         no_flow ? "SEM_FLUXO" : "FLUXO",
                         neg_glitch_clamped ? " (glitch_neg_filtrado)" : "");
            } else {
                ESP_LOGI(TAG, "VALIDACAO: total=%.6f L (primeira amostra, baseline)", liters);
            }
            last_liters = liters;
            last_t_us = t_us;
        }

        vTaskDelay(pdMS_TO_TICKS(AS6031_VALIDATE_INTERVAL_MS));
    }
}
#endif

#define DAILY_WAKEUP_US ((uint64_t)AS6031_DAILY_WAKEUP_SECONDS * 1000000ULL)
#define MQTT_BROKER_URI         "mqtts://broker.exemplo.local:8883"
#define MQTT_TOPIC_PREFIX       "londrina"
#define GATEWAY_BAIRRO          "centro"
#define GATEWAY_SETOR           "setor-a"

#define SENSOR_MSG_VERSION      1U
#define SENSOR_MSG_TYPE_DATA    1U
#define SENSOR_MSG_TYPE_ACK     2U
#define MAX_ACK_WAIT_MS         300
#define MAX_SEND_RETRIES        3

// Status do node para telemetria/diagnostico remoto
typedef enum {
    NODE_STATUS_OK = 0,
    NODE_STATUS_ADDR_NOT_CONFIGURED = 1,
    NODE_STATUS_SPI_READ_FAIL = 2,
    NODE_STATUS_TOTAL_REGRESSED = 3,
    NODE_STATUS_FW_INFO_CHANGED = 4,
    NODE_STATUS_DELTA_TOO_LARGE = 5,
    NODE_STATUS_NVS_ERROR = 6,
} node_status_code_t;

// NVS keys
#define NVS_NS_METER "meter"
#define NVS_KEY_TOTAL_L "total_l"
#define NVS_KEY_RAW_L "raw_l"
#define NVS_KEY_FWU_RNG "fwu_rng"
#define NVS_KEY_FWU_REV "fwu_rev"
#define NVS_KEY_FWA_REV "fwa_rev"

// --- Persistência em Deep Sleep (RTC RAM) ---
RTC_DATA_ATTR double   total_volume_liters = 0;
RTC_DATA_ATTR int64_t  last_time_stamp = 0;
RTC_DATA_ATTR int64_t  last_transmission = 0;
RTC_DATA_ATTR uint32_t tx_sequence = 0;
RTC_DATA_ATTR uint16_t last_node_status = NODE_STATUS_OK;
RTC_DATA_ATTR uint32_t last_fwu_rng = 0;
RTC_DATA_ATTR uint32_t last_fwu_rev = 0;
RTC_DATA_ATTR uint32_t last_fwa_rev = 0;
RTC_DATA_ATTR double   last_raw_totalizer_l = -1.0;
RTC_DATA_ATTR int8_t   totalizer_direction = 0; // +1 crescente, -1 decrescente, 0 desconhecida
RTC_DATA_ATTR uint8_t  gateway_mac[6] = {0x24, 0x6F, 0x28, 0x00, 0x00, 0x01}; // Ajustar para MAC real da Central

// Estrutura de Mensagem para o Broker MQTT (via Gateway)
// v1 (legado): sem reset_reason/wakeup_cause
typedef struct __attribute__((packed)) {
    uint8_t  version;
    uint8_t  msg_type;
    uint16_t payload_len;
    uint32_t device_id;
    uint32_t sequence;
    float    volume_total;
    uint16_t battery_mv;
    uint16_t status_code;
    uint16_t crc16;
} sensor_msg_v1_t;

typedef struct __attribute__((packed)) {
    uint8_t  version;
    uint8_t  msg_type;
    uint16_t payload_len;
    uint32_t device_id;
    uint32_t sequence;
    float    volume_total;
    uint16_t battery_mv;
    uint16_t status_code;
    uint8_t  reset_reason;
    uint8_t  wakeup_cause;
    uint16_t crc16;
} sensor_msg_t;

typedef struct __attribute__((packed)) {
    uint8_t  version;
    uint8_t  msg_type;
    uint16_t payload_len;
    uint32_t device_id;
    uint32_t sequence;
    uint8_t  status;
    uint16_t crc16;
} gateway_ack_t;

spi_device_handle_t spi_handle;

static bool g_send_done = false;
static esp_now_send_status_t g_last_send_status = ESP_NOW_SEND_FAIL;
static bool g_ack_received = false;
static bool g_ack_valid = false;
static uint32_t g_last_sent_sequence = 0;

#if !DEVICE_ROLE_NODE
typedef struct {
    uint32_t device_id;
    uint32_t sequence;
    float volume_total;
    uint16_t battery_mv;
    uint16_t status_code;
    uint8_t reset_reason;
    uint8_t wakeup_cause;
    int8_t rssi;
} gateway_rx_event_t;

static QueueHandle_t g_gateway_queue = NULL;
static esp_mqtt_client_handle_t g_mqtt_client = NULL;
#endif

#if SOC_PM_SUPPORT_EXT0_WAKEUP
static bool is_rtc_gpio(gpio_num_t gpio)
{
    return rtc_gpio_is_valid_gpio(gpio);
}
#endif

static bool is_broadcast_mac(const uint8_t mac[6])
{
    return mac[0] == 0xFF && mac[1] == 0xFF && mac[2] == 0xFF &&
           mac[3] == 0xFF && mac[4] == 0xFF && mac[5] == 0xFF;
}

static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

#if DEVICE_ROLE_NODE
static void on_espnow_send(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    g_last_send_status = status;
    g_send_done = true;
}

static void on_espnow_recv(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int len)
{
    (void)esp_now_info;
    if (data == NULL || len != (int)sizeof(gateway_ack_t)) {
        return;
    }

    gateway_ack_t ack;
    memcpy(&ack, data, sizeof(ack));

    if (ack.version != SENSOR_MSG_VERSION || ack.msg_type != SENSOR_MSG_TYPE_ACK) {
        return;
    }

    uint16_t recv_crc = ack.crc16;
    ack.crc16 = 0;
    uint16_t calc_crc = crc16_ccitt((const uint8_t *)&ack, sizeof(ack));
    if (recv_crc != calc_crc) {
        ESP_LOGW(TAG, "ACK com CRC invalido");
        return;
    }

    if (ack.sequence == g_last_sent_sequence && ack.status == 1) {
        g_ack_received = true;
        g_ack_valid = true;
    }
}
#endif

#if !DEVICE_ROLE_NODE
static void build_telemetry_topic(char *out, size_t out_len, uint32_t device_id)
{
    snprintf(out, out_len, "%s/%s/%s/%lu/telemetry",
             MQTT_TOPIC_PREFIX,
             GATEWAY_BAIRRO,
             GATEWAY_SETOR,
             (unsigned long)device_id);
}

static void gateway_publish_event(const gateway_rx_event_t *evt)
{
    if (g_mqtt_client == NULL) {
        return;
    }

    char topic[128] = {0};
    char payload[256] = {0};

    build_telemetry_topic(topic, sizeof(topic), evt->device_id);
    snprintf(payload,
             sizeof(payload),
             "{\"protocolVersion\":%u,\"deviceId\":%lu,\"sequence\":%lu,\"volumeTotalLiters\":%.3f,\"batteryMv\":%u,\"statusCode\":%u,\"resetReason\":%u,\"wakeupCause\":%u,\"gatewayId\":\"gw-%s-%s\",\"rssi\":%d}",
             SENSOR_MSG_VERSION,
             (unsigned long)evt->device_id,
             (unsigned long)evt->sequence,
             evt->volume_total,
             evt->battery_mv,
             evt->status_code,
             (unsigned)evt->reset_reason,
             (unsigned)evt->wakeup_cause,
             GATEWAY_BAIRRO,
             GATEWAY_SETOR,
             evt->rssi);

    int msg_id = esp_mqtt_client_publish(g_mqtt_client, topic, payload, 0, 1, 0);
    if (msg_id < 0) {
        ESP_LOGW(TAG, "Falha ao publicar MQTT para device=%lu", (unsigned long)evt->device_id);
    }
}

static void gateway_worker_task(void *arg)
{
    (void)arg;
    gateway_rx_event_t evt;
    while (1) {
        if (xQueueReceive(g_gateway_queue, &evt, portMAX_DELAY) == pdTRUE) {
            gateway_publish_event(&evt);
        }
    }
}

static void on_espnow_recv(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int len)
{
    if (esp_now_info == NULL || data == NULL) {
        return;
    }

    // Aceita v1 (legado) e v2 (atual) para facilitar upgrades em campo.
    const bool is_v1 = (len == (int)sizeof(sensor_msg_v1_t));
    const bool is_v2 = (len == (int)sizeof(sensor_msg_t));
    if (!is_v1 && !is_v2) {
        return;
    }

    // Validacao basica do cabecalho e do tamanho declarado.
    uint8_t version = data[0];
    uint8_t msg_type = data[1];
    uint16_t payload_len = 0;
    memcpy(&payload_len, &data[2], sizeof(payload_len));

    bool valid = true;
    if (version != SENSOR_MSG_VERSION || msg_type != SENSOR_MSG_TYPE_DATA || payload_len != (uint16_t)(len - 6)) {
        valid = false;
    }

    // CRC: sempre considerado os ultimos 2 bytes do frame.
    uint16_t recv_crc = 0;
    memcpy(&recv_crc, &data[len - 2], sizeof(recv_crc));
    uint8_t tmp[sizeof(sensor_msg_t)] = {0};
    memcpy(tmp, data, (size_t)len);
    tmp[len - 2] = 0;
    tmp[len - 1] = 0;
    uint16_t calc_crc = crc16_ccitt(tmp, (size_t)len);
    if (recv_crc != calc_crc) {
        valid = false;
    }

    uint32_t device_id = 0;
    uint32_t sequence = 0;
    float volume_total = 0.0f;
    uint16_t battery_mv = 0;
    uint16_t status_code = 0;
    uint8_t reset_reason = 0;
    uint8_t wakeup_cause = 0;

    if (is_v2) {
        sensor_msg_t msg;
        memcpy(&msg, data, sizeof(msg));
        device_id = msg.device_id;
        sequence = msg.sequence;
        volume_total = msg.volume_total;
        battery_mv = msg.battery_mv;
        status_code = msg.status_code;
        reset_reason = msg.reset_reason;
        wakeup_cause = msg.wakeup_cause;
    } else {
        sensor_msg_v1_t msg;
        memcpy(&msg, data, sizeof(msg));
        device_id = msg.device_id;
        sequence = msg.sequence;
        volume_total = msg.volume_total;
        battery_mv = msg.battery_mv;
        status_code = msg.status_code;
    }

    gateway_ack_t ack = {
        .version = SENSOR_MSG_VERSION,
        .msg_type = SENSOR_MSG_TYPE_ACK,
        .payload_len = sizeof(gateway_ack_t) - 6,
        .device_id = device_id,
        .sequence = sequence,
        .status = valid ? 1 : 0,
        .crc16 = 0,
    };
    ack.crc16 = crc16_ccitt((const uint8_t *)&ack, sizeof(ack));

    esp_now_peer_info_t peer = { .channel = 0, .encrypt = false };
    memcpy(peer.peer_addr, esp_now_info->src_addr, ESP_NOW_ETH_ALEN);
    esp_err_t add_peer_err = esp_now_add_peer(&peer);
    if (add_peer_err != ESP_OK && add_peer_err != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGW(TAG, "Falha ao adicionar peer node: %s", esp_err_to_name(add_peer_err));
    }

    esp_err_t send_err = esp_now_send(esp_now_info->src_addr, (uint8_t *)&ack, sizeof(ack));
    if (send_err != ESP_OK) {
        ESP_LOGW(TAG, "Falha ao enviar ACK: %s", esp_err_to_name(send_err));
    }

    if (!valid || g_gateway_queue == NULL) {
        return;
    }

    gateway_rx_event_t evt = {
        .device_id = device_id,
        .sequence = sequence,
        .volume_total = volume_total,
        .battery_mv = battery_mv,
        .status_code = status_code,
        .reset_reason = reset_reason,
        .wakeup_cause = wakeup_cause,
        .rssi = esp_now_info->rx_ctrl ? esp_now_info->rx_ctrl->rssi : 0,
    };

    if (xQueueSend(g_gateway_queue, &evt, 0) != pdPASS) {
        ESP_LOGW(TAG, "Fila do gateway cheia, telemetria descartada");
    }
}
#endif

static esp_err_t nvs_load_anchor(double *out_total_l,
                                 double *out_raw_l,
                                 uint32_t *out_fwu_rng,
                                 uint32_t *out_fwu_rev,
                                 uint32_t *out_fwa_rev)
{
    if (!out_total_l) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS_METER, NVS_READONLY, &h);
    if (err != ESP_OK) {
        return err;
    }

    double total = 0.0;
    size_t len = sizeof(total);
    err = nvs_get_blob(h, NVS_KEY_TOTAL_L, &total, &len);
    if (err == ESP_OK && len == sizeof(total)) {
        *out_total_l = total;
    }

    if (out_raw_l) {
        double raw = -1.0;
        len = sizeof(raw);
        if (nvs_get_blob(h, NVS_KEY_RAW_L, &raw, &len) == ESP_OK && len == sizeof(raw)) {
            *out_raw_l = raw;
        }
    }

    if (out_fwu_rng) {
        uint32_t v = 0;
        if (nvs_get_u32(h, NVS_KEY_FWU_RNG, &v) == ESP_OK) *out_fwu_rng = v;
    }
    if (out_fwu_rev) {
        uint32_t v = 0;
        if (nvs_get_u32(h, NVS_KEY_FWU_REV, &v) == ESP_OK) *out_fwu_rev = v;
    }
    if (out_fwa_rev) {
        uint32_t v = 0;
        if (nvs_get_u32(h, NVS_KEY_FWA_REV, &v) == ESP_OK) *out_fwa_rev = v;
    }

    nvs_close(h);
    return err;
}

static esp_err_t nvs_store_anchor(double total_l, double raw_l, uint32_t fwu_rng, uint32_t fwu_rev, uint32_t fwa_rev)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS_METER, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_blob(h, NVS_KEY_TOTAL_L, &total_l, sizeof(total_l));
    if (err == ESP_OK) err = nvs_set_blob(h, NVS_KEY_RAW_L, &raw_l, sizeof(raw_l));
    if (err == ESP_OK) err = nvs_set_u32(h, NVS_KEY_FWU_RNG, fwu_rng);
    if (err == ESP_OK) err = nvs_set_u32(h, NVS_KEY_FWU_REV, fwu_rev);
    if (err == ESP_OK) err = nvs_set_u32(h, NVS_KEY_FWA_REV, fwa_rev);
    if (err == ESP_OK) err = nvs_commit(h);

    nvs_close(h);
    return err;
}

#if AS6031_MAINTENANCE_MODE
static void as6031_maintenance_loop(spi_device_handle_t dev)
{
    ESP_LOGW(TAG, "MANUTENCAO: modo ativo (sem deep sleep / sem TX), intervalo=%ums", (unsigned)AS6031_MAINTENANCE_INTERVAL_MS);
    uint32_t fwu_rng = 0, fwu_rev = 0, fwa_rev = 0;
    if (as6031_read_firmware_info(dev, &fwu_rng, &fwu_rev, &fwa_rev) == ESP_OK) {
        ESP_LOGI(TAG, "MANUTENCAO: AS6031 FWU_RNG=0x%08lX FWU_REV=0x%08lX FWA_REV=0x%08lX",
                 (unsigned long)fwu_rng, (unsigned long)fwu_rev, (unsigned long)fwa_rev);
    } else {
        ESP_LOGW(TAG, "MANUTENCAO: nao foi possivel ler FW info");
    }

    double last_liters = NAN;
    int64_t last_t_us = 0;
    while (1) {
        double liters = 0.0;
        esp_err_t err = as6031_read_flow_volume_liters(dev, &liters);
        int64_t t_us = esp_timer_get_time();

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "MANUTENCAO: falha ao ler total: %s", esp_err_to_name(err));
        } else {
            if (isfinite(last_liters) && last_t_us > 0 && t_us > last_t_us) {
                const double dt_s = (double)(t_us - last_t_us) / 1000000.0;
                const double dL = liters - last_liters;
                const double lpm = dt_s > 0 ? (dL / dt_s) * 60.0 : 0.0;
                ESP_LOGI(TAG, "MANUTENCAO: total=%.6f L  delta=%.6f L  dt=%.3fs  taxa=%.3f L/min", liters, dL, dt_s, lpm);
            } else {
                ESP_LOGI(TAG, "MANUTENCAO: total=%.6f L (primeira amostra)", liters);
            }
            last_liters = liters;
            last_t_us = t_us;
        }
        vTaskDelay(pdMS_TO_TICKS(AS6031_MAINTENANCE_INTERVAL_MS));
    }
}
#endif

// --- Inicialização do SPI ---
void init_spi() {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_MISO, .mosi_io_num = PIN_MOSI, .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1, .quadhd_io_num = -1
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1500000, .mode = 1, .spics_io_num = PIN_CS, .queue_size = 7
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle));
}

// --- Leitura do AS6031 ---
void read_as6031(double *diff_ns) {
    if (diff_ns == NULL) {
        return;
    }

    *diff_ns = 0.0;

    uint32_t sum_up = 0;
    uint32_t sum_down = 0;

    esp_err_t err = as6031_send_remote_cmd(spi_handle, AS6031_RC_BM_REQ);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TOF: BM_REQ falhou: %s", esp_err_to_name(err));
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(1));

    err = as6031_read_u32(spi_handle, 0x080u, &sum_up);
    if (err == ESP_OK) {
        err = as6031_read_u32(spi_handle, 0x084u, &sum_down);
    }

    const uint32_t clr_irq_flag = 0x00000001u;
    (void)as6031_raawr_dwords(spi_handle, 0x0DDu, &clr_irq_flag, 1, false, NULL);
    (void)as6031_send_remote_cmd(spi_handle, AS6031_RC_BM_RLS);

    if (err != ESP_OK || sum_up == 0 || sum_up == 0xFFFFFFFFu) {
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "TOF: leitura falhou: %s", esp_err_to_name(err));
        }
        return;
    }

    double lsb_ns = 125.0 / 65536.0;
    double up_ns = (sum_up * lsb_ns) / 10.0;
    double down_ns = (sum_down * lsb_ns) / 10.0;
    *diff_ns = up_ns - down_ns;
}

// --- Cálculo de Volume ---
double calculate_instant_volume(double dTOF_ns, int64_t dt_us) {
    double area = PI * pow((D / 2.0), 2);
    double speed = (dTOF_ns * 1e-9 * (CO * CO)) / (2.0 * L);
    double flow_ls = speed * area * 1000.0;
    return flow_ls * (dt_us / 1000000.0);
}

#if AS6031_COMMISSIONING_TOF_MODE
static double commissioning_median_diff(double *buf, int count)
{
    if (count <= 0) {
        return 0.0;
    }

    double sorted[5] = {0};
    for (int i = 0; i < count; i++) {
        sorted[i] = buf[i];
    }

    for (int i = 0; i < count - 1; i++) {
        for (int j = i + 1; j < count; j++) {
            if (sorted[i] > sorted[j]) {
                double tmp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = tmp;
            }
        }
    }

    if ((count % 2) == 0) {
        return (sorted[(count / 2) - 1] + sorted[count / 2]) * 0.5;
    }
    return sorted[count / 2];
}

static void as6031_commissioning_tof_loop(void)
{
    ESP_LOGW(TAG, "COMISSIONAMENTO TOF ativo (Caminho B)");
    ESP_LOGW(TAG, "Este modo nao entra em deep sleep e nao faz TX ESP-NOW");
    ESP_LOGW(TAG, "Intervalo=%ums, limiar=%d mL/s",
             (unsigned)AS6031_COMMISSIONING_INTERVAL_MS,
             (int)AS6031_COMMISSIONING_FLOW_THRESHOLD_MLPS);

    init_spi();

    esp_err_t setup_err = as6031_apply_reference_tof_setup(spi_handle);
    if (setup_err != ESP_OK) {
        ESP_LOGW(TAG, "COMISSIONAMENTO: setup de referencia falhou: %s", esp_err_to_name(setup_err));
    }

    double total_liters = 0.0;
    int64_t last_t_us = esp_timer_get_time();
    double diff_buf[5] = {0};
    int diff_idx = 0;
    bool diff_filled = false;
    const double min_flow_ls = ((double)AS6031_COMMISSIONING_FLOW_THRESHOLD_MLPS) / 1000.0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(AS6031_COMMISSIONING_INTERVAL_MS));

        int64_t now_us = esp_timer_get_time();
        int64_t dt_us = now_us - last_t_us;
        last_t_us = now_us;
        if (dt_us <= 0) {
            continue;
        }

        double dTOF_ns = 0.0;
        read_as6031(&dTOF_ns);

        if (fabs(dTOF_ns) > 200.0) {
            ESP_LOGW(TAG, "COMISSIONAMENTO: dTOF fora de faixa (%.2f ns)", dTOF_ns);
            continue;
        }

        diff_buf[diff_idx] = dTOF_ns;
        diff_idx = (diff_idx + 1) % 5;
        if (diff_idx == 0) {
            diff_filled = true;
        }
        int sample_count = diff_filled ? 5 : diff_idx;
        if (sample_count == 0) {
            sample_count = 1;
        }

        double filtered_diff_ns = commissioning_median_diff(diff_buf, sample_count);
        double delta_l = calculate_instant_volume(filtered_diff_ns, dt_us);
        double dt_s = (double)dt_us / 1000000.0;
        double flow_ls = dt_s > 0.0 ? (delta_l / dt_s) : 0.0;

        if (fabs(flow_ls) < min_flow_ls) {
            flow_ls = 0.0;
            delta_l = 0.0;
        } else {
            total_liters += delta_l;
        }

        ESP_LOGI(TAG,
                 "COMISSIONAMENTO: dTOF=%.2f ns, Q=%.4f L/s, total=%.4f L",
                 filtered_diff_ns,
                 flow_ls,
                 total_liters);
    }
}
#endif

// --- Comunicação Mesh (ESP-NOW) ---
static esp_err_t init_radio_stack(void)
{
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_netif_init falhou: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "event_loop falhou: %s", esp_err_to_name(err));
        return err;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init falhou: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode falhou: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start falhou: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init falhou: %s", esp_err_to_name(err));
        return err;
    }

#if DEVICE_ROLE_NODE
    err = esp_now_register_send_cb(on_espnow_send);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "send_cb falhou: %s", esp_err_to_name(err));
        return err;
    }
#endif

    err = esp_now_register_recv_cb(on_espnow_recv);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "recv_cb falhou: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

static void deinit_radio_stack(void)
{
#if DEVICE_ROLE_NODE
    esp_now_unregister_send_cb();
#endif
    esp_now_unregister_recv_cb();
    esp_now_deinit();
    esp_wifi_stop();
    esp_wifi_deinit();
    esp_event_loop_delete_default();
}

bool transmit_data() {
    if (is_broadcast_mac(gateway_mac)) {
        ESP_LOGE(TAG, "Gateway MAC em broadcast, ACK real indisponivel");
        return false;
    }

    if (init_radio_stack() != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar radio, envio abortado");
        return false;
    }

    esp_now_peer_info_t peer = { .channel = 0, .encrypt = false };
    memcpy(peer.peer_addr, gateway_mac, 6);
    if (esp_now_add_peer(&peer) != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao adicionar peer ESP-NOW");
        deinit_radio_stack();
        return false;
    }

    sensor_msg_t msg = {
        .version = SENSOR_MSG_VERSION,
        .msg_type = SENSOR_MSG_TYPE_DATA,
        .payload_len = sizeof(sensor_msg_t) - 6,
        .device_id = 0xABCD,
        .sequence = ++tx_sequence,
        .volume_total = (float)total_volume_liters,
        .battery_mv = 3600,
        .status_code = last_node_status,
        .reset_reason = (uint8_t)esp_reset_reason(),
        .wakeup_cause = (uint8_t)esp_sleep_get_wakeup_cause(),
        .crc16 = 0
    };
    msg.crc16 = crc16_ccitt((const uint8_t *)&msg, sizeof(msg));

    g_last_sent_sequence = msg.sequence;

    bool delivered = false;
    for (int attempt = 1; attempt <= MAX_SEND_RETRIES && !delivered; ++attempt) {
        ESP_LOGI(TAG, "Tentativa ESP-NOW %d/%d seq=%lu", attempt, MAX_SEND_RETRIES, (unsigned long)msg.sequence);

        g_send_done = false;
        g_last_send_status = ESP_NOW_SEND_FAIL;
        g_ack_received = false;
        g_ack_valid = false;

        esp_err_t send_err = esp_now_send(gateway_mac, (uint8_t *)&msg, sizeof(msg));
        if (send_err != ESP_OK) {
            ESP_LOGE(TAG, "Falha no esp_now_send: %s", esp_err_to_name(send_err));
            vTaskDelay(pdMS_TO_TICKS(80 * attempt));
            continue;
        }

        for (int i = 0; i < 20 && !g_send_done; ++i) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (!g_send_done || g_last_send_status != ESP_NOW_SEND_SUCCESS) {
            ESP_LOGW(TAG, "Envio sem confirmacao de camada MAC");
            vTaskDelay(pdMS_TO_TICKS(100 * attempt));
            continue;
        }

        int ack_ticks = MAX_ACK_WAIT_MS / 10;
        for (int i = 0; i < ack_ticks && !g_ack_received; ++i) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (g_ack_received && g_ack_valid) {
            delivered = true;
            ESP_LOGI(TAG, "ACK do gateway confirmado para seq=%lu", (unsigned long)msg.sequence);
        } else {
            ESP_LOGW(TAG, "Timeout/invalid ACK para seq=%lu", (unsigned long)msg.sequence);
            vTaskDelay(pdMS_TO_TICKS(120 * attempt));
        }
    }

    deinit_radio_stack();

    return delivered;
}

#if !DEVICE_ROLE_NODE
static esp_err_t init_gateway_mqtt(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };
    g_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (g_mqtt_client == NULL) {
        ESP_LOGE(TAG, "Falha ao criar cliente MQTT");
        return ESP_FAIL;
    }

    esp_err_t err = esp_mqtt_client_start(g_mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao iniciar MQTT: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}
#endif

void app_main(void) {
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

#if AS6031_DISCOVERY_MODE
    as6031_discovery_log_persisted_summary();
#endif

#if CONFIG_ESPNOW_TEST_MODE
    espnow_test_run();
    return;
#endif

#if AS6031_COMMISSIONING_TOF_MODE
    as6031_commissioning_tof_loop();
    return;
#endif

#if DEVICE_ROLE_NODE
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
        ESP_LOGW(TAG,
                 "Wakeup cause=UNDEFINED (power-on/reset). Para validar continuidade do totalizador interno, use wake por timer (sem power-cycle da placa).");
    }

    // Carrega ancora de NVS (se existir). Ajuda na recuperacao apos perda total de energia.
    double nvs_total = 0.0;
    double nvs_raw_l = -1.0;
    uint32_t nvs_fwu_rng = 0, nvs_fwu_rev = 0, nvs_fwa_rev = 0;
    esp_err_t anchor_err = nvs_load_anchor(&nvs_total, &nvs_raw_l, &nvs_fwu_rng, &nvs_fwu_rev, &nvs_fwa_rev);
    if (anchor_err == ESP_OK) {
        // Atualiza RTC se parecer que perdemos estado (ou em cold boot).
        if (cause == ESP_SLEEP_WAKEUP_UNDEFINED || (total_volume_liters <= 0.0 && nvs_total > 0.0)) {
            total_volume_liters = nvs_total;
        }
        if ((cause == ESP_SLEEP_WAKEUP_UNDEFINED || last_raw_totalizer_l < 0.0) && nvs_raw_l >= 0.0) {
            last_raw_totalizer_l = nvs_raw_l;
        }
        if (cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
            last_fwu_rng = nvs_fwu_rng;
            last_fwu_rev = nvs_fwu_rev;
            last_fwa_rev = nvs_fwa_rev;
        }
    }

    if (isfinite(total_volume_liters) && total_volume_liters > AS6031_MIGRATION_MAX_TOTAL_LITERS) {
        ESP_LOGW(TAG,
                 "Acumulado legado implausivel (%.3f L). Reinicializando acumulado para migracao de escala.",
                 total_volume_liters);
        total_volume_liters = 0.0;
    }

    init_spi();

#if AS6031_INTERNAL_TOTALIZER
    // Modo desejado: AS6031 roda autonomamente (Flow Meter Mode) e o ESP acorda somente 1x/dia.
    // Neste modo, o ESP não depende da interrupção do AS6031 para acumular volume.

    // Primeiro, valida leitura de FW para decidir se deve forcar SYS_INIT.
    uint32_t fwu_rng = 0, fwu_rev = 0, fwa_rev = 0;
    bool fw_info_valid = (as6031_read_firmware_info(spi_handle, &fwu_rng, &fwu_rev, &fwa_rev) == ESP_OK);
    const bool fw_all_ff = fw_info_valid &&
                           (fwu_rng == 0xFFFFFFFFu && fwu_rev == 0xFFFFFFFFu && fwa_rev == 0xFFFFFFFFu);
    const bool need_sys_init = (AS6031_FORCE_SYS_INIT_ON_BOOT != 0) || !fw_info_valid || fw_all_ff;

    esp_err_t fm_err = as6031_prepare_flow_meter_mode(spi_handle, need_sys_init);
    if (fm_err != ESP_OK) {
        ESP_LOGW(TAG, "Falha ao preparar Flow Meter Mode: %s", esp_err_to_name(fm_err));
        last_node_status = NODE_STATUS_SPI_READ_FAIL;
    }

    // Sempre que acordar, tenta ler FW info e detectar reset/troca.
    // Se vier all-FF, força um SYS_INIT e tenta novamente no mesmo ciclo.
    bool fw_read_ok = false;
    for (int fw_try = 0; fw_try < 2; fw_try++) {
        if (as6031_read_firmware_info(spi_handle, &fwu_rng, &fwu_rev, &fwa_rev) != ESP_OK) {
            if (fw_try == 0) {
                ESP_LOGW(TAG, "FW info indisponivel; tentando recuperar com SYS_INIT");
                (void)as6031_prepare_flow_meter_mode(spi_handle, true);
                continue;
            }
            break;
        }

        const bool fw_all_ff_now = (fwu_rng == 0xFFFFFFFFu && fwu_rev == 0xFFFFFFFFu && fwa_rev == 0xFFFFFFFFu);
        if (fw_all_ff_now && fw_try == 0) {
            ESP_LOGW(TAG, "FW info all-FF; tentando recuperar com SYS_INIT");
            (void)as6031_prepare_flow_meter_mode(spi_handle, true);
            continue;
        }

        fw_read_ok = !fw_all_ff_now;
        break;
    }

    if (fw_read_ok) {
        ESP_LOGI(TAG, "AS6031 FWU_RNG=0x%08lX FWU_REV=0x%08lX FWA_REV=0x%08lX",
                 (unsigned long)fwu_rng, (unsigned long)fwu_rev, (unsigned long)fwa_rev);
        if ((last_fwu_rng || last_fwu_rev || last_fwa_rev) &&
            (fwu_rng != last_fwu_rng || fwu_rev != last_fwu_rev || fwa_rev != last_fwa_rev)) {
            ESP_LOGW(TAG, "Anomalia: FW info mudou (possivel reset/troca de firmware)");
            last_node_status = NODE_STATUS_FW_INFO_CHANGED;
        }
        last_fwu_rng = fwu_rng;
        last_fwu_rev = fwu_rev;
        last_fwa_rev = fwa_rev;
    } else {
        ESP_LOGW(TAG, "Nao foi possivel ler informacoes de firmware validas do AS6031");
        last_node_status = NODE_STATUS_SPI_READ_FAIL;
    }

#if AS6031_MAINTENANCE_MODE
    // Modo manutencao: nao transmite e nao entra em deep sleep.
    if (AS6031_ADDR_RAM_FLOW_VOLUME_INT != 0xFFFFu && AS6031_ADDR_RAM_FLOW_VOLUME_FRACTION != 0xFFFFu) {
        as6031_maintenance_loop(spi_handle);
    }
#endif

    // Se os enderecos nao estao definidos, entra em modo discovery para ajudar a encontrar os offsets.
#if AS6031_DISCOVERY_MODE
    if (AS6031_ADDR_RAM_FLOW_VOLUME_INT == 0xFFFFu || AS6031_ADDR_RAM_FLOW_VOLUME_FRACTION == 0xFFFFu) {
        esp_err_t ref_err = as6031_apply_reference_tof_setup(spi_handle);
        if (ref_err != ESP_OK) {
            ESP_LOGW(TAG, "DISCOVERY: setup de referencia falhou: %s", esp_err_to_name(ref_err));
        }

        as6031_discovery_scan(spi_handle);

        const int repeats = (int)AS6031_DISCOVERY_REPEAT_COUNT;
        const int rounds = (repeats <= 0) ? 1 : repeats;
        for (int i = 1; i < rounds; i++) {
            vTaskDelay(pdMS_TO_TICKS(10000));
            as6031_discovery_scan(spi_handle);
        }

        ESP_LOGW(TAG, "DISCOVERY: finalizado (%d rodada(s)); seguindo fluxo normal (status de endereco nao configurado).", rounds);
    }
#endif

    // Se os enderecos estao definidos, pode entrar em modo de validacao (loop de leitura + L/min).
#if AS6031_VALIDATE_MODE
    if (AS6031_ADDR_RAM_FLOW_VOLUME_INT != 0xFFFFu && AS6031_ADDR_RAM_FLOW_VOLUME_FRACTION != 0xFFFFu) {
        as6031_validate_totalizer_loop(spi_handle);
    }
#endif

    // IMPORTANTE: em deep sleep o chip reinicia; esp_timer_get_time() volta a 0.
    // Como este modo acorda apenas por TIMER (1 dia), basta transmitir sempre que acordar por TIMER.
    const bool should_transmit = (cause == ESP_SLEEP_WAKEUP_TIMER) || (cause == ESP_SLEEP_WAKEUP_UNDEFINED);

    if (should_transmit) {
        uint16_t status = last_node_status;
        const bool has_raw_anchor = isfinite(last_raw_totalizer_l) && (last_raw_totalizer_l >= 0.0);
        const bool needs_baseline_anchor =
            !isfinite(total_volume_liters) ||
            (total_volume_liters < 0.0) ||
            !has_raw_anchor;

        if (AS6031_ADDR_RAM_FLOW_VOLUME_INT == 0xFFFFu || AS6031_ADDR_RAM_FLOW_VOLUME_FRACTION == 0xFFFFu) {
            status = NODE_STATUS_ADDR_NOT_CONFIGURED;
        } else {
            double read_l = 0.0;
            esp_err_t vol_err = as6031_read_flow_volume_liters(spi_handle, &read_l);
            if (vol_err == ESP_ERR_INVALID_RESPONSE) {
                ESP_LOGW(TAG, "Leitura invalida do totalizador; tentando recuperar com SYS_INIT");
                if (as6031_prepare_flow_meter_mode(spi_handle, true) == ESP_OK) {
                    vol_err = as6031_read_flow_volume_liters(spi_handle, &read_l);
                }
            }
            if (vol_err != ESP_OK) {
                ESP_LOGE(TAG, "Falha ao ler volume interno do AS6031: %s", esp_err_to_name(vol_err));
                status = NODE_STATUS_SPI_READ_FAIL;
            } else {
                const double eps = 0.0005; // 0.5 mL
                if (needs_baseline_anchor) {
                    // Primeiro ciclo valido: ancora leitura bruta e preserva acumulado conhecido.
                    last_raw_totalizer_l = read_l;
                    totalizer_direction = 0;
                    if (!isfinite(total_volume_liters) || total_volume_liters < 0.0) {
                        total_volume_liters = 0.0;
                    }
                    status = (status == NODE_STATUS_FW_INFO_CHANGED) ? NODE_STATUS_FW_INFO_CHANGED : NODE_STATUS_OK;
                    ESP_LOGW(TAG,
                             "Flow Meter: baseline bruto ancorado em %.6f L (acumulado=%.6f L)",
                             last_raw_totalizer_l,
                             total_volume_liters);
                } else {
                    const double raw_prev = last_raw_totalizer_l;
                    const double raw_delta = read_l - raw_prev;
                    const double raw_ratio =
                        (read_l > eps && raw_prev > eps)
                            ? ((raw_prev > read_l) ? (raw_prev / read_l) : (read_l / raw_prev))
                            : 1.0;
                    const bool legacy_anchor_mismatch = (raw_ratio >= AS6031_MIGRATION_REANCHOR_RATIO);
                    const bool previous_cycle_failed =
                        (last_node_status == NODE_STATUS_SPI_READ_FAIL) ||
                        (last_node_status == NODE_STATUS_FW_INFO_CHANGED) ||
                        (last_node_status == NODE_STATUS_DELTA_TOO_LARGE) ||
                        (last_node_status == NODE_STATUS_TOTAL_REGRESSED);

                    if (totalizer_direction == 0 && fabs(raw_delta) > eps) {
                        totalizer_direction = (raw_delta > 0.0) ? 1 : -1;
                        ESP_LOGW(TAG,
                                 "Flow Meter: direcao do totalizador autodetectada: %s",
                                 (totalizer_direction > 0) ? "crescente" : "decrescente");
                    }

                    int8_t effective_direction = totalizer_direction;
                    if (effective_direction == 0) {
#if AS6031_TOTALIZER_COUNTER_DECREASING
                        effective_direction = -1;
#else
                        effective_direction = 1;
#endif
                    }

                    double delta_l = (effective_direction > 0) ? raw_delta : (-raw_delta);

                    if (fabs(raw_delta) > (double)AS6031_MAX_DAILY_DELTA_LITERS) {
                        if (previous_cycle_failed || legacy_anchor_mismatch) {
                            // Recupera automaticamente quando ha salto apos falha previa ou mismatch de ancora.
                            last_raw_totalizer_l = read_l;
                            totalizer_direction = 0;
                            if (!isfinite(total_volume_liters) || total_volume_liters > AS6031_MIGRATION_MAX_TOTAL_LITERS) {
                                total_volume_liters = 0.0;
                            }
                            status = (status == NODE_STATUS_FW_INFO_CHANGED) ? NODE_STATUS_FW_INFO_CHANGED : NODE_STATUS_OK;
                            ESP_LOGW(TAG,
                                     "Reancoragem de recuperacao: raw_prev=%.6f raw_now=%.6f ratio=%.2f acumulado=%.6f L",
                                     raw_prev,
                                     read_l,
                                     raw_ratio,
                                     total_volume_liters);
                        } else {
                            ESP_LOGW(TAG,
                                     "Anomalia: delta bruto muito alto (raw_prev=%.6f raw_now=%.6f raw_delta=%.6f L, limite=%d L)",
                                     raw_prev,
                                     read_l,
                                     raw_delta,
                                     (int)AS6031_MAX_DAILY_DELTA_LITERS);
                            status = NODE_STATUS_DELTA_TOO_LARGE;
                        }
                    } else if (delta_l < -eps) {
                        ESP_LOGW(TAG,
                                 "Anomalia: consumo derivado regrediu (raw_prev=%.6f raw_now=%.6f delta=%.6f L)",
                                 raw_prev,
                                 read_l,
                                 delta_l);
                        status = NODE_STATUS_TOTAL_REGRESSED;
                    } else {
                        if (delta_l < 0.0) {
                            delta_l = 0.0;
                        }
                        total_volume_liters += delta_l;
                        last_raw_totalizer_l = read_l;
                        status = (status == NODE_STATUS_FW_INFO_CHANGED) ? NODE_STATUS_FW_INFO_CHANGED : NODE_STATUS_OK;
                        ESP_LOGI(TAG,
                                 "Flow Meter: raw_prev=%.6f raw_now=%.6f delta=%.6f L acumulado=%.6f L",
                                 raw_prev,
                                 read_l,
                                 delta_l,
                                 total_volume_liters);
                    }
                }

                // Persiste ancora em NVS quando houve leitura valida (inclusive baseline inicial).
                if (status == NODE_STATUS_OK || status == NODE_STATUS_FW_INFO_CHANGED) {
                    esp_err_t store_err = nvs_store_anchor(total_volume_liters,
                                                           last_raw_totalizer_l,
                                                           last_fwu_rng,
                                                           last_fwu_rev,
                                                           last_fwa_rev);
                    if (store_err != ESP_OK) {
                        ESP_LOGW(TAG, "Falha ao persistir ancora NVS: %s", esp_err_to_name(store_err));
                        if (status == NODE_STATUS_OK) {
                            status = NODE_STATUS_NVS_ERROR;
                        }
                    }
                }
            }
        }

        last_node_status = status;
        ESP_LOGI(TAG,
                 "AS6031 consumo acumulado (interno): %.4f L (raw=%.6f L status=%u)",
                 total_volume_liters,
                 last_raw_totalizer_l,
                 (unsigned)last_node_status);

        (void)transmit_data();
    } else {
        ESP_LOGI(TAG, "Acordou fora da janela diaria; voltando a dormir");
    }

    // Deep sleep com wakeup apenas por timer.
    ESP_LOGI(TAG,
             "Flow Meter Mode: programando proximo wake em %u s",
             (unsigned)AS6031_DAILY_WAKEUP_SECONDS);
    esp_sleep_enable_timer_wakeup(DAILY_WAKEUP_US);
    esp_deep_sleep_start();
#else
    // Modo legado: acorda por interrupcao do AS6031, calcula volume no ESP e persiste em RTC RAM.
    int64_t now = esp_timer_get_time();
    double dTOF = 0;
    read_as6031(&dTOF);

    bool wake_from_flow_irq = (cause == ESP_SLEEP_WAKEUP_EXT0 || cause == ESP_SLEEP_WAKEUP_GPIO);
    if (wake_from_flow_irq && last_time_stamp > 0) {
        int64_t dt = now - last_time_stamp;
        if (dTOF > -200 && dTOF < 200) {
            total_volume_liters += calculate_instant_volume(dTOF, dt);
        }
    }
    last_time_stamp = now;

    // Envio diário (86400s)
    if ((now - last_transmission) > 86400000000LL || cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
        if (transmit_data()) {
            last_transmission = now;
        }
    }

    ESP_LOGI(TAG, "Volume Acumulado: %.4f L", total_volume_liters);

    // Configuração do sono
#if SOC_PM_SUPPORT_EXT0_WAKEUP
    if (is_rtc_gpio((gpio_num_t)PIN_INT_AS6031)) {
        ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_INT_AS6031, 0));
    } else {
        ESP_LOGW(TAG, "GPIO de interrupcao nao e RTC, ext0 desativado");
    }
#else
    ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(1ULL << PIN_INT_AS6031, ESP_GPIO_WAKEUP_GPIO_LOW));
#endif
    esp_sleep_enable_timer_wakeup(3600ULL * 1000000ULL); // Wakeup de segurança 1h
    esp_deep_sleep_start();
#endif
#else
    ESP_LOGI(TAG, "Inicializando modo Gateway");
    g_gateway_queue = xQueueCreate(32, sizeof(gateway_rx_event_t));
    if (g_gateway_queue == NULL) {
        ESP_LOGE(TAG, "Falha ao criar fila do gateway");
        return;
    }

    ESP_ERROR_CHECK(init_radio_stack());
    if (init_gateway_mqtt() != ESP_OK) {
        ESP_LOGW(TAG, "MQTT indisponivel, gateway seguira com ACK local");
    }

    xTaskCreate(gateway_worker_task, "gw_worker", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Gateway pronto para receber nodes");
#endif
}