#include "meter_node.h"

#include <string.h>
#include <math.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_private/esp_clk.h"   // esp_rtc_get_time_us() — relógio monotônico que persiste no deep sleep

#include "flow_packet.h"
#include "mesh_dedup.h"
#include "as6031_remote.h"

static const char *TAG = "FLOW_METER";

// ---------------------------------------------------------------------------
// Configuração ESP-NOW / Mesh
// ---------------------------------------------------------------------------
#define FLOW_ESPNOW_CHANNEL          1
#define FLOW_METER_MAX_FORWARD_HOPS  4
#define FLOW_METER_DEFAULT_BATTERY_PCT 100U

// ---------------------------------------------------------------------------
// NVS — persistência da sequência entre resets completos
// ---------------------------------------------------------------------------
#define FLOW_METER_NVS_NS      "flow_mesh"
#define FLOW_METER_NVS_KEY_SEQ "seq"

// ---------------------------------------------------------------------------
// Pinos SPI e interrupção do AS6031
// ---------------------------------------------------------------------------
#define PIN_NUM_MISO  GPIO_NUM_13
#define PIN_NUM_MOSI  GPIO_NUM_11
#define PIN_NUM_CLK   GPIO_NUM_12
#define PIN_NUM_CS    GPIO_NUM_10
// PIN_NUM_INT deve ser um GPIO RTC válido para EXT0 (ex: GPIO_NUM_4 no ESP32)
#define PIN_NUM_INT   GPIO_NUM_4

// ---------------------------------------------------------------------------
// Limites de fluxo e temporização do deep sleep
// ---------------------------------------------------------------------------
#define FLOW_THRESHOLD_L_S    0.02        // L/s — abaixo disso considera fluxo zero
#define SLEEP_INTERVAL_TURBO  500000ULL   // 500 ms em µs  (fluxo ativo)
#define SLEEP_INTERVAL_ECO    5000000ULL  // 5 s em µs     (sem fluxo)
#define TIMEOUT_ECO_WAKEUPS   10          // wakeups consecutivos sem fluxo → muda p/ ECO

// ---------------------------------------------------------------------------
// RAM RTC — sobrevive ao deep sleep, é zerada apenas no power-on ou reset hard
// ---------------------------------------------------------------------------
RTC_DATA_ATTR static double   rtc_total_volume_l  = 0.0;   // Volume acumulado (L)
RTC_DATA_ATTR static int64_t  rtc_last_wake_us     = 0;     // Timestamp do último wakeup
RTC_DATA_ATTR static uint32_t rtc_sequence         = 0;     // Sequência de pacotes
RTC_DATA_ATTR static uint32_t rtc_no_flow_count    = 0;     // Wakeups consecutivos sem fluxo
RTC_DATA_ATTR static bool     rtc_turbo_mode       = true;  // Modo de sleep atual
RTC_DATA_ATTR static bool     rtc_cold_boot        = true;  // true até o primeiro deep sleep

// ---------------------------------------------------------------------------
// Variáveis de módulo (RAM normal — reiniciadas a cada wakeup, o que é OK)
// ---------------------------------------------------------------------------
static const uint8_t s_broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t       s_self_mac[6]      = {0};
static mesh_dedup_table_t s_dedup       = {0};
static spi_device_handle_t spi_dev      = NULL;
static volatile bool s_espnow_tx_done   = false;

// ===========================================================================
// NVS — carrega/salva sequência para sobreviver a power-on completo
// (a RTC RAM é zerada no power-on, mas a NVS não)
// ===========================================================================
static esp_err_t nvs_load_sequence(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(FLOW_METER_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }

    uint32_t seq = 0;
    err = nvs_get_u32(h, FLOW_METER_NVS_KEY_SEQ, &seq);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        seq = 0;
        err = ESP_OK;
    }
    if (err == ESP_OK) {
        rtc_sequence = seq;
    }

    nvs_close(h);
    return err;
}

static esp_err_t nvs_store_sequence(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(FLOW_METER_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_u32(h, FLOW_METER_NVS_KEY_SEQ, rtc_sequence);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }

    nvs_close(h);
    return err;
}

// ===========================================================================
// Callbacks ESP-NOW
// ===========================================================================
static void on_espnow_send(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    (void)status;
    s_espnow_tx_done = true;
}

static void on_espnow_recv(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data, int len)
{
    (void)recv_info;
    if (!data || len != (int)sizeof(flow_packet_t)) return;

    flow_packet_t pkt;
    memcpy(&pkt, data, sizeof(pkt));

    if (!flow_packet_validate_crc32(&pkt)) return;
    if (pkt.type != FLOW_PKT_TYPE_METER_DATA) return;

    const int64_t now_ms = esp_timer_get_time() / 1000LL;
    if (mesh_dedup_is_duplicate(&s_dedup, pkt.meter_id,
                                pkt.sequence, pkt.type, now_ms)) return;

    if (pkt.hop_count >= FLOW_METER_MAX_FORWARD_HOPS) return;

    pkt.hop_count++;
    flow_packet_update_crc32(&pkt);
    esp_now_send(s_broadcast_mac, (const uint8_t *)&pkt, sizeof(pkt));
}

// ===========================================================================
// Inicialização Wi-Fi + ESP-NOW
// ===========================================================================
static esp_err_t init_wifi_espnow(void)
{
    esp_err_t err;

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "wifi init");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "wifi storage");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "wifi mode");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start");
    ESP_RETURN_ON_ERROR(esp_wifi_set_channel(FLOW_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE),
                        TAG, "wifi channel");

    ESP_RETURN_ON_ERROR(esp_now_init(), TAG, "esp_now init");
    ESP_RETURN_ON_ERROR(esp_now_register_send_cb(on_espnow_send), TAG, "send cb");
    ESP_RETURN_ON_ERROR(esp_now_register_recv_cb(on_espnow_recv), TAG, "recv cb");

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, s_broadcast_mac, ESP_NOW_ETH_ALEN);
    peer.channel = 0;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    err = esp_now_add_peer(&peer);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) return err;

    return ESP_OK;
}

// ===========================================================================
// Inicialização SPI + AS6031
// ===========================================================================
static esp_err_t init_spi_as6031(bool cold_boot)
{
    // Deep sleep reinicia a CPU, então o handle SPI precisa ser recriado a cada wakeup.
    // O que deve permanecer restrito ao cold boot é apenas o setup do AS6031.
    spi_bus_config_t buscfg = {
        .miso_io_num     = PIN_NUM_MISO,
        .mosi_io_num     = PIN_NUM_MOSI,
        .sclk_io_num     = PIN_NUM_CLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 32,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1500000,
        .mode           = 1,          // CPOL=0, CPHA=1
        .spics_io_num   = PIN_NUM_CS,
        .queue_size     = 1,
    };

    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO),
                        TAG, "spi bus init");
    ESP_RETURN_ON_ERROR(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_dev),
                        TAG, "spi add device");

    if (cold_boot) {
        ESP_LOGI(TAG, "SPI inicializado (cold boot).");

        // Aplica configuração de referência TOF + RC_SYS_RST APENAS no cold boot.
        // Em wakeups o AS6031 continua rodando e o acumulador interno é preservado.
        ESP_RETURN_ON_ERROR(as6031_apply_reference_tof_setup(spi_dev),
                            TAG, "as6031 setup");
        ESP_LOGI(TAG, "AS6031 full setup concluído.");
    } else {
        ESP_LOGI(TAG, "SPI inicializado (wakeup) — preservando acumulador interno do AS6031.");
    }

    return ESP_OK;
}

// ===========================================================================
// Transmissão single-shot: envia um pacote e aguarda conclusão (ou timeout)
// ===========================================================================
static void transmit_packet(double volume_l, double flow_l_s)
{
    flow_packet_t pkt;
    flow_packet_init_empty(&pkt);

    pkt.type        = FLOW_PKT_TYPE_METER_DATA;
    pkt.sequence    = rtc_sequence + 1U;
    memcpy(pkt.meter_id, s_self_mac, sizeof(pkt.meter_id));

    // volume em litros inteiros; delta em mL/s (escalonado × 1000)
    pkt.volume_liters = (uint32_t)volume_l;
    pkt.delta_liters  = (uint32_t)(flow_l_s * 1000.0);

    pkt.timestamp_ms  = esp_timer_get_time() / 1000LL;
    pkt.battery_pct   = FLOW_METER_DEFAULT_BATTERY_PCT;
    pkt.hop_count     = 0;
    flow_packet_update_crc32(&pkt);

    // Registra no dedup local para não reencaminhar o próprio pacote
    mesh_dedup_is_duplicate(&s_dedup, pkt.meter_id,
                            pkt.sequence, pkt.type, pkt.timestamp_ms);

    s_espnow_tx_done = false;
    esp_err_t err = esp_now_send(s_broadcast_mac,
                                 (const uint8_t *)&pkt, sizeof(pkt));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_now_send falhou: %s", esp_err_to_name(err));
        return;
    }

    // Aguarda callback de confirmação (máx. 100 ms)
    const int64_t deadline = esp_timer_get_time() + 100000LL;
    while (!s_espnow_tx_done && esp_timer_get_time() < deadline) {
        vTaskDelay(1);
    }

    rtc_sequence = pkt.sequence;

    // Persiste na NVS apenas a cada 10 pacotes para poupar flash
    if ((rtc_sequence % 10) == 0) {
        if (nvs_store_sequence() != ESP_OK) {
            ESP_LOGW(TAG, "Falha ao persistir sequência");
        }
    }

    ESP_LOGI(TAG, "TX seq=%lu vol=%.2fL flow=%.4fL/s hop=0",
             (unsigned long)pkt.sequence, volume_l, flow_l_s);
}

// ===========================================================================
// Configura e entra em deep sleep
// ===========================================================================
static void enter_deep_sleep(bool turbo)
{
    uint64_t sleep_us = turbo ? SLEEP_INTERVAL_TURBO : SLEEP_INTERVAL_ECO;

    // Para o Wi-Fi antes de dormir para economizar energia
    esp_wifi_stop();
    esp_wifi_deinit();

    // Acorda por sinal do AS6031 (INTN ativo em LOW) OU por timer de segurança
    gpio_pullup_en(PIN_NUM_INT);
    gpio_pulldown_dis(PIN_NUM_INT);
    esp_sleep_enable_ext0_wakeup(PIN_NUM_INT, 0);   // nível LOW = medição pronta
    esp_sleep_enable_timer_wakeup(sleep_us);

    ESP_LOGI(TAG, "Deep sleep: %s (%llu ms)",
             turbo ? "TURBO" : "ECO", sleep_us / 1000ULL);

    esp_deep_sleep_start();
    // — nunca retorna —
}

// ===========================================================================
// Ponto de entrada principal — chamado a cada wakeup do deep sleep
// ===========================================================================
void meter_node_run(void)
{
    // -----------------------------------------------------------------------
    // 1. Diagnóstico de causa do wakeup
    // -----------------------------------------------------------------------
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    bool is_cold_boot = (cause == ESP_SLEEP_WAKEUP_UNDEFINED) || rtc_cold_boot;

    ESP_LOGI(TAG, "Wakeup: causa=%d cold_boot=%d turbo=%d seq=%lu",
             cause, is_cold_boot, rtc_turbo_mode, (unsigned long)rtc_sequence);

    // -----------------------------------------------------------------------
    // 2. Inicialização de módulos (acontece a cada wakeup; periféricos
    //    precisam ser reinicializados após deep sleep)
    // -----------------------------------------------------------------------
    mesh_dedup_init(&s_dedup);
    esp_read_mac(s_self_mac, ESP_MAC_WIFI_STA);

    // Carrega sequência da NVS apenas no cold boot (RTC RAM zerada).
    // Nos wakeups seguintes rtc_sequence persiste na RTC RAM sem custo de flash.
    if (is_cold_boot) {
        if (nvs_load_sequence() != ESP_OK) {
            ESP_LOGW(TAG, "NVS sem sequência salva; iniciando em 0");
            rtc_sequence = 0;
        }
    }

    esp_err_t err = init_spi_as6031(is_cold_boot);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar SPI/AS6031: %s", esp_err_to_name(err));
        enter_deep_sleep(rtc_turbo_mode);
        return;
    }

    ESP_ERROR_CHECK(init_wifi_espnow());

    // Marca que o cold boot foi tratado; persiste em RTC para os próximos wakeups.
    if (is_cold_boot) {
        rtc_cold_boot = false;
    }

    // -----------------------------------------------------------------------
    // 3. Cálculo do delta de tempo desde o último wakeup
    // -----------------------------------------------------------------------
    // esp_rtc_get_time_us() é o único relógio monotônico que sobrevive ao deep sleep.
    // esp_timer_get_time() zera a cada wakeup — não pode ser usado aqui.
    int64_t tempo_atual = (int64_t)esp_rtc_get_time_us();
    double delta_t_sec  = 0.0;

    if (rtc_last_wake_us != 0) {
        delta_t_sec = (double)(tempo_atual - rtc_last_wake_us) / 1e6;
        if (delta_t_sec <= 0.0 || delta_t_sec > 60.0) {
            delta_t_sec = (double)(rtc_turbo_mode ?
                                   SLEEP_INTERVAL_TURBO :
                                   SLEEP_INTERVAL_ECO) / 1e6;
        }
    }
    rtc_last_wake_us = tempo_atual;

    // -----------------------------------------------------------------------
    // 4. Leitura do volume acumulado diretamente da RAM do AS6031
    //    O AS6031 continua medindo enquanto o ESP32 dorme. Ele acumula
    //    o volume internamente em seus registradores de resultado.
    // -----------------------------------------------------------------------

    // as6031_read_flow_volume_liters() já adquire e libera o BM internamente.
    // Não chamar BM_REQ antes — causaria double-acquire.
    double volume_l = 0.0;
    err = as6031_read_flow_volume_liters(spi_dev, &volume_l);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao ler volume do AS6031: %s", esp_err_to_name(err));
        enter_deep_sleep(rtc_turbo_mode);
        return; // nunca alcançado — deep_sleep não retorna
    }

    // Limpa flag de interrupção para que o pino INTN volte para HIGH antes
    // do próximo deep sleep. A função de leitura já liberou o BM internamente,
    // portanto NÃO há BM_REQ aqui — causaria double-acquire.
    as6031_send_remote_cmd(spi_dev, AS6031_RC_IF_CLR);
    as6031_send_remote_cmd(spi_dev, AS6031_RC_BM_RLS); // no-op se já livre; defensivo

    ESP_LOGI(TAG, "Volume AS6031: %.4f L | anterior: %.4f L | dt: %.3f s",
             volume_l, rtc_total_volume_l, delta_t_sec);

    // -----------------------------------------------------------------------
    // 5. Cálculo da vazão instantânea (delta volume / delta tempo)
    // -----------------------------------------------------------------------
    double delta_vol    = volume_l - rtc_total_volume_l;
    double flow_l_s     = (delta_t_sec > 0.0) ? (delta_vol / delta_t_sec) : 0.0;

    // Aceita apenas fluxo positivo acima do limiar (filtra ruído e inversão)
    if (flow_l_s < FLOW_THRESHOLD_L_S) {
        flow_l_s = 0.0;
    }

    // Atualiza a fonte absoluta da verdade (RAM do AS6031 → RTC RAM do ESP32)
    rtc_total_volume_l = volume_l;

    // -----------------------------------------------------------------------
    // 6. Lógica de modo TURBO ↔ ECO
    //    - Com fluxo detectado: TURBO (wakeups a cada 500 ms)
    //    - Sem fluxo por N wakeups consecutivos: ECO (wakeups a cada 5 s)
    // -----------------------------------------------------------------------
    if (flow_l_s > 0.0) {
        rtc_no_flow_count = 0;
        rtc_turbo_mode    = true;
    } else {
        rtc_no_flow_count++;
        if (rtc_turbo_mode && rtc_no_flow_count >= TIMEOUT_ECO_WAKEUPS) {
            rtc_turbo_mode = false;
            ESP_LOGI(TAG, "Sem fluxo por %d wakeups → modo ECO", TIMEOUT_ECO_WAKEUPS);
        }
    }

    // -----------------------------------------------------------------------
    // 7. Transmissão ESP-NOW (single-shot antes do sleep)
    // -----------------------------------------------------------------------
    transmit_packet(volume_l, flow_l_s);

    // -----------------------------------------------------------------------
    // 8. Entra em deep sleep — o AS6031 continua medindo sozinho
    // -----------------------------------------------------------------------
    enter_deep_sleep(rtc_turbo_mode);
}