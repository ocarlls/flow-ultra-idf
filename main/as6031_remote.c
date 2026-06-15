#include "as6031_remote.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "AS6031_REMOTE";

// Conversao equivalente ao modulo calc do sketch Arduino.
static const double AS6031_TOF_NUM = 0.1;
static const double AS6031_LSB_S = (125e-9 / 65536.0);
static const double AS6031_PIPE_AREA = (3.1415926 * (0.025 * 0.025) / 4.0);
static const double AS6031_SPEED_K = ((1482.0 * 1482.0) / (2.0 * 0.085));
static const double AS6031_FLOW_POLARITY = -1.0;

static inline uint8_t rc_opcode_with_addr_msb(uint8_t base_opcode_even, uint16_t addr_dword)
{
    // base_opcode_even must have LSB=0 (e.g., 0x7A, 0x7E, 0x5A, 0x5E)
    // If addr_dword bit8 is set, use odd opcode (LSB=1).
    return (uint8_t)(base_opcode_even | ((addr_dword >> 8) & 0x01u));
}

static inline uint8_t addr_low(uint16_t addr_dword)
{
    return (uint8_t)(addr_dword & 0xFFu);
}

static inline uint32_t be32_to_u32(const uint8_t *b)
{
    return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | (uint32_t)b[3];
}

static inline void u32_to_be32(uint32_t v, uint8_t *b)
{
    b[0] = (uint8_t)(v >> 24);
    b[1] = (uint8_t)(v >> 16);
    b[2] = (uint8_t)(v >> 8);
    b[3] = (uint8_t)(v);
}

static const uint32_t s_ref_config_official[15] = {
    0x48DBA399, 0x00800401, 0x00110110, 0x00000001,
    0x010703FF, 0x20060C08, 0x0101E080, 0x00242020,
    0x006A0804, 0x60160208, 0x000FEA0E, 0x00A0DE41,
    0x95A0C06C, 0x40110000, 0x4027000F
};

static const uint32_t s_ref_reg_d0_fast = 0x00000001;
static const uint32_t s_ref_reg_d1 = 0x00001780;
static const uint32_t s_ref_reg_d2 = 0x00001780;
static const uint32_t s_ref_reg_da = 0x00000021;
static const uint32_t s_ref_reg_db = 0x00000021;
static const uint32_t s_ref_ufr_zero = 0x00000000;

static void as6031_log_candidate_pairs_once(spi_device_handle_t dev)
{
    static bool s_logged_candidates = false;
    if (s_logged_candidates || dev == NULL) {
        return;
    }

    const uint16_t base = 0x084u;
    uint32_t v[5] = {0};

    esp_err_t err = as6031_send_remote_cmd(dev, AS6031_RC_BM_REQ);
    if (err != ESP_OK) {
        return;
    }

    for (size_t i = 0; i < 5; i++) {
        err = as6031_read_u32(dev, (uint16_t)(base + i), &v[i]);
        if (err != ESP_OK) {
            break;
        }
    }

    (void)as6031_send_remote_cmd(dev, AS6031_RC_BM_RLS);
    if (err != ESP_OK) {
        return;
    }

    for (size_t i = 0; i < 4; i++) {
        const uint16_t a0 = (uint16_t)(base + i);
        const uint16_t a1 = (uint16_t)(a0 + 1u);
        const double l_hi_lo = ((double)v[i] + ((double)v[i + 1] / 4294967296.0)) * 1000.0;
        const double l_lo_hi = ((double)v[i + 1] + ((double)v[i] / 4294967296.0)) * 1000.0;
        ESP_LOGW(TAG,
                 "FLOW_DIAG_CAND: pair=0x%03X/0x%03X hi_lo=0x%08lX/0x%08lX L_hi_lo=%.6f L_lo_hi=%.6f",
                 (unsigned)a0,
                 (unsigned)a1,
                 (unsigned long)v[i],
                 (unsigned long)v[i + 1],
                 l_hi_lo,
                 l_lo_hi);
    }

    s_logged_candidates = true;
}

static esp_err_t as6031_read_volume_snapshot(spi_device_handle_t dev,
                                             uint16_t int_addr,
                                             uint16_t frac_addr,
                                             uint32_t *out_int,
                                             uint32_t *out_frac)
{
    uint32_t int_a = 0;
    uint32_t frac_a = 0;
    uint32_t int_b = 0;

    esp_err_t err = as6031_read_u32(dev, int_addr, &int_a);
    if (err != ESP_OK) {
        return err;
    }

    err = as6031_read_u32(dev, frac_addr, &frac_a);
    if (err != ESP_OK) {
        return err;
    }

    err = as6031_read_u32(dev, int_addr, &int_b);
    if (err != ESP_OK) {
        return err;
    }

    if (int_a == int_b) {
        *out_int = int_a;
        *out_frac = frac_a;
    } else {
        // Se a parte inteira mudou entre leituras, reler fracao para montar snapshot consistente.
        uint32_t frac_b = 0;
        err = as6031_read_u32(dev, frac_addr, &frac_b);
        if (err != ESP_OK) {
            return err;
        }
        *out_int = int_b;
        *out_frac = frac_b;
    }

    return ESP_OK;
}

esp_err_t as6031_raard_dwords(spi_device_handle_t dev,
                             uint16_t addr_dword,
                             uint32_t *out_dwords,
                             size_t dword_count,
                             bool status_first,
                             uint8_t *out_status)
{
    if (dev == NULL || out_dwords == NULL || dword_count == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (addr_dword > 0x1FFu) {
        return ESP_ERR_INVALID_ARG;
    }

    // RC_RAA_RD  = 0x7A/0x7B
    // RC_RAA_RDS = 0x7E/0x7F (status first)
    const uint8_t base_cmd = status_first ? 0x7Eu : 0x7Au;
    const uint8_t cmd = rc_opcode_with_addr_msb(base_cmd, addr_dword);

    const size_t status_bytes = status_first ? 1 : 0;
    const size_t data_bytes = 4 * dword_count;
    const size_t total_len = 2 + status_bytes + data_bytes;

    // Keep buffers on stack; typical reads are small.
    uint8_t tx[2 + 1 + 4 * 32] = {0};
    uint8_t rx[2 + 1 + 4 * 32] = {0};

    if (total_len > sizeof(tx)) {
        return ESP_ERR_INVALID_SIZE;
    }

    tx[0] = cmd;
    tx[1] = addr_low(addr_dword);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = total_len * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    esp_err_t err = spi_device_transmit(dev, &t);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "spi_device_transmit read falhou: %s", esp_err_to_name(err));
        return err;
    }

    size_t rx_off = 2;
    if (status_first) {
        if (out_status) {
            *out_status = rx[rx_off];
        }
        rx_off += 1;
    }

    for (size_t i = 0; i < dword_count; i++) {
        out_dwords[i] = be32_to_u32(&rx[rx_off + 4 * i]);
    }

    return ESP_OK;
}

esp_err_t as6031_raawr_dwords(spi_device_handle_t dev,
                             uint16_t addr_dword,
                             const uint32_t *dwords,
                             size_t dword_count,
                             bool status_first,
                             uint8_t *out_status)
{
    if (dev == NULL || dwords == NULL || dword_count == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (addr_dword > 0x1FFu) {
        return ESP_ERR_INVALID_ARG;
    }

    // RC_RAA_WR  = 0x5A/0x5B
    // RC_RAA_WRS = 0x5E/0x5F (status before write)
    const uint8_t base_cmd = status_first ? 0x5Eu : 0x5Au;
    const uint8_t cmd = rc_opcode_with_addr_msb(base_cmd, addr_dword);

    const size_t data_bytes = 4 * dword_count;
    const size_t total_len = 2 + data_bytes;

    uint8_t tx[2 + 4 * 32] = {0};
    if (total_len > sizeof(tx)) {
        return ESP_ERR_INVALID_SIZE;
    }

    tx[0] = cmd;
    tx[1] = addr_low(addr_dword);
    for (size_t i = 0; i < dword_count; i++) {
        u32_to_be32(dwords[i], &tx[2 + 4 * i]);
    }

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = total_len * 8;
    t.tx_buffer = tx;

    // Per datasheet, WRS would return status first; with full duplex, we'd need an RX buffer.
    // Implementing this only when requested.
    uint8_t rx[2 + 4 * 32] = {0};
    if (status_first) {
        t.rx_buffer = rx;
    }

    esp_err_t err = spi_device_transmit(dev, &t);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "spi_device_transmit write falhou: %s", esp_err_to_name(err));
        return err;
    }

    if (status_first && out_status) {
        // In SPI, during cmd+addr bytes the returned data is undefined; status is expected after that.
        // Here we only captured rx for parity; not used by current firmware.
        *out_status = rx[2];
    }

    return ESP_OK;
}

esp_err_t as6031_read_firmware_info(spi_device_handle_t dev, uint32_t *out_fwu_rng, uint32_t *out_fwu_rev, uint32_t *out_fwa_rev)
{
    esp_err_t err = as6031_send_remote_cmd(dev, AS6031_RC_BM_REQ);
    if (err != ESP_OK) {
        return err;
    }

    uint32_t v = 0;

    if (out_fwu_rng) {
        err = as6031_read_u32(dev, AS6031_ADDR_SRR_FWU_RNG, &v);
        if (err != ESP_OK) goto done;
        *out_fwu_rng = v;
    }

    if (out_fwu_rev) {
        err = as6031_read_u32(dev, AS6031_ADDR_SRR_FWU_REV, &v);
        if (err != ESP_OK) goto done;
        *out_fwu_rev = v;
    }

    if (out_fwa_rev) {
        err = as6031_read_u32(dev, AS6031_ADDR_SRR_FWA_REV, &v);
        if (err != ESP_OK) goto done;
        *out_fwa_rev = v;
    }

done:
    {
        esp_err_t rel_err = as6031_send_remote_cmd(dev, AS6031_RC_BM_RLS);
        if (err == ESP_OK && rel_err != ESP_OK) {
            err = rel_err;
        }
    }

    return err;
}

esp_err_t as6031_read_flow_volume_liters(spi_device_handle_t dev, double *out_liters)
{
    if (dev == NULL || out_liters == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (AS6031_ADDR_RAM_FLOW_VOLUME_INT == 0xFFFFu || AS6031_ADDR_RAM_FLOW_VOLUME_FRACTION == 0xFFFFu) {
        ESP_LOGE(TAG, "Enderecos de FLOW_VOLUME nao configurados. Defina AS6031_ADDR_RAM_FLOW_VOLUME_INT/FRACTION.");
        return ESP_ERR_INVALID_STATE;
    }

    const uint16_t int_addr = (uint16_t)AS6031_ADDR_RAM_FLOW_VOLUME_INT;
    const uint16_t frac_addr = (uint16_t)AS6031_ADDR_RAM_FLOW_VOLUME_FRACTION;

    esp_err_t err = as6031_send_remote_cmd(dev, AS6031_RC_BM_REQ);
    if (err != ESP_OK) {
        return err;
    }

    uint32_t vol_int_u32 = 0;
    uint32_t vol_frac_u32 = 0;
    err = as6031_read_volume_snapshot(dev, int_addr, frac_addr, &vol_int_u32, &vol_frac_u32);

    esp_err_t rel_err = as6031_send_remote_cmd(dev, AS6031_RC_BM_RLS);
    if (err == ESP_OK && rel_err != ESP_OK) {
        err = rel_err;
    }
    if (err != ESP_OK) {
        return err;
    }

    // Leitura totalmente em 0xFF indica resposta inválida do barramento/dispositivo.
    if (vol_int_u32 == 0xFFFFFFFFu && vol_frac_u32 == 0xFFFFFFFFu) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const double liters_fixed32 = ((double)vol_int_u32 + ((double)vol_frac_u32 / 4294967296.0)) * 1000.0;
    const double liters_u32_ml = (double)vol_int_u32 / 1000.0;
    const double liters_u32_ul = (double)vol_int_u32 / 1000000.0;

#if AS6031_VOLUME_INT_IS_MILLILITERS
    *out_liters = liters_u32_ml;
#else
    *out_liters = liters_fixed32;
#endif

    // Diagnostico de mapa/escala: ajuda a validar se os enderecos configurados estao corretos.
    static bool s_logged_totalizer_diag = false;
    if (!s_logged_totalizer_diag) {
        ESP_LOGW(TAG,
                 "FLOW_DIAG: addr_int=0x%03X addr_frac=0x%03X int=0x%08lX frac=0x%08lX | L_32.32=%.6f L_u32_mL=%.6f L_u32_uL=%.6f mode=%s out=%.6f",
                 (unsigned)int_addr,
                 (unsigned)frac_addr,
                 (unsigned long)vol_int_u32,
                 (unsigned long)vol_frac_u32,
                 liters_fixed32,
                 liters_u32_ml,
                 liters_u32_ul,
#if AS6031_VOLUME_INT_IS_MILLILITERS
                 "u32_mL",
#else
                 "32.32_m3",
#endif
                 *out_liters);
        as6031_log_candidate_pairs_once(dev);
        s_logged_totalizer_diag = true;
    }

    return ESP_OK;
}

esp_err_t as6031_send_remote_cmd(spi_device_handle_t dev, uint8_t cmd)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &cmd;

    esp_err_t err = spi_device_transmit(dev, &t);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Falha ao enviar RC 0x%02X: %s", cmd, esp_err_to_name(err));
    }
    return err;
}

esp_err_t as6031_prepare_flow_meter_mode(spi_device_handle_t dev, bool optional_sys_init)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (optional_sys_init) {
        esp_err_t err = as6031_send_remote_cmd(dev, AS6031_RC_SYS_INIT);
        if (err != ESP_OK) {
            return err;
        }

        // Datasheet indica novo boot flow apos SYS_INIT; aguarda para evitar acessar no meio da inicializacao.
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    esp_err_t req_err = as6031_send_remote_cmd(dev, AS6031_RC_BM_REQ);
    if (req_err != ESP_OK) {
        return req_err;
    }

    esp_err_t op_err = as6031_send_remote_cmd(dev, AS6031_RC_IF_CLR);
    if (op_err == ESP_OK) {
        op_err = as6031_send_remote_cmd(dev, AS6031_RC_MCT_ON);
    }

    esp_err_t rel_err = as6031_send_remote_cmd(dev, AS6031_RC_BM_RLS);
    if (op_err != ESP_OK) {
        return op_err;
    }
    if (rel_err != ESP_OK) {
        return rel_err;
    }

    return ESP_OK;
}

esp_err_t as6031_apply_reference_tof_setup(spi_device_handle_t dev)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = as6031_send_remote_cmd(dev, AS6031_RC_BM_REQ);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(1));

    err = as6031_send_remote_cmd(dev, AS6031_RC_SYS_RST);
    if (err != ESP_OK) {
        (void)as6031_send_remote_cmd(dev, AS6031_RC_BM_RLS);
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    const uint32_t clr_boot_flags = 0x00000003u;
    err = as6031_raawr_dwords(dev, 0x0DDu, &clr_boot_flags, 1, false, NULL);
    if (err != ESP_OK) {
        (void)as6031_send_remote_cmd(dev, AS6031_RC_BM_RLS);
        return err;
    }

    err = as6031_send_remote_cmd(dev, AS6031_RC_BM_RLS);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    err = as6031_send_remote_cmd(dev, AS6031_RC_BM_REQ);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(1));

    err = as6031_send_remote_cmd(dev, AS6031_RC_MCT_OFF);
    if (err != ESP_OK) {
        (void)as6031_send_remote_cmd(dev, AS6031_RC_BM_RLS);
        return err;
    }

    for (uint16_t i = 0; i < 15; i++) {
        const uint16_t addr = (uint16_t)(0x0C0u + i);
        err = as6031_raawr_dwords(dev, addr, &s_ref_config_official[i], 1, false, NULL);
        if (err != ESP_OK) {
            (void)as6031_send_remote_cmd(dev, AS6031_RC_BM_RLS);
            return err;
        }
    }

    err = as6031_raawr_dwords(dev, 0x0D0u, &s_ref_reg_d0_fast, 1, false, NULL);
    if (err == ESP_OK) err = as6031_raawr_dwords(dev, 0x0D1u, &s_ref_reg_d1, 1, false, NULL);
    if (err == ESP_OK) err = as6031_raawr_dwords(dev, 0x0D2u, &s_ref_reg_d2, 1, false, NULL);
    if (err == ESP_OK) err = as6031_raawr_dwords(dev, 0x0DAu, &s_ref_reg_da, 1, false, NULL);
    if (err == ESP_OK) err = as6031_raawr_dwords(dev, 0x0DBu, &s_ref_reg_db, 1, false, NULL);
    for (uint16_t addr = 0x030u; err == ESP_OK && addr <= 0x037u; addr++) {
        err = as6031_raawr_dwords(dev, addr, &s_ref_ufr_zero, 1, false, NULL);
    }
    if (err != ESP_OK) {
        (void)as6031_send_remote_cmd(dev, AS6031_RC_BM_RLS);
        return err;
    }

    err = as6031_send_remote_cmd(dev, AS6031_RC_BM_RLS);
    if (err != ESP_OK) {
        return err;
    }

    err = as6031_send_remote_cmd(dev, AS6031_RC_BM_REQ);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(1));

    err = as6031_send_remote_cmd(dev, AS6031_RC_MCT_ON);
    if (err == ESP_OK) {
        err = as6031_send_remote_cmd(dev, AS6031_RC_IF_CLR);
    }

    esp_err_t rel_err = as6031_send_remote_cmd(dev, AS6031_RC_BM_RLS);
    if (err != ESP_OK) {
        return err;
    }
    if (rel_err != ESP_OK) {
        return rel_err;
    }

    return ESP_OK;
}

esp_err_t as6031_read_delta_tof_seconds(spi_device_handle_t dev, double *out_delta_tof_s)
{
    if (dev == NULL || out_delta_tof_s == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = as6031_send_remote_cmd(dev, AS6031_RC_BM_REQ);
    if (err != ESP_OK) {
        return err;
    }

    uint32_t raw_tof_u32 = 0;
    err = as6031_read_u32(dev, 0x038u, &raw_tof_u32);
    if (err == ESP_OK) {
        const uint32_t clear_error_flag = 0x00000001u;
        err = as6031_raawr_dwords(dev, 0x0DDu, &clear_error_flag, 1, false, NULL);
    }

    esp_err_t rel_err = as6031_send_remote_cmd(dev, AS6031_RC_BM_RLS);
    if (err == ESP_OK && rel_err != ESP_OK) {
        err = rel_err;
    }
    if (err != ESP_OK) {
        return err;
    }

    const int32_t raw_tof_i32 = (int32_t)raw_tof_u32;
    *out_delta_tof_s = ((double)raw_tof_i32) * AS6031_LSB_S * AS6031_TOF_NUM;
    return ESP_OK;
}

esp_err_t as6031_read_flow_lps_from_ufr(spi_device_handle_t dev, double *out_flow_lps)
{
    if (dev == NULL || out_flow_lps == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    double delta_tof_s = 0.0;
    esp_err_t err = as6031_read_delta_tof_seconds(dev, &delta_tof_s);
    if (err != ESP_OK) {
        return err;
    }

    const double speed_mps = delta_tof_s * AS6031_SPEED_K * AS6031_FLOW_POLARITY;
    *out_flow_lps = speed_mps * AS6031_PIPE_AREA * 1000.0;
    return ESP_OK;
}

esp_err_t as6031_read_raw_tof_i32(spi_device_handle_t dev, int32_t *out_raw_tof)
{
    if (dev == NULL || out_raw_tof == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = as6031_send_remote_cmd(dev, AS6031_RC_BM_REQ);
    if (err != ESP_OK) {
        return err;
    }

    uint32_t raw_u32 = 0;
    err = as6031_read_u32(dev, 0x038u, &raw_u32);
    if (err == ESP_OK) {
        const uint32_t clear_error_flag = 0x00000001u;
        err = as6031_raawr_dwords(dev, 0x0DDu, &clear_error_flag, 1, false, NULL);
    }

    esp_err_t rel_err = as6031_send_remote_cmd(dev, AS6031_RC_BM_RLS);
    if (err == ESP_OK && rel_err != ESP_OK) {
        err = rel_err;
    }
    if (err != ESP_OK) {
        return err;
    }

    *out_raw_tof = (int32_t)raw_u32;
    return ESP_OK;
}

esp_err_t as6031_debug_dump_regs(spi_device_handle_t dev)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = as6031_send_remote_cmd(dev, AS6031_RC_BM_REQ);
    if (err != ESP_OK) {
        return err;
    }

    uint32_t r_d0 = 0, r_e0 = 0, r34 = 0, r35 = 0, r36 = 0, r37 = 0, r38 = 0;
    err = as6031_read_u32(dev, 0x0D0u, &r_d0);
    if (err == ESP_OK) err = as6031_read_u32(dev, 0x0E0u, &r_e0);
    if (err == ESP_OK) err = as6031_read_u32(dev, 0x034u, &r34);
    if (err == ESP_OK) err = as6031_read_u32(dev, 0x035u, &r35);
    if (err == ESP_OK) err = as6031_read_u32(dev, 0x036u, &r36);
    if (err == ESP_OK) err = as6031_read_u32(dev, 0x037u, &r37);
    if (err == ESP_OK) err = as6031_read_u32(dev, 0x038u, &r38);

    esp_err_t rel_err = as6031_send_remote_cmd(dev, AS6031_RC_BM_RLS);
    if (err == ESP_OK && rel_err != ESP_OK) {
        err = rel_err;
    }
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG,
             "AS6031 DBG D0=0x%08lX E0=0x%08lX 34=0x%08lX 35=0x%08lX 36=0x%08lX 37=0x%08lX 38=0x%08lX",
             (unsigned long)r_d0,
             (unsigned long)r_e0,
             (unsigned long)r34,
             (unsigned long)r35,
             (unsigned long)r36,
             (unsigned long)r37,
             (unsigned long)r38);

    return ESP_OK;
}
