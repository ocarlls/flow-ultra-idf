#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "sdkconfig.h"

// Datasheet AS6031 (DS000587) — Remote Communication / Memory Access
// RC_RAA_RD  : 0x7A (RAM/Register, addr[8]=0), 0x7B (RAM/Register, addr[8]=1)
// RC_RAA_RDS : 0x7E (RAM/Register, addr[8]=0), 0x7F (RAM/Register, addr[8]=1)
// RC_RAA_WR  : 0x5A / 0x5B
// RC_RAA_WRS : 0x5E / 0x5F

// RAA address space is 0x000..0x1FF (9 bits). The LSB of the RC opcode maps to RAA_ADR[8].

// ---- Addresses used by the Flow Meter Mode totalizer ----
// IMPORTANT:
// The datasheet references RAM cells like RAM_R_FLOW_VOLUME_INT and RAM_R_FLOW_VOLUME_FRACTION,
// but does not list their numeric addresses in the public PDF. These addresses depend on the
// firmware map used (ScioSense applied firmware / user firmware).
//
// Set these macros to the correct RAA addresses for your firmware map.
#ifndef AS6031_ADDR_RAM_FLOW_VOLUME_INT
#ifdef CONFIG_AS6031_RAM_FLOW_VOLUME_INT_ADDR
#define AS6031_ADDR_RAM_FLOW_VOLUME_INT CONFIG_AS6031_RAM_FLOW_VOLUME_INT_ADDR
#else
#define AS6031_ADDR_RAM_FLOW_VOLUME_INT 0xFFFFu
#endif
#endif

#ifndef AS6031_ADDR_RAM_FLOW_VOLUME_FRACTION
#ifdef CONFIG_AS6031_RAM_FLOW_VOLUME_FRACTION_ADDR
#define AS6031_ADDR_RAM_FLOW_VOLUME_FRACTION CONFIG_AS6031_RAM_FLOW_VOLUME_FRACTION_ADDR
#else
#define AS6031_ADDR_RAM_FLOW_VOLUME_FRACTION 0xFFFFu
#endif
#endif

#ifndef AS6031_VOLUME_INT_IS_MILLILITERS
#ifdef CONFIG_AS6031_VOLUME_INT_IS_MILLILITERS
#define AS6031_VOLUME_INT_IS_MILLILITERS CONFIG_AS6031_VOLUME_INT_IS_MILLILITERS
#else
#define AS6031_VOLUME_INT_IS_MILLILITERS 1
#endif
#endif

// Status & Result registers (direct mapped, see datasheet section 12.2.4)
#define AS6031_ADDR_SRR_FWU_RNG  0x0ECu
#define AS6031_ADDR_SRR_FWU_REV  0x0EDu
#define AS6031_ADDR_SRR_FWA_REV  0x0EEu

// Datasheet section 10.1.2 and 10.1.5 (Reset/Inits + Debug/System Commands)
#define AS6031_RC_SYS_RST   0x99
#define AS6031_RC_SYS_INIT  0x9A
#define AS6031_RC_BM_RLS    0x87
#define AS6031_RC_BM_REQ    0x88
#define AS6031_RC_MCT_OFF   0x8A
#define AS6031_RC_MCT_ON    0x8B
#define AS6031_RC_IF_CLR    0x8D

// Reads one or more 32-bit DWORDs from the Random Access Area (RAA).
// - addr_dword: RAA dword address (0x000..0x1FF)
// - out_dwords: output buffer for dword_count entries
// - status_first: when true, uses RC_RAA_RDS and returns SYS_STATUS in out_status (optional)
esp_err_t as6031_raard_dwords(spi_device_handle_t dev,
                             uint16_t addr_dword,
                             uint32_t *out_dwords,
                             size_t dword_count,
                             bool status_first,
                             uint8_t *out_status);

// Reads one or more 32-bit DWORDs from the Random Access Area (RAA).
// - start_addr: starting address in the RAA
// - out: output buffer for dword_count entries
// - dword_count: number of DWORDs to read
// - check_crc: when true, checks the CRC of each DWORD
// - out_crc: output buffer for the CRCs (optional)
esp_err_t as6031_raard_dwords(spi_device_handle_t dev, uint16_t start_addr, uint32_t *out, size_t dword_count, bool check_crc, uint8_t *out_crc);

// Writes one or more 32-bit DWORDs to the Random Access Area (RAA).
esp_err_t as6031_raawr_dwords(spi_device_handle_t dev,
                             uint16_t addr_dword,
                             const uint32_t *dwords,
                             size_t dword_count,
                             bool status_first,
                             uint8_t *out_status);

static inline esp_err_t as6031_read_u32(spi_device_handle_t dev, uint16_t addr_dword, uint32_t *out)
{
    return as6031_raard_dwords(dev, addr_dword, out, 1, false, NULL);
}

// Reads firmware revision/range registers (if bootloader has run and mapped them).
esp_err_t as6031_read_firmware_info(spi_device_handle_t dev, uint32_t *out_fwu_rng, uint32_t *out_fwu_rev, uint32_t *out_fwa_rev);

// Reads the internally accumulated flow volume and converts to liters.
// Requires AS6031_ADDR_RAM_FLOW_VOLUME_INT/FRACTION to be set to valid RAA addresses.
esp_err_t as6031_read_flow_volume_liters(spi_device_handle_t dev, double *out_liters);

// Sends a raw one-byte remote command over SPI (datasheet section 10.1.1).
esp_err_t as6031_send_remote_cmd(spi_device_handle_t dev, uint8_t cmd);

// Prepares AS6031 for autonomous Flow Meter operation.
// - optional_sys_init: when true, sends RC_SYS_INIT first (0x9A) to re-trigger boot flow.
// - always requests/release bus master and turns MCT ON.
esp_err_t as6031_prepare_flow_meter_mode(spi_device_handle_t dev, bool optional_sys_init);

// Applies a reference TOF setup equivalent to the validated Arduino sequence:
// RC_BM_REQ -> RC_SYS_RST -> clear flags(0x0DD)
// -> write CR 0x0C0..0x0CE and SHR 0x0D0/0x0D1/0x0D2/0x0DA/0x0DB
// -> RC_MCT_ON + RC_IF_CLR.
esp_err_t as6031_apply_reference_tof_setup(spi_device_handle_t dev);
