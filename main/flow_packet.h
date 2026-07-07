#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FLOW_PACKET_MAGIC   0x464CU /* "FL" */
#define FLOW_PACKET_VERSION 1U

typedef enum {
    FLOW_PKT_TYPE_METER_DATA = 1,
    FLOW_PKT_TYPE_ACK = 2,
    FLOW_PKT_TYPE_ROUTE = 3,
    FLOW_PKT_TYPE_BEACON = 4,
} flow_packet_type_t;

typedef struct __attribute__((packed)) {
    uint16_t magic;           // 0x464C ("FL")
    uint8_t  version;         // protocol version = 1
    uint8_t  type;            // METER_DATA=1, ACK=2, ROUTE=3
    uint32_t sequence;        // per-device monotonic counter
    uint8_t  meter_id[6];     // MAC of the originating meter
    uint32_t volume_liters;   // total accumulated volume
    uint32_t delta_liters;    // consumption since last report
    int64_t  timestamp_ms;    // esp_timer_get_time() / 1000
    uint8_t  battery_pct;     // 0–100
    int8_t   espnow_rssi;     // RSSI of last ESP-NOW hop (-dBm)
    int8_t   lora_rssi;       // RSSI of last LoRa hop (-dBm)
    int8_t   lora_snr;        // SNR of last LoRa hop
    uint8_t  hop_count;       // total hops from meter to root
    uint8_t  subroot_mac[6];  // MAC of the subroot that ingested this packet
    uint8_t  route_path[4][6];// MACs of up to 4 LoRa hops
    uint8_t  route_len;       // number of valid entries in route_path
    uint32_t crc32;           // CRC32 of all fields above
} flow_packet_t;

static inline bool flow_packet_header_ok(const flow_packet_t *pkt)
{
    return (pkt != NULL) && (pkt->magic == FLOW_PACKET_MAGIC) && (pkt->version == FLOW_PACKET_VERSION);
}

static inline uint32_t flow_crc32_ieee(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint32_t)data[i];
        for (int bit = 0; bit < 8; ++bit) {
            uint32_t mask = (uint32_t)(-(int32_t)(crc & 1U));
            crc = (crc >> 1U) ^ (0xEDB88320U & mask);
        }
    }
    return ~crc;
}

static inline uint32_t flow_packet_compute_crc32(const flow_packet_t *pkt)
{
    if (pkt == NULL) {
        return 0U;
    }
    return flow_crc32_ieee((const uint8_t *)pkt, offsetof(flow_packet_t, crc32));
}

static inline bool flow_packet_validate_crc32(const flow_packet_t *pkt)
{
    if (!flow_packet_header_ok(pkt)) {
        return false;
    }
    return pkt->crc32 == flow_packet_compute_crc32(pkt);
}

static inline void flow_packet_update_crc32(flow_packet_t *pkt)
{
    if (pkt == NULL) {
        return;
    }
    pkt->crc32 = flow_packet_compute_crc32(pkt);
}

static inline void flow_packet_init_empty(flow_packet_t *pkt)
{
    if (pkt == NULL) {
        return;
    }
    memset(pkt, 0, sizeof(*pkt));
    pkt->magic = FLOW_PACKET_MAGIC;
    pkt->version = FLOW_PACKET_VERSION;
    pkt->espnow_rssi = INT8_MIN;
    pkt->lora_rssi = INT8_MIN;
    pkt->lora_snr = INT8_MIN;
}

/* ===========================================================================
 * BEACON — sincronia de tempo + descoberta de pai (gradiente por rank).
 * Difundido do ROOT (rank 0) para fora; cada node rebroadcasta com rank+1.
 * Estrutura compacta e separada do flow_packet_t (que é o quadro de dados).
 * =========================================================================== */
typedef struct __attribute__((packed)) {
    uint16_t magic;               // 0x464C ("FL")
    uint8_t  version;             // protocol version = 1
    uint8_t  type;                // = FLOW_PKT_TYPE_BEACON (4)
    uint8_t  root_id[6];          // MAC do root (raiz da árvore)
    uint8_t  sender_id[6];        // MAC do node que (re)transmitiu este beacon (pai imediato)
    uint8_t  hop_from_root;       // rank: 0 no root, +1 a cada salto
    uint8_t  reserved;            // alinhamento/futuro
    uint32_t sync_seq;            // contador de sessões de sync (monotônico do root)
    int64_t  root_epoch_ms;       // tempo do root em ms (âncora p/ alinhar relógio)
    uint32_t next_collect_in_ms;  // ms até a próxima janela de coleta (16:30)
    uint32_t next_sync_in_ms;     // ms até o próximo sync
    uint32_t crc32;               // CRC32 de todos os campos acima
} flow_beacon_t;

static inline bool flow_beacon_header_ok(const flow_beacon_t *b)
{
    return (b != NULL) && (b->magic == FLOW_PACKET_MAGIC) &&
           (b->version == FLOW_PACKET_VERSION) && (b->type == FLOW_PKT_TYPE_BEACON);
}

static inline uint32_t flow_beacon_compute_crc32(const flow_beacon_t *b)
{
    if (b == NULL) {
        return 0U;
    }
    return flow_crc32_ieee((const uint8_t *)b, offsetof(flow_beacon_t, crc32));
}

static inline bool flow_beacon_validate_crc32(const flow_beacon_t *b)
{
    if (!flow_beacon_header_ok(b)) {
        return false;
    }
    return b->crc32 == flow_beacon_compute_crc32(b);
}

static inline void flow_beacon_update_crc32(flow_beacon_t *b)
{
    if (b == NULL) {
        return;
    }
    b->crc32 = flow_beacon_compute_crc32(b);
}

static inline void flow_beacon_init_empty(flow_beacon_t *b)
{
    if (b == NULL) {
        return;
    }
    memset(b, 0, sizeof(*b));
    b->magic = FLOW_PACKET_MAGIC;
    b->version = FLOW_PACKET_VERSION;
    b->type = FLOW_PKT_TYPE_BEACON;
}

#ifdef __cplusplus
}
#endif
