#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MESH_DEDUP_TABLE_SIZE 64U
#define MESH_DEDUP_TTL_MS     60000LL

typedef struct {
    bool in_use;
    uint8_t meter_id[6];
    uint32_t sequence;
    int64_t last_seen_ms;
    uint8_t seen_types_mask;
} mesh_dedup_entry_t;

typedef struct {
    mesh_dedup_entry_t entries[MESH_DEDUP_TABLE_SIZE];
    portMUX_TYPE mux;
} mesh_dedup_table_t;

void mesh_dedup_init(mesh_dedup_table_t *table);

// Retorna true se for duplicado (dentro da TTL) para o mesmo (meter_id, sequence, type).
// Caso contrario, registra como visto e retorna false.
bool mesh_dedup_is_duplicate(mesh_dedup_table_t *table,
                            const uint8_t meter_id[6],
                            uint32_t sequence,
                            uint8_t type,
                            int64_t now_ms);

#ifdef __cplusplus
}
#endif
