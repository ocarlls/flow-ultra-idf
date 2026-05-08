#include "mesh_dedup.h"

#include <string.h>

static bool mac_eq6(const uint8_t a[6], const uint8_t b[6])
{
    return memcmp(a, b, 6) == 0;
}

static void mac_copy6(uint8_t dst[6], const uint8_t src[6])
{
    memcpy(dst, src, 6);
}

void mesh_dedup_init(mesh_dedup_table_t *table)
{
    if (table == NULL) {
        return;
    }

    memset(table, 0, sizeof(*table));
    table->mux = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
}

static void expire_entries(mesh_dedup_table_t *table, int64_t now_ms)
{
    for (size_t i = 0; i < MESH_DEDUP_TABLE_SIZE; ++i) {
        if (!table->entries[i].in_use) {
            continue;
        }
        if ((now_ms - table->entries[i].last_seen_ms) > MESH_DEDUP_TTL_MS) {
            table->entries[i].in_use = false;
        }
    }
}

static size_t find_oldest_entry(const mesh_dedup_table_t *table)
{
    size_t oldest_idx = 0;
    int64_t oldest_ms = INT64_MAX;
    for (size_t i = 0; i < MESH_DEDUP_TABLE_SIZE; ++i) {
        if (!table->entries[i].in_use) {
            continue;
        }
        if (table->entries[i].last_seen_ms < oldest_ms) {
            oldest_ms = table->entries[i].last_seen_ms;
            oldest_idx = i;
        }
    }
    return oldest_idx;
}

static uint8_t type_to_mask(uint8_t type)
{
    switch (type) {
    case 1: // METER_DATA
        return 1U << 0U;
    case 2: // ACK
        return 1U << 1U;
    case 3: // ROUTE
        return 1U << 2U;
    default:
        // Tipos desconhecidos compartilham um unico bit.
        return 1U << 7U;
    }
}

bool mesh_dedup_is_duplicate(mesh_dedup_table_t *table,
                            const uint8_t meter_id[6],
                            uint32_t sequence,
                            uint8_t type,
                            int64_t now_ms)
{
    if (table == NULL || meter_id == NULL) {
        return true;
    }

    const uint8_t mask = type_to_mask(type);

    bool is_dup = false;

    portENTER_CRITICAL(&table->mux);

    expire_entries(table, now_ms);

    // Busca entrada existente
    for (size_t i = 0; i < MESH_DEDUP_TABLE_SIZE; ++i) {
        mesh_dedup_entry_t *e = &table->entries[i];
        if (!e->in_use) {
            continue;
        }
        if (e->sequence == sequence && mac_eq6(e->meter_id, meter_id)) {
            if ((e->seen_types_mask & mask) != 0U) {
                is_dup = true;
            } else {
                e->seen_types_mask |= mask;
            }
            e->last_seen_ms = now_ms;
            portEXIT_CRITICAL(&table->mux);
            return is_dup;
        }
    }

    // Nova entrada
    size_t free_idx = MESH_DEDUP_TABLE_SIZE;
    for (size_t i = 0; i < MESH_DEDUP_TABLE_SIZE; ++i) {
        if (!table->entries[i].in_use) {
            free_idx = i;
            break;
        }
    }

    if (free_idx == MESH_DEDUP_TABLE_SIZE) {
        free_idx = find_oldest_entry(table);
    }

    mesh_dedup_entry_t *e = &table->entries[free_idx];
    e->in_use = true;
    mac_copy6(e->meter_id, meter_id);
    e->sequence = sequence;
    e->last_seen_ms = now_ms;
    e->seen_types_mask = mask;

    portEXIT_CRITICAL(&table->mux);

    return false;
}
