#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Utilidades de energia compartilhadas pelos roles a bateria (meter/subroot).

// Escape hatch p/ regravacao: o USB-JTAG do ESP32-C6 some em deep sleep.
// Se CONFIG_FLOW_STAY_AWAKE_GPIO estiver em nivel BAIXO no boot (ex.: botao
// BOOT segurado), trava aqui acordado para sempre (log periodico). Chamar no
// inicio de cada wake, antes de qualquer coisa cara.
void flow_power_stay_awake_hatch(void);

// Marca Wi-Fi/ESP-NOW como inicializado (para teardown seguro).
void flow_power_mark_wifi_up(void);

// Desliga ESP-NOW + Wi-Fi (idempotente; seguro chamar sem init).
void flow_power_wifi_teardown(void);

// Teardown do Wi-Fi + timer wakeup + deep sleep. Nao retorna.
// (O E220, quando presente, deve ser posto em sleep ANTES pelo chamador com
// e220_lora_prepare_deep_sleep().)
#if CONFIG_FLOW_TEST_NO_SLEEP
void flow_power_deep_sleep_ms(uint32_t sleep_ms);
#else
void flow_power_deep_sleep_ms(uint32_t sleep_ms) __attribute__((noreturn));
#endif

#ifdef __cplusplus
}
#endif
