#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Modo de TESTE de consumo de transmissao (Fase 1).
 *
 * Gera pacotes flow_packet_t SINTETICOS (sem AS6031/SPI) e os transmite por
 * ESP-NOW para medir o consumo SOMENTE da transmissao de dados.
 *
 * Selecionado em build time por CONFIG_FLOW_METER_TX_TEST. A estrategia de
 * sleep e escolhida por CONFIG_FLOW_METER_LIGHT_SLEEP:
 *   - y: light sleep + WiFi connectionless duty-cycled (recebe/relay dormindo)
 *   - n: baseline deep sleep so-transmissao (transmissor cego)
 *
 * Nunca retorna.
 */
void meter_tx_test_run(void);

#ifdef __cplusplus
}
#endif
