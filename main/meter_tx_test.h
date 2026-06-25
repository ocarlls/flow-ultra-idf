#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Modo de operacao do node METER da malha (sem sensor/SPI).
 *
 * O ESP fica em LIGHT SLEEP com o WiFi ligado e o radio em duty-cycle
 * connectionless (wake-on-radio por WiFi). Ele NAO origina dados: dorme ate
 * captar um pacote ESP-NOW numa janela de escuta e, ao receber, acorda,
 * reencaminha (relay) para o proximo node e volta a dormir.
 *
 * Selecionado em build time por CONFIG_FLOW_METER_TX_TEST. O light sleep real
 * exige CONFIG_PM_ENABLE + CONFIG_FREERTOS_USE_TICKLESS_IDLE.
 *
 * Nunca retorna.
 */
void meter_tx_test_run(void);

#ifdef __cplusplus
}
#endif
