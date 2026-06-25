#include "meter_node.h"
#include "meter_tx_test.h"

// ---------------------------------------------------------------------------
// Node "meter" da malha.
// Opera EXCLUSIVAMENTE como relay ESP-NOW em light sleep com WiFi ligado
// (wake-on-radio): dorme ate o radio captar um pacote durante a janela de
// escuta, acorda, reencaminha para o proximo node e volta a dormir.
// A implementacao fica em meter_tx_test.c. Nao ha leitura de sensor nem SPI.
// ---------------------------------------------------------------------------
void meter_node_run(void)
{
    meter_tx_test_run();
}
