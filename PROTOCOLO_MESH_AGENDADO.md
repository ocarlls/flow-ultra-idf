# Protocolo Mesh Agendado — Rede Híbrida ESP-NOW + LoRa

Versao do protocolo: 2
Status: **camada de sincronia agendada IMPLEMENTADA (ver §14) — store-and-forward, agregação e cripto seguem no plano**

> Objetivo: meters a bateria reportam **1×/dia** ao root (sempre ligado), atravessando
> uma malha multi-salto onde **todos os intermediários também são a bateria e dormem**.
> Híbrida: **ESP-NOW** dentro de clusters densos, **LoRa** para vencer os vãos.

---

## 0. Decisões confirmadas (pelo usuário)

| Tema | Decisão |
|------|---------|
| Relógio (sem cristal 32 kHz por enquanto) | **(B) re-sincronizar mais vezes/dia** para conter a deriva do RC |
| Direção | **Só uplink** (meter→root). Sem downlink → meters nunca escutam fora da janela |
| Horário-base da coleta de dados | **16:30**, 1×/dia |
| Profundidade da árvore | **≤ 4 saltos LoRa** (confirmado) → `route_path[4]` basta, sem mudar struct |
| Buffer store-and-forward | **1 dia** de leituras |
| Criptografia | **Sim** |
| Agregação no subroot | **Sim** |
| Provisionamento | **Descoberta dinâmica** (o rádio acha o vizinho); só a **chave de rede** é provisionada (Kconfig→NVS) |
| Tempo no ROOT | **RTC/GPS** ancora o 16:30 (sem NTP) |
| Fonte do dado no meter | **Placeholder sintético** agora; sensor real depois |

---

## 1. Estado ATUAL do código (o que já existe — base de reuso)

A exploração mostrou que o **caminho de encaminhamento já funciona**; falta a camada de baixo consumo + confiabilidade.

| Componente | Já faz hoje | file |
|-----------|-------------|------|
| **root_node.c** | LoRa RX contínuo + dedup + ACK + log de rota. **Sem** Wi-Fi/sleep. | `root_node.c:131-188` |
| **subroot_node.c** | ESP-NOW RX (de meters) → LoRa TX; `route_path` append; reverse-path ACK; light-sleep opcional; self-test opcional | `subroot_node.c:80-305,408-432` |
| **meter (meter_tx_test.c)** | Relay ESP-NOW em light sleep; **NÃO origina** dados | `meter_tx_test.c:163-236` |
| **mesh_dedup** | Tabela 64 entradas, TTL 60 s, chave `(meter_id,sequence,type)`, thread-safe | `mesh_dedup.h:13-37` |
| **flow_packet** | Struct + CRC32 + helpers; já tem `subroot_mac`, `route_path[4][6]`, `route_len`, `hop_count`, RSSI/SNR | `flow_packet.h:21-94` |
| **e220 LoRa** | init/tx/rx(task)/eventos; modos M0/M1 (normal/WOR/sleep/config); RSSI byte | `e220_lora.h:62-68`, `e220_lora.c` |

**O que NÃO existe ainda (escopo deste protocolo):**
1. **Deep sleep agendado** (hoje os roles ficam acordados / light sleep contínuo).
2. **Sincronia de tempo** (beacon) para a janela comum.
3. **Store-and-forward** (buffer em RTC RAM/NVS).
4. **Origem de dado no meter** (AS6031 foi removido — ver §9).
5. **Criptografia** de payload.
6. **Agregação** de múltiplas leituras num quadro LoRa.

**Reuso direto do git (código de produção removido):** deep sleep + NVS + RTC estão em
`git show 94cc172:main/meter_node.c` (`RTC_DATA_ATTR`, `nvs_load/store_sequence`,
`esp_sleep_enable_timer_wakeup`, `esp_deep_sleep_start`, `esp_sleep_get_wakeup_cause`).

---

## 2. Topologia e papéis

```
[Cluster denso]                         [Vão grande]
 meter ─ESP-NOW─┐
 meter ─ESP-NOW─┤►  SUBROOT ───LoRa───►  SUBROOT ──LoRa──►  ROOT
 meter ─ESP-NOW─┘   (cabeça do cluster                      (sempre ligado,
                     + ponte LoRa)                           referência de tempo)
```

- **METER** (bateria): origina 1 leitura/dia, envia ao pai por ESP-NOW. Dorme.
- **SUBROOT** (bateria): junta meters (ESP-NOW), **agrega** e repassa por LoRa rumo ao root. Dorme.
- **ROOT** (sempre ligado): sink + **mestre de tempo**; recebe, faz ACK, repassa ao backend.

**Descoberta dinâmica de pai (sem comissionamento de topologia — "o rádio acha o vizinho"):**
o BEACON carrega `hop_from_root` (rank); o root anuncia rank 0. Cada node, durante o sync,
escuta beacons dos vizinhos **nos dois rádios** e adota como **pai** o vizinho de **menor
`hop_from_root`** (desempate por melhor RSSI/SNR); o **rádio do uplink** é aquele em que o
melhor beacon chegou. O node assume `meu_hop = pai_hop + 1` e **rebroadcasta** o beacon →
a árvore se forma sozinha e **auto-recupera** se o pai sumir (no próximo round escolhe outro).
Só a **chave de rede** é provisionada (segredo compartilhado, Kconfig agora / NVS depois) —
a descoberta resolve a topologia, não o segredo.

---

## 3. Ciclo diário e re-sincronização (decisão B, sem cristal)

A deriva do RC interno é ~1–3%/dia (pior com temperatura). Para manter a janela curta
**sem cristal**, separamos **dados** de **sincronia**:

- **Coleta de dados: 1×/dia às 16:30.**
- **Sync (resync): a cada `RESYNC_PERIOD` (ex.: 2–4 h).** Em cada resync o node acorda
  brevemente, captura o BEACON do pai, corrige o relógio, e volta a dormir. Isso limita a
  deriva acumulada a `RESYNC_PERIOD` (não a 24 h) → guarda de escuta de ~segundos a dezenas
  de segundos em vez de ~30 min.
- A **guarda** de cada acordada = `deriva_estimada(RESYNC_PERIOD) + margem`.
- **Migração p/ cristal (produção):** com 32 kHz, `RESYNC_PERIOD` pode voltar a 24 h e a
  guarda cai pra segundos — **só mudando 2 parâmetros de Kconfig**, sem mexer no protocolo.

> Trade-off explícito: (B) gasta energia em ~6–12 acordadas curtas de sync/dia, mas evita a
> guarda gigante. Vamos medir no PPK2 e ajustar `RESYNC_PERIOD`.

---

## 4. Sincronia de tempo (beacon em cascata)

- **ROOT** é a âncora (relógio estável; idealmente NTP pelo uplink).
- No instante de sync, o ROOT difunde **BEACON** (LoRa + ESP-NOW), repetido N vezes.
- Cada **SUBROOT** acorda com a guarda, **escuta o BEACON do pai**, ajusta relógio +
  `next_sync`/`next_collect`, e **repassa o BEACON** aos filhos.
- Cada **METER** escuta o BEACON do seu subroot e alinha.
- Deriva relevante = acumulada desde o último sync (≤ `RESYNC_PERIOD`).

---

## 5. Coleta (convergecast) + agregação

Na janela das 16:30:
1. **METER**: envia DATA própria (+ buffer pendente) ao pai por ESP-NOW; aguarda ACK; retry com jitter.
2. **SUBROOT**: recebe DATA dos meters (ESP-NOW), deduplica, e **agrega** várias leituras num
   único quadro LoRa (ver formato §6.4) rumo ao pai/root; aguarda ACK.
3. **ROOT**: recebe, deduplica, ACK, entrega ao backend.

Profundidade variável: `hop_count` + `route_path[4]` limitam/registram os saltos LoRa
(máx. 4 hops hoje no struct). A janela de coleta (`COLLECT_WINDOW_MS`) é dimensionada pela
profundidade observada em campo (parâmetro ajustável).

---

## 6. Pacotes

Reaproveita `flow_packet.h`. Tipos: `METER_DATA=1`, `ACK=2`, `ROUTE=3`, **`BEACON=4` (novo)**.

### 6.1 DATA — `flow_packet_t` (já existe, sem mudança de layout)
### 6.2 ACK — struct compacta `{magic,ver,type=ACK, ack_origin_id[6], ack_sequence, from_id[6], status, crc32}`
### 6.3 BEACON (novo) — `{magic,ver,type=BEACON, root_id[6], sync_seq, root_epoch_ms, next_collect_in_ms, next_sync_in_ms, hop_from_root, sender_id[6], crc32}`
### 6.4 AGREGADO (novo) — quadro LoRa com header + vetor de **registros compactos** por leitura:
`reading_record = {meter_id[6], sequence(4), volume_liters(4), delta_liters(4), battery_pct(1)}` ≈ 19 B.
Cabe ~10 leituras por quadro LoRa de 200 B → economiza airtime/energia no subroot.

---

## 7. Criptografia (decisão: sim) — **camada de aplicação (AES-GCM)**

Por quê não a nativa: o **ESP-NOW só cifra unicast** (a malha usa broadcast) e o **LoRa é
texto puro** (E220 transparente). Logo, cifrar no protocolo é o único jeito **uniforme**.

- **AEAD AES-128-GCM via `mbedtls`** sobre o corpo do `flow_packet` (e do agregado).
- **Chave de rede** provisionada (NVS, ou Kconfig p/ protótipo).
- **Nonce** derivado de `(meter_id, sequence, type)` — único por pacote.
- Ganha **autenticidade** (anti-spoof) além de sigilo — melhor que só CRC.
- Requer adicionar **`mbedtls`** ao `PRIV_REQUIRES` em `main/CMakeLists.txt`.
- Campo extra: **tag de autenticação (16 B)** no pacote (ou variante cifrada).

(Allowlist de MAC no root + TLS no MQTT do gateway continuam recomendados, como no doc legado.)

---

## 8. Confiabilidade (store-and-forward)

- **ACK por salto** + **retry com jitter** (backoff aleatório anti-colisão). Máx. `N_RETRIES`.
- **Buffer** em **RTC RAM** (`RTC_DATA_ATTR`, sobrevive ao deep sleep) + **NVS** (sobrevive a
  power-loss). **Nunca SRAM.** FIFO; cada item `{origem, sequence, payload, retries, pendente}`.
  Dimensão = **1 dia** de leituras (decisão).
- **Limite** `MAX_RETRIES_TOTAL` → marca erro/loga e descarta (não trava buffer).
- **Dedup** (`mesh_dedup`, já existe) por `(meter_id, sequence)`.

---

## 9. ⚠️ Lacuna: origem do dado no meter

O meter hoje é **relay puro** (AS6031 removido a seu pedido). No cenário real ele precisa
**originar** a leitura 1×/dia. Como a fonte do sensor está indefinida, proponho:
- **Agora:** originar um pacote com leitura **placeholder/sintética** (volume crescente,
  bateria via ADC) — suficiente pra validar todo o protocolo (sync, coleta, ACK, buffer, cripto).
- **Depois:** plugar a fonte real do sensor quando definida.

---

## 10. Energia — gargalo do subroot (lembrete)

Subroot acorda mais (junta cluster + TX LoRa) → descarrega antes. Bateria maior/solar; janela
e payload mínimos; **agregação** ajuda. Hardware: **pôr o E220 em sleep (M0/M1)** no deep sleep;
**medir deep sleep real** da placa no PPK2 (quiescente do regulador decide a autonomia).

---

## 11. Plano de implementação (etapas + arquivos)

Construção incremental, validável por etapa. Reuso forte do existente e do git `94cc172`.

| # | Etapa | Arquivos (novo/alterado) | Reuso |
|---|-------|--------------------------|-------|
| 0 | **Módulos comuns** (tirar duplicação): `espnow_common`, `lora_common`, `mac_utils`, `lightsleep_common` | novos `.c/.h` em `main/` | extrair de `subroot_node.c`/`meter_tx_test.c`/`root_node.c` |
| 1 | **Deep sleep agendado + sync + descoberta de pai** (beacon do root em cascata; nodes acordam alinhados E escolhem pai por menor rank/melhor RSSI) | novo `flow_sync.c/.h`; `flow_rtc.h` (estado RTC); `flow_nvs.c` (schedule/seq); + BEACON em `flow_packet.h`; alterar `root/subroot/meter_*` | `git 94cc172` (deep sleep/NVS/RTC); `e220`, `esp_now` |
| 2 | **Origem no meter** (placeholder) + **convergecast + ACK/retry** | `meter_*`, `subroot_node.c` | `flow_packet`, `mesh_dedup`, ACK de `root_node.c:105-127` |
| 3 | **Store-and-forward** (buffer RTC/NVS, FIFO, retries, descarte) | novo `flow_store_fwd.c/.h` | `git 94cc172` NVS |
| 4 | **Agregação no subroot** (registro compacto, quadro LoRa) | `subroot_node.c`, `flow_packet.h` (agregado) | `e220` |
| 5 | **Criptografia AES-GCM** (mbedtls) em todos os rádios | novo `flow_crypto.c/.h`; `main/CMakeLists.txt` (+mbedtls); chave em NVS | `mbedtls` |
| 6 | **Robustez**: fallback de pai, modo SCAN (perdeu sync), agregação/airtime, limites | roles | — |

**Validação por etapa (PPK2 + logs):**
- E1: confirmar no log que meters/subroots acordam e capturam o BEACON; janelas se sobrepõem.
- E2: leitura do meter chega ao root com ACK (sem perda no caminho feliz).
- E3: derrubar um salto e ver o dado ser reentregue no próximo ciclo (buffer).
- E4: ver N leituras num único quadro LoRa.
- E5: confirmar payload cifrado (sniffer não lê) e tag validada.

---

## 12. Parâmetros Kconfig (a criar, sob `FLOW_MESH_MODE`)

`FLOW_COLLECT_HOUR=16`, `FLOW_COLLECT_MIN=30`, `FLOW_RESYNC_PERIOD_S` (ex.: 10800=3 h),
`FLOW_SYNC_GUARD_MS`, `FLOW_COLLECT_WINDOW_MS`, `FLOW_N_RETRIES`, `FLOW_RETRY_JITTER_MS`,
`FLOW_BUFFER_DAYS=1`, `FLOW_MAX_RETRIES_TOTAL`, `FLOW_CRYPTO_ENABLE`, `FLOW_CRYPTO_KEY_HEX`.

---

## 14. ✅ IMPLEMENTADO — Camada de sincronia agendada de baixo consumo

### 14.1 Beacon: burst + descoberta (root)

O root **não** emite mais beacon contínuo a cada 2 s (ocupava 10-15% do airtime LoRa).
Agora (`root_node.c`):
- **Burst de sync**: a cada `FLOW_RESYNC_PERIOD_S`, janela de `FLOW_BEACON_BURST_MS`
  (10 s) com beacons a cada `FLOW_BEACON_SPACING_MS` (500 ms). `sync_seq` incrementa
  **1× por burst** → filhos deduplicam o repasse.
- **Lead da coleta**: todo início de coleta é precedido de um burst extra
  (`FLOW_COLLECT_LEAD_MS` = burst + 5 s antes). Quem capta chega à coleta com relógio
  fresco (deriva de segundos, não de horas) → janela de coleta curta mesmo sem cristal.
- **Descoberta**: fora dos bursts, 1 beacon a cada `FLOW_DISCOVERY_PERIOD_S` (30 s)
  com o **mesmo** `sync_seq` (não gera repasse). Nó em SCAN acha a rede em ≤30 s.
- **Silêncio na janela de coleta** (canal livre p/ dados).
- Beacon carrega `next_sync_in_ms` (próximo burst) e `next_collect_in_ms` reais.

### 14.2 Cascata auto-alinhada (subroot, `FLOW_SUBROOT_DEEP_SLEEP=y`)

Cada nó agenda o wake **relativo ao burst do próprio pai** (não ao root): dorme
`next_sync_in - guard`, acorda, escuta LoRa (Wi-Fi **desligado** durante a escuta),
capta o beacon → mede deriva → liga Wi-Fi → **repassa em burst nos dois rádios**
(`FLOW_REBROADCAST_COPIES`×, ESP-NOW p/ meters + LoRa se rank < 4, com a agenda
descontada do tempo decorrido) → decide próximo evento → E220 em sleep
(M0=M1=1 + `gpio_hold`) → deep sleep. O atraso por salto fica embutido na agenda
relativa; o jitter de cascata é coberto por `FLOW_HOP_JITTER_MS` × rank na guarda.
Seleção de pai agora usa `flow_sync_consider_beacon` (menor rank, empate RSSI) — antes
o subroot aceitava e repassava qualquer beacon.

### 14.3 Guarda adaptativa (deriva medida, não chutada)

`flow_sync.c` (`flow_drift_*`): a cada resync o nó mede
`err_ms = (chegada do beacon desde o wake) - guard` e acumula em ppm (controle
integral, ganho ½; outliers >5% rejeitados; persiste em RTC RAM). O sleep seguinte é
corrigido por esse ppm. A guarda usa a **incerteza residual** (começa em
`FLOW_DRIFT_INIT_PPM`=3%, cai pela metade a cada amostra até `FLOW_DRIFT_RESIDUAL_PPM`
=0,3%):

`guard = incerteza_ppm × sleep + rank × FLOW_HOP_JITTER_MS`, clamp
[`FLOW_GUARD_MIN_MS`=2 s, `FLOW_GUARD_MAX_MS`=120 s], **dobra por miss consecutivo**.

Convergência (resync 3 h): 1ª acordada guard=108 s → 54 s → 27 s → ... → regime
**~32 s** (0,3% × 3 h). Com cristal 32 kHz: `FLOW_DRIFT_INIT_PPM`≈500 → guard de regime
= `GUARD_MIN` (2 s), **só Kconfig**.

### 14.4 Fallback

Miss → guard ×2 no próximo ciclo; `FLOW_SCAN_AFTER_MISSES` (3) misses → **SCAN**
(escuta `FLOW_SCAN_LISTEN_MS`=45 s > descoberta 30 s; re-tenta a cada RESYNC/4).
Cold boot → SCAN. Nó de rank ≥2 em cold boot só acha a rede quando o pai repassa um
burst (pais dormem) → junção em ≤ alguns períodos de resync; na instalação, usar o
escape hatch ou ligar na ordem root→subroots→meters.

### 14.5 Energia — o que foi feito e a conta

Medidas implementadas:
1. E220 dormindo (~2 µA) durante o deep sleep: `e220_lora_sleep()` +
   `e220_lora_prepare_deep_sleep()` (hold de M0/M1 — sem isso os pinos flutuam e o
   E220 volta a RX contínuo, ~11 mA = ~264 mAh/dia!).
2. Registradores do E220 são não-voláteis → re-wake usa `skip_radio_config`
   (economiza 50 ms-1,5 s de UART/timeouts por acordada).
3. Subroot escuta LoRa com **Wi-Fi desligado**; só liga p/ repassar (~200 ms).
4. Early-exit: escuta termina no 1º beacon válido (não espera a janela toda).
5. Meter na coleta **não escuta**: acorda com relógio fresco, transmite N cópias
   (com jitter anti-colisão) e dorme.
6. Alvo <30 s → não paga boot+reinit: espera acordado (`vTaskDelay`).
7. Escape hatch (`FLOW_STAY_AWAKE_GPIO`=9/BOOT baixo no boot → fica acordado) resolve
   o USB-JTAG do C6 sumir em deep sleep.

Estimativa/dia do METER (C6, resync 3 h = 8 acordadas + 1 coleta; correntes típicas:
deep sleep ~15 µA, boot ~300 ms @ 30 mA, Wi-Fi RX ~85 mA, guard médio ≈ regime 32 s):
- Deep sleep: 24 h × 15 µA ≈ **0,36 mAh**
- Boots: 9 × 0,3 s × 30 mA ≈ **0,02 mAh**
- Escuta sync: 8 × (32 s + ~2 s) × 85 mA ≈ **6,4 mAh** ← domina
- Coleta (TX burst): 1 × 3 s × 90 mA ≈ **0,08 mAh**
- **Total ≈ 6,9 mAh/dia → ~14 meses em 3000 mAh.**
  Com cristal 32 kHz (guard 2 s): escuta cai p/ 8 × 4 s × 85 mA ≈ 0,76 mAh →
  **total ≈ 1,2 mAh/dia → ~6,8 anos.** A guarda é O PARÂMETRO da autonomia.

SUBROOT idem + LoRa: escuta LoRa (E220 RX ~11 mA + ESP ativo ~25 mA) em vez de Wi-Fi
RX na fase de sync → 8 × 34 s × 36 mA ≈ 2,7 mAh + coleta (janela 15 s + margem, Wi-Fi
ligado, ~110 mA) ≈ 0,6 mAh + deep sleep/boots ≈ 0,4 mAh → **≈ 3,7 mAh/dia → ~2,2 anos
em 3000 mAh** (melhor que o meter no regime sem cristal porque escutar LoRa é ~2,4×
mais barato que escutar Wi-Fi). Validar tudo no PPK2 — números são de datasheet.

Alavanca futura (documentada, não implementada): **WOR RX do E220 + wake por AUX**
(ext wakeup) na escuta de sync do subroot — guard grande a custo de ~µA médios; o
driver já tem os enums de modo, falta expor WOR_RX/WOR_TX e armar o AUX como fonte
de wake. Vale a pena se a guarda de regime ficar >30 s sem cristal.

### 14.6 Perfil de bancada (validar a cascata em minutos)

```
CONFIG_FLOW_RESYNC_PERIOD_S=120       # resync a cada 2 min
CONFIG_FLOW_COLLECT_PERIOD_S=600     # coleta a cada 10 min
CONFIG_FLOW_GUARD_MAX_MS=30000
CONFIG_FLOW_SCAN_LISTEN_MS=45000
```
Logs por acordada mostram: tipo do wake, deriva medida (ppm), guard usada, pai/rank,
agenda. Validação: (1) meter/subroot captam beacon e dormem alinhados; (2) guard
encolhendo a cada ciclo; (3) derrubar o pai → miss → escalada → SCAN → readoção;
(4) coleta: DATA do meter chega ao root com ACK; (5) PPK2: corrente de deep sleep
com E220 em hold (≠ RX contínuo).

---

## 13. ✅ Pendências resolvidas

1. **Dado do meter**: placeholder sintético agora; sensor real depois.
2. **Tempo no ROOT**: **RTC/GPS** ancora o 16:30 (sem NTP).
3. **Profundidade**: **≤ 4 saltos LoRa** → `route_path[4]` basta.
4. **Provisionamento**: **descoberta dinâmica** (o rádio acha o vizinho). Só a **chave de rede** é provisionada (Kconfig agora / NVS depois).
