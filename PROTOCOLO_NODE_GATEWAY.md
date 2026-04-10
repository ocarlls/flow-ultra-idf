# Protocolo Node <-> Gateway (ESP-NOW) e MQTT

Versao do protocolo: 1

## 1. Quadro de dados do Node (ESP-NOW)

Tipo: DATA (`msg_type = 1`)

Campos (ordem no payload binario):
- `version` (u8): versao do protocolo.
- `msg_type` (u8): tipo da mensagem.
- `payload_len` (u16): tamanho logico do payload apos cabecalho.
- `device_id` (u32): identificador unico do node.
- `sequence` (u32): contador monotonicamente crescente por node.
- `volume_total` (float32): volume acumulado em litros.
- `battery_mv` (u16): bateria em milivolts.
- `crc16` (u16): CRC-16/CCITT-FALSE calculado sobre o frame com `crc16=0`.

Politica de envio no node:
- Maximo de 3 tentativas por janela de transmissao.
- ACK obrigatorio para considerar entrega confirmada.
- Em falha de ACK, `last_transmission` nao e atualizado (reenvio na proxima janela).

## 2. Quadro de ACK do Gateway (ESP-NOW)

Tipo: ACK (`msg_type = 2`)

Campos:
- `version` (u8)
- `msg_type` (u8)
- `payload_len` (u16)
- `device_id` (u32)
- `sequence` (u32): deve espelhar a sequencia recebida.
- `status` (u8): `1=aceito`, `0=rejeitado`.
- `crc16` (u16): CRC-16/CCITT-FALSE com `crc16=0` no calculo.

Regras de validacao no node:
- `version` e `msg_type` devem ser validos.
- `crc16` deve bater.
- `sequence` deve ser igual ao ultimo frame enviado.
- `status` deve ser `1`.

## 3. Topicos MQTT (Gateway -> Broker)

Padrao de topicos:
- Telemetria: `londrina/{bairro}/{setor}/{device_id}/telemetry`
- Estado do node: `londrina/{bairro}/{setor}/{device_id}/state`
- Alarmes: `londrina/{bairro}/{setor}/{device_id}/alarm`

QoS recomendado:
- Telemetria de faturamento: QoS 1.
- Estado/heartbeat: QoS 0 ou 1 conforme criticidade.
- Alarmes: QoS 1.

Payload JSON sugerido para telemetria:
```json
{
  "protocolVersion": 1,
  "deviceId": 43981,
  "sequence": 1234,
  "volumeTotalLiters": 987.654,
  "batteryMv": 3600,
  "gatewayId": "gw-norte-01",
  "rssi": -67,
  "receivedAt": "2026-04-02T18:20:00Z",
  "firmwareVersion": "node-0.1.0"
}
```

## 4. Requisitos de seguranca minima

- Gateway autenticado no broker MQTT com TLS.
- Lista de MACs permitidos por gateway.
- Rejeicao de payload com CRC invalido.
- Log de auditoria para `device_id`, `sequence`, `gatewayId` e status de ACK.
