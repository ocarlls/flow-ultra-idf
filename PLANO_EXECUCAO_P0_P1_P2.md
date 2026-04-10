# Plano de Execucao P0/P1/P2 - Hidrometro Inteligente

Contexto: projeto para Londrina com ESP32-C6 + AS6031, rede Node/Gateway e foco em durabilidade de 10 anos, confiabilidade e antifraude.

## P0 (0-30 dias) - Base funcional e confiavel

Objetivo: ter ciclo completo medicao -> acumulacao -> envio diario com resiliencia minima.

### Entregas
- Firmware node com deep sleep por padrao e wake por timer/interrupcao valida.
- Leitura basica do AS6031 e acumulacao de volume em RTC RAM.
- Envio diario via ESP-NOW com callback de confirmacao.
- Gateway MVP recebendo ESP-NOW e publicando MQTT QoS 1.
- Logs essenciais de diagnostico (reset cause, falha de envio, timeout).

### Responsaveis sugeridos
- Firmware embarcado (Node): Eng. Embarcado A.
- Gateway + MQTT: Eng. Backend/IoT B.
- QA de firmware basico: QA C.
- Coordenacao tecnica: Tech Lead D.

### Criterio de saida P0
- Node mede e envia diariamente sem travar por 7 dias continuos em bancada.
- Taxa de entrega diaria >= 95% no ambiente de teste.

## P1 (31-60 dias) - Robustez operacional e seguranca inicial

Objetivo: reduzir perdas, endurecer comportamento em falhas e preparar seguranca de producao.

### Entregas
- Fila local e retransmissao no gateway quando sem IP.
- Antiduplicacao de pacotes e controle de sequencia.
- Calibracao inicial do modelo de vazao (erro de medicao reduzido).
- Hardening de firmware: imagens assinadas em ambiente de homologacao.
- Estrutura de provisionamento de identidade unica por node.

### Responsaveis sugeridos
- Firmware embarcado (Node): Eng. Embarcado A.
- Gateway resiliente: Eng. Backend/IoT B.
- Seguranca embarcada: Eng. Seguranca E.
- Testes de campo assistidos: Operacao F + QA C.

### Criterio de saida P1
- Taxa de entrega diaria >= 98% em piloto controlado.
- Perda de dados apos reset = 0 em cenarios de teste definidos.

## P2 (61-90 dias) - Industrializacao e antifraude completa

Objetivo: prontidao de escala municipal e controles antifraude completos.

### Entregas
- Secure Boot e Flash Encryption em fluxo de fabrica.
- Programacao de eFuses com dupla validacao e trilha de auditoria.
- Playbook de incidentes e monitoramento operacional por bairro/setor.
- Processo de rollout de firmware (canario, lote, rollback controlado).
- Validacao final metrologica e de seguranca para faturamento.

### Responsaveis sugeridos
- Seguranca de produto: Eng. Seguranca E.
- Operacao de campo: Operacao F.
- Plataforma de dados e monitoramento: Eng. Plataforma G.
- Governanca/compliance: PM/Compliance H.

### Criterio de saida P2
- Auditoria de seguranca aprovada.
- Criticos de medicao e faturamento dentro da tolerancia contratada.
- Procedimento de fabrica pronto para escala.

## Riscos principais e mitigacoes
- Dependencia de radio em area densa: mitigar com piloto em multiplos bairros e ajuste de janela de envio.
- Erro de medicao por variacao termica: mitigar com compensacao de temperatura e recalibracao por lote.
- Complexidade de seguranca em producao: mitigar com esteira de provisionamento versionada e checklist de eFuses.

## Proximo passo imediato (esta semana)
- Consolidar target de hardware (ESP32-C6) no projeto.
- Fechar contrato de payload ESP-NOW e topicos MQTT.
- Iniciar testes de consumo por estado de energia.
