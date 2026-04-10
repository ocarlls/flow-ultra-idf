# Checklist Tecnico - Hidrometro Inteligente (ESP32-C6 + AS6031)

Objetivo: transformar o prototipo em operacao industrial para Londrina com foco em 10 anos de durabilidade, baixo consumo, confiabilidade e antifraude.

## 1. Firmware do Node (Hidrometro)

### 1.1 Base do projeto ESP-IDF
- [ ] Definir target do projeto para ESP32-C6.
- [ ] Organizar modulos: medicao, energia, armazenamento, radio, seguranca, diagnostico.
- [ ] Configurar particoes com espaco para OTA, NVS e dados operacionais.
- [ ] Definir niveis de log para producao (reduzir consumo e ruido).

### 1.2 Medicao ultrassonica (AS6031)
- [ ] Integrar driver do AS6031 (I2C/SPI conforme hardware final).
- [ ] Validar interrupcao de wakeup por evento de fluxo.
- [ ] Calibrar calculo de volume por tempo de voo e velocidade do som no tubo.
- [ ] Implementar compensacao por temperatura (se sensor disponivel).
- [ ] Definir estrategia anti-ruido: filtro de pulsos espurios e hysteresis.
- [ ] Criar rotina de autodiagnostico do sensor (timeouts, saturacao, erro de leitura).

### 1.3 Energia e ciclo de vida
- [ ] Implementar deep sleep como estado padrao (meta: >99% do tempo).
- [ ] Salvar acumulados e estado em RTC RAM com fallback para NVS.
- [ ] Tratar brownout e retomada sem perda de volume acumulado.
- [ ] Medir consumo em todos os estados: sleep, wake de medicao, wake de transmissao.
- [ ] Definir budget energetico para 10 anos considerando perfil real de uso.

### 1.4 Comunicacao Node -> Gateway
- [ ] Implementar envio via ESP-NOW com frame compacto (ID, volume, bateria, contador, CRC).
- [ ] Adicionar sequenciamento de mensagens e janela de retransmissao.
- [ ] Exigir ACK do gateway para confirmar entrega diaria.
- [ ] Implementar politica de retentativa e backoff quando sem ACK.
- [ ] Garantir integridade de payload (CRC e validacao de tamanho/campos).

### 1.5 Robustez operacional
- [ ] Definir watchdogs para tarefas criticas.
- [ ] Isolar falhas por modulo (medicao, radio, armazenamento).
- [ ] Registrar eventos criticos em buffer circular para forense de falhas.
- [ ] Implementar contador de reboot e causa de reset para telemetria.

## 2. Gateway (Central de Bairro)

### 2.1 Recepcao e agregacao
- [ ] Implementar receptor ESP-NOW para multiplos nodes.
- [ ] Manter tabela de nodes (ultimo pacote, RSSI, nivel de bateria, falhas).
- [ ] Rejeitar duplicatas e pacotes invalidos.
- [ ] Persistir buffer local quando sem conectividade IP.

### 2.2 Publicacao MQTT
- [ ] Estruturar topicos por cidade/bairro/setor/node.
- [ ] Definir payload com metadados (timestamp, versao fw, qualidade do sinal, alarmes).
- [ ] Configurar QoS adequado para faturamento (minimo QoS 1).
- [ ] Implementar reconexao resiliente e fila de reenvio.
- [ ] Habilitar TLS e autenticacao forte no broker privado.

### 2.3 Operacao e monitoramento
- [ ] Expor metricas do gateway: disponibilidade, backlog, perdas, latencia.
- [ ] Criar alarmes de anomalia (node silencioso, consumo anormal, burst de erro).
- [ ] Definir estrategia de atualizacao de firmware segura para gateway.

## 3. Seguranca (Cofre Digital)

### 3.1 Hardening de firmware
- [ ] Assinar imagens de firmware para Secure Boot.
- [ ] Ativar Flash Encryption no modo de producao.
- [ ] Proteger chaves e segredos em areas seguras (NVS criptografada quando aplicavel).

### 3.2 eFuses e antitamper
- [ ] Programar eFuses de bloqueio de debug/JTAG em lote de producao.
- [ ] Bloquear rollback indevido de firmware.
- [ ] Definir procedimento irreversivel de gravacao com dupla checagem.

### 3.3 Seguranca de dados e rede
- [ ] Autenticar nodes no gateway (whitelist por identidade unica).
- [ ] Definir rotacao de chaves e politica de provisionamento seguro.
- [ ] Implementar trilha de auditoria para eventos de seguranca.

## 4. Qualidade, Testes e Validacao

### 4.1 Testes de firmware
- [ ] Criar testes unitarios para calculo de volume e filtros.
- [ ] Criar testes de integracao para ciclo medicao -> armazenamento -> envio.
- [ ] Simular falhas: perda de ACK, reset durante escrita, ruido de sensor.

### 4.2 Testes de campo (Londrina)
- [ ] Executar piloto em area de alta densidade urbana.
- [ ] Medir taxa real de entrega diaria por node.
- [ ] Medir autonomia estimada com perfil real de consumo.
- [ ] Validar comportamento em variacao de temperatura e umidade.

### 4.3 Criterios de aceite
- [ ] Confiabilidade de entrega diaria acima da meta definida pela operacao.
- [ ] Erro de medicao dentro da tolerancia de faturamento.
- [ ] Nenhuma perda de dados apos reset/queda de energia.
- [ ] Processo de boot seguro e criptografia validados em auditoria.

## 5. Industrializacao e Escala

### 5.1 Provisionamento em fabrica
- [ ] Criar fluxo de gravacao em linha: firmware, chaves, eFuses, testes finais.
- [ ] Gerar identidade unica por dispositivo e registrar inventario.
- [ ] Emitir relatorio automatico de aprovacao por unidade.

### 5.2 Operacao em massa
- [ ] Definir versionamento de firmware e politicas de rollout.
- [ ] Estabelecer playbook de incidentes (node offline, fraude, divergencia de medicao).
- [ ] Definir SLA de coleta e processamento de dados para faturamento.

### 5.3 Governanca e compliance
- [ ] Definir politica de retencao e protecao de dados.
- [ ] Documentar responsabilidades entre campo, rede, plataforma e seguranca.
- [ ] Planejar auditorias periodicas de seguranca e precisao metrologica.

## 6. Proximos marcos sugeridos (90 dias)
- [ ] Semana 1-2: baseline de firmware ESP-IDF + medicao basica + deep sleep.
- [ ] Semana 3-4: ESP-NOW com ACK + gateway MQTT funcional.
- [ ] Semana 5-6: hardening (Secure Boot, Flash Encryption, eFuses em homologacao).
- [ ] Semana 7-8: testes de estresse, energia e resiliencia.
- [ ] Semana 9-12: piloto em campo, ajustes finais e plano de rollout.
