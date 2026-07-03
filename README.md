# Pluviômetro Eletrônico

Projeto de um **pluviômetro eletrônico** utilizando **sensores piezoelétricos**.

* **Hardware**: Circuito elétrico integrado com um microcontrolador ESP32
* **Firmware**: Captura dos dados e transmissão via protocolo MQTT, operando sobre conexão WiFi
* **Nuvem**: Cálculo do volume de chuva (mm e mm/h) a partir do sinal cru

## Estrutura do Repositório

```
pluviometro-eletronico/
├── checkpoints-anteriores/   # Material desenvolvido em checkpoints anteriores (ultrapassado)
├── firmware/                 # Firmware de leitura dos piezos e transmissão de dados (ESP32)
├── hardware/                 # Esquemáticos elétricos do pluviômetro
├── telemetria/               # Simulador da telemetria
├── .gitignore
└── README.md
```

## Arquitetura: mínimo na placa, cálculo na nuvem

A abordagem atual divide o trabalho assim: **a placa só mede, a nuvem interpreta**.

O ESP32 lê os 4 piezos, preenche um buffer de `N` amostras por canal e publica os
buffers **brutos** (tensão em V) junto com o tempo real de coleta e a geometria do
sensor. Ele não calcula integral, mm, mm/h, RMS nem picos. Toda essa interpretação
acontece na nuvem (ThingsBoard / processamento), que guarda o histórico e a forma de
onda completa.

Vantagens:

- **Recalibração livre**: como a nuvem tem o sinal cru de todo o histórico, dá para
  mudar a constante de calibração ou o próprio algoritmo e reavaliar tudo, sem
  reprogramar o sensor.
- **Placa mais simples e econômica**: sem timer, sem acumuladores, sem multiplicação
  por calibração. Entre coletas o ESP poderia até entrar em deep sleep.
- **Flexibilidade**: qualquer métrica nova (filtragem de artefatos, contagem de
  impactos, detecção de eventos) é só uma consulta sobre os dados já guardados.

### O que a placa envia

A cada buffer cheio, o ESP publica um JSON via MQTT:

```json
{
  "ident": "pluviometro_teste",
  "T": 1.395,
  "n": 300,
  "area_maior_cm2": 144.0,
  "area_menor_cm2": 47.6,
  "buf_A0": [0.0, 0.0, 0.142, ...],
  "buf_A1": [...],
  "buf_A2": [...],
  "buf_A3": [...]
}
```

| Campo | O que é |
|-------|---------|
| `ident` | identificador do device (usado pelo stream flespi -> ThingsBoard) |
| `T` | duração **real** da coleta dos `N` pontos (s), medida com `millis()` |
| `n` | amostras por canal |
| `area_maior_cm2` | área das faces maiores (A0, A2) |
| `area_menor_cm2` | área das faces menores (A1, A3) |
| `buf_A0..buf_A3` | buffers brutos de tensão (V) de cada piezo |

> O payload tem alguns KB (4 buffers de `N` floats). No firmware é preciso aumentar o
> buffer interno do PubSubClient (`setBufferSize`), senão a mensagem é truncada.

## Precisão por área: combinando os 4 piezos

O pluviômetro tem 4 faces piezoelétricas de dois tamanhos:

- Faces **maiores** (A0, A2): 144 cm² cada.
- Faces **menores** (A1, A3): 47,6 cm² cada.
- Área total de captação: `2*144 + 2*47,6 = 383,2 cm²`.

Cada piezo é um coletor de área conhecida. O integral `S` de cada face é proporcional à
quantidade de água que bateu nela, ou seja, proporcional à taxa de chuva **vezes** a
área daquela face. Uma face maior recebe mais gotas e gera um sinal proporcionalmente
maior (no simulador isso aparece como o fator `AREA_MENOR / AREA_MAIOR`, que reduz o
sinal das faces menores na proporção da área).

Para ganhar precisão, a nuvem trata os 4 piezos como **um único coletor maior**: soma os
integrais e divide pela área total. Combinar mais área é o mesmo que ter um pluviômetro
maior, então o ruído estatístico (pouca gota numa janela curta) se cancela e a lâmina
(mm) fica mais estável do que a de um piezo isolado.

Cálculo na nuvem, por reporte:

```
# 1. integral de cada face a partir do buffer (mesma fórmula de antes, agora na nuvem)
S_i = (soma das tensões do buffer_i) * T / n        [V·s]

# 2. combina as 4 faces como um coletor de área total
S_total = S_A0 + S_A1 + S_A2 + S_A3
A_total = 383,2 cm²

# 3. lâmina caída no período (a calibração c embute a área e a sensibilidade)
lamina_mm = c * S_total

# ou, deixando a área explícita (densidade de sinal por área):
densidade   = S_total / A_total        [V·s/cm²]
lamina_mm   = c_area * densidade
```

A ponderação por área é exatamente essa divisão pela área total: as faces maiores pesam
mais, as menores pesam menos, na proporção em que captam chuva. O acumulado (mm) e as
intensidades (mm/h) saem de somar `S_total` sobre diferentes janelas de tempo.

## Simulador

[`telemetria/simulador-telemetria.py`](telemetria/simulador-telemetria.py) reproduz a telemetria da
abordagem atual: gera os buffers brutos dos 4 piezos (com sinal proporcional à área de
cada face) e publica no broker no mesmo formato do firmware. Útil para desenvolver e
testar o processamento na nuvem sem o hardware.

```
pip install paho-mqtt
python simulador-telemetria.py
```

## Segurança

Credenciais sensíveis (Rede WiFi, Broker MQTT) não são incluídas no repositório.

Utilize o arquivo [`firmware/exemplo_secrets.h`](firmware/exemplo_secrets.h) como base para criar seu próprio `secrets.h`.
