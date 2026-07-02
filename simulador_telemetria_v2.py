"""
Simulador de telemetria V2 — 4 piezos, publica ao encher o buffer.

Geometria do pluviômetro:
  - Lados maiores (A0, A2): 144 cm² cada
  - Lados menores (A1, A3): 47,6 cm² cada

LÓGICA:
  1. Preenche um buffer de tamanho fixo (N amostras por canal)
  2. Cada canal tem seu próprio tempo de coleta (T),
     pois são lidos sequencialmente no ADS1115
  3. Assim que encheu, publica os 4 buffers + tempo
  4. Zera e recomeça

  Vantagem: sem timer, sem desperdício de energia.
  O ESP pode inclusive entrar em deep sleep entre ciclos de coleta.

Payload publicado ao encher o buffer:
  {
    "ident": "pluviometro_teste",
    "T":     1.395,                         # tempo real do loop completo (s)
    "n":     300,                            # amostras por canal
    "area_maior_cm2": 144.0,
    "area_menor_cm2": 47.6,
    "buf_A0": [0.0, 0.0, 0.142, ...],
    "buf_A1": [...],
    "buf_A2": [...],
    "buf_A3": [...]
  }

Requisito: pip install paho-mqtt
Uso:        python simulador_telemetria_v2.py
Para parar: Ctrl+C
"""

import json
import math
import os
import random
import re
import sys
import time

import paho.mqtt.client as mqtt

CAMINHO_SECRETS = "secrets.h"
IDENT = "pluviometro_teste"

# Áreas úteis de cada par (cm²)
AREA_MAIOR = 144.0    # A0, A2
AREA_MENOR = 47.6     # A1, A3

# Fator de escala do sinal proporcional à área.
FATOR_MAIOR = 1.0
FATOR_MENOR = AREA_MENOR / AREA_MAIOR   # ~0.33

# Simulação do ADC / buffer
BUFFER_SIZE = 300           # mesmo do firmware (N amostras por canal)
SAMPLE_RATE = 860           # máximo do ADS1115 (SPS)
NOISE_FLOOR = 0.003         # abaixo disso vira 0 (V)

# Os 4 canais são lidos intercalados no mesmo loop (A0,A1,A2,A3 por iteração).
# Tempo total do loop: 300 iterações × 4 leituras / 860 SPS ≈ 1.395 s
TEMPO_BUFFER = (BUFFER_SIZE * 4) / SAMPLE_RATE

# descarte por timeout (PARA IMPLEMENTAR NO FIRMWARE REAL)
# Se o buffer não encher dentro de TIMEOUT_BUFFER segundos, descarta.
# Útil para filtrar artefatos (folha, inseto, objeto pousado no sensor)
# que geram sinal constante/estranho sem ser chuva.
#
# TIMEOUT_BUFFER = 5.0  # segundos — ajustar conforme necessidade
#
# No loop de coleta do ESP:
#   unsigned long t0_coleta = millis();
#   for (int i = 0; i < BUFFER_SIZE; i++) {
#       // ... lê amostra ...
#
#       // Verifica timeout
#       // if (millis() - t0_coleta > TIMEOUT_BUFFER * 1000) {
#       //     Serial.println("TIMEOUT: buffer descartado (possivel artefato)");
#       //     // zera buffer e recomeça sem publicar
#       //     break;
#       // }
#   }
#   // Só publica se o buffer encheu completo (i == BUFFER_SIZE)─


# leitura do secrets.h 
def carregar_secrets(caminho):
    if not os.path.exists(caminho):
        sys.exit(f"Arquivo nao encontrado: {caminho}\n"
                 f"Coloque o secrets.h na mesma pasta ou ajuste CAMINHO_SECRETS.")
    config = {}
    padrao = re.compile(r'#define\s+(\w+)\s+(.+)')
    with open(caminho, "r", encoding="utf-8") as arquivo:
        for linha in arquivo:
            achou = padrao.match(linha.strip())
            if achou:
                config[achou.group(1)] = achou.group(2).strip().strip('"')
    return config


# callbacks MQTT 
def ao_conectar(client, userdata, flags, reason_code, properties=None):
    if reason_code == 0:
        print("Conectado ao broker com sucesso!")
        userdata["conectado"] = True
    else:
        print(f"Falha na conexao. Codigo de retorno: {reason_code}")


def ao_desconectar(client, userdata, *args):
    userdata["conectado"] = False
    reason = args[1] if len(args) >= 2 else (args[0] if args else "desconhecido")
    print(f"Desconectado do broker. Motivo: {reason}")


#  modelo de chuva 
def evoluir_intensidade(intensidade):
    """Random walk com tendência a secar + pancadas ocasionais.
    Retorna a nova intensidade em [0, ~1.2]."""
    intensidade += random.gauss(0, 0.08) - 0.03
    if random.random() < 0.06:
        intensidade += random.uniform(0.3, 0.9)
    return max(0.0, min(1.2, intensidade))


def gerar_buffer_tensao(intensidade, fator_area, n_amostras):
    """Gera um buffer de N amostras de tensão simulando o sinal bruto do piezo.

    Modelo:
      - Sem chuva (intensidade < 0.02): tudo zero.
      - Com chuva: pulsos esporádicos (pico + decaimento exponencial),
        amplitude e frequência proporcionais à intensidade e à área.

    Retorna lista de floats (tensão em V, 4 casas decimais).
    """
    buffer = [0.0] * n_amostras

    if intensidade < 0.02:
        return buffer

    i_eff = intensidade * fator_area
    prob_impacto = min(0.15, i_eff * 25.0 / n_amostras)

    i = 0
    while i < n_amostras:
        if random.random() < prob_impacto:
            # Pulso de impacto (gota)
            amplitude = min(3.3, i_eff * 1.2 + abs(random.gauss(0, 0.3 * fator_area)))
            duracao = random.randint(4, 8)
            for j in range(duracao):
                if i + j >= n_amostras:
                    break
                valor = amplitude * math.exp(-j * 0.6)
                valor += random.gauss(0, 0.005)
                buffer[i + j] = round(max(0.0, min(3.3, valor)), 4)
            i += duracao
        else:
            # Ruído de fundo
            ruido = abs(random.gauss(0, 0.002))
            buffer[i] = round(ruido, 4) if ruido >= NOISE_FLOOR else 0.0
            i += 1

    return buffer


def tempo_coleta():
    """Simula o tempo real que o ESP leva para preencher os 4 buffers.
    No hardware real os 4 canais são lidos intercalados no mesmo for loop,
    então o tempo é único. Pequena variação por jitter do I2C / RTOS."""
    return round(TEMPO_BUFFER + random.uniform(-0.01, 0.01), 4)


# main 
def main():
    cfg = carregar_secrets(CAMINHO_SECRETS)
    try:
        broker = cfg["BROKER_MQTT"]
        porta = int(cfg["PORTA_MQTT"])
        token = cfg["TOKEN_BROKER"]
        topico = cfg["TOPICO_MQTT"]
    except KeyError as faltando:
        sys.exit(f"Faltou definir {faltando} no secrets.h")

    estado = {"conectado": False}
    try:
        cliente = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                              userdata=estado)
    except (AttributeError, TypeError):
        cliente = mqtt.Client(userdata=estado)

    cliente.username_pw_set(token)
    cliente.on_connect = ao_conectar
    cliente.on_disconnect = ao_desconectar

    print(f"Conectando ao broker {broker}:{porta} ...")
    cliente.connect(broker, porta, keepalive=60)
    cliente.loop_start()

    inicio = time.time()
    while not estado["conectado"]:
        if time.time() - inicio > 10:
            cliente.loop_stop()
            sys.exit("Nao foi possivel conectar ao broker em 10 s.")
        time.sleep(0.1)

    print(f"\nSimulador V2 — 4 piezos (publica ao encher buffer)")
    print(f"  A0, A2 (maiores): {AREA_MAIOR} cm²  |  A1, A3 (menores): {AREA_MENOR} cm²")
    print(f"  Buffer: {BUFFER_SIZE} amostras/canal  |  4 canais intercalados")
    print(f"  ADS1115: {SAMPLE_RATE} SPS  |  T ≈ {TEMPO_BUFFER:.3f} s por ciclo\n")

    intensidade = 0.0
    ciclo = 0
    try:
        while True:
            #  Evolui intensidade da chuva 
            intensidade = evoluir_intensidade(intensidade)

            # Tempo de coleta do loop completo (único para os 4 canais)
            # No ESP real: t0=millis() → for(300){lê A0,A1,A2,A3} → T=millis()-t0
            T = tempo_coleta()

            #  Gera buffers brutos para cada canal 
            buf_A0 = gerar_buffer_tensao(intensidade, FATOR_MAIOR, BUFFER_SIZE)
            buf_A1 = gerar_buffer_tensao(intensidade, FATOR_MENOR, BUFFER_SIZE)
            buf_A2 = gerar_buffer_tensao(intensidade, FATOR_MAIOR, BUFFER_SIZE)
            buf_A3 = gerar_buffer_tensao(intensidade, FATOR_MENOR, BUFFER_SIZE)

            # (FIRMWARE REAL) Descarte por timeout
            # Se o loop demorou mais que TIMEOUT_BUFFER para encher,
            # descarta tudo e recomeça sem publicar.
            # Isso filtra artefatos como folhas, insetos ou objetos pousados
            # no sensor que geram sinal constante sem ser chuva.
            #
            # if T > TIMEOUT_BUFFER:
            #     print(f"[DESCARTADO] Timeout na coleta — possível artefato")
            #     continue

            #  Monta e publica payload
            payload = json.dumps({
                "ident": IDENT,
                "T": T,
                "n": BUFFER_SIZE,
                "area_maior_cm2": AREA_MAIOR,
                "area_menor_cm2": AREA_MENOR,
                "buf_A0": buf_A0,
                "buf_A1": buf_A1,
                "buf_A2": buf_A2,
                "buf_A3": buf_A3,
            })

            resultado = cliente.publish(topico, payload)

            #  Log resumido 
            ciclo += 1
            estado_chuva = ("seco" if intensidade < 0.02
                            else f"chuva (i={intensidade:.2f})")

            def pico(buf):
                return max(buf)

            if resultado.rc == mqtt.MQTT_ERR_SUCCESS:
                print(f"[#{ciclo}] [{estado_chuva}] T={T:.3f}s  "
                      f"({len(payload)} B)  "
                      f"picos: {pico(buf_A0):.3f} {pico(buf_A1):.3f} "
                      f"{pico(buf_A2):.3f} {pico(buf_A3):.3f}")
            else:
                print(f"[#{ciclo}] Falha ao publicar (rc={resultado.rc})")

            # Espera simulada ≈ tempo que o ESP levaria para o loop completo
            time.sleep(T)

    except KeyboardInterrupt:
        print("\nEncerrando simulador V2...")
    finally:
        cliente.loop_stop()
        cliente.disconnect()


if __name__ == "__main__":
    main()
