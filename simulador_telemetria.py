"""
Simulador de telemetria REALISTA para ThingsBoard (via Flespi).

Diferenca para o simulador simples: aqui S, impactos, pico e rms NAO sao
sorteados de forma independente. Todos derivam de uma mesma "intensidade de
chuva" que evolui ao longo do tempo (periodos secos, pancadas, decaimento),
entao os valores fazem sentido em conjunto -- como numa medicao real.

Util para ver a dashboard se comportando como chuva de verdade e validar os
calculos (acumulado, intensidade) e os filtros de diagnostico.

Requisito: pip install paho-mqtt
Uso:        python simulador_telemetria_realista.py
Para parar: Ctrl+C
"""

import json
import os
import random
import re
import sys
import time

import paho.mqtt.client as mqtt

CAMINHO_SECRETS = "secrets.h"
INTERVALO = 10                      # segundos entre publicacoes
IDENT = "pluviometro_teste"         # deve bater com o ident do canal flespi


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


def ao_conectar(client, userdata, flags, reason_code, properties=None):
    if reason_code == 0:
        print("Conectado ao broker com sucesso!")
        userdata["conectado"] = True
    else:
        print(f"Falha na conexao. Codigo de retorno: {reason_code}")


def ao_desconectar(client, userdata, *args):
    userdata["conectado"] = False
    print("Desconectado do broker.")


def evoluir_intensidade(intensidade):
    """Random walk com tendencia a secar + pancadas ocasionais.
    Faz a chuva vir em 'spells' (como na vida real), nao ruido por amostra.
    Retorna a nova intensidade em [0, ~1.2] (unidade abstrata de intensidade)."""
    intensidade += random.gauss(0, 0.08) - 0.03   # deriva, tendendo a baixar
    if random.random() < 0.06:                     # ~6% de chance: comeca pancada
        intensidade += random.uniform(0.3, 0.9)
    return max(0.0, min(1.2, intensidade))


def medidas_coerentes(intensidade):
    """Deriva S, impactos, pico e rms de UMA MESMA intensidade, de forma
    fisicamente plausivel: secam juntos, sobem juntos, e pico >= rms sempre."""
    if intensidade < 0.02:                         # seco -> tudo zero
        return 0.0, 0, 0.0, 0.0

    # S (integral, V.s no intervalo) cresce com a intensidade
    S = max(0.0, intensidade * 0.40 + random.gauss(0, 0.02))
    # impactos: contagem proporcional a intensidade, com dispersao
    impactos = max(0, int(random.gauss(intensidade * 25, intensidade * 4)))
    # rms cresce com a intensidade (energia media do sinal)
    rms = max(0.0, intensidade * 0.12 + random.gauss(0, 0.01))
    # pico SEMPRE >= rms; sobe com a intensidade + variacao de gota grande
    pico = min(3.3, rms + intensidade * 1.2 + abs(random.gauss(0, 0.2)))

    return round(S, 4), impactos, round(pico, 3), round(rms, 4)


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

    intensidade = 0.0
    try:
        while True:
            intensidade = evoluir_intensidade(intensidade)
            S, impactos, pico, rms = medidas_coerentes(intensidade)
            T = round(INTERVALO + random.uniform(-0.3, 0.3), 2)

            payload = json.dumps({
                "ident": IDENT,
                "S": S,
                "T": T,
                "impactos": impactos,
                "pico": pico,
                "rms": rms,
            })

            resultado = cliente.publish(topico, payload)
            estado_chuva = "seco" if intensidade < 0.02 else f"chuva (i={intensidade:.2f})"
            if resultado.rc == mqtt.MQTT_ERR_SUCCESS:
                print(f"[{estado_chuva}] Publicado: {payload}")
            else:
                print(f"Falha ao publicar (rc={resultado.rc})")

            time.sleep(INTERVALO)

    except KeyboardInterrupt:
        print("\nEncerrando simulador...")
    finally:
        cliente.loop_stop()
        cliente.disconnect()


if __name__ == "__main__":
    main()