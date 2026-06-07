import json
import os
import random
import re
import sys
import time

import paho.mqtt.client as mqtt

# Caminho do secrets.h. Ajuste se ele estiver em outra pasta.
CAMINHO_SECRETS = "secrets.h"

# Intervalo entre publicacoes, em segundos.
INTERVALO = 10


def carregar_secrets(caminho):
    """Le as linhas '#define CHAVE valor' do secrets.h e devolve um dict."""
    if not os.path.exists(caminho):
        sys.exit(f"Arquivo nao encontrado: {caminho}\n"
                 f"Coloque o secrets.h na mesma pasta ou ajuste CAMINHO_SECRETS.")

    config = {}
    padrao = re.compile(r'#define\s+(\w+)\s+(.+)')
    with open(caminho, "r", encoding="utf-8") as arquivo:
        for linha in arquivo:
            achou = padrao.match(linha.strip())
            if achou:
                chave = achou.group(1)
                valor = achou.group(2).strip().strip('"')  # tira aspas se houver
                config[chave] = valor
    return config


def ao_conectar(client, userdata, flags, reason_code, properties=None):
    if reason_code == 0:
        print("Conectado ao broker com sucesso!")
    else:
        print(f"Falha na conexao. Codigo de retorno: {reason_code}")
        print("Dicas: codigo 4/5 = token errado; "
              "se nem chega a conectar = host/porta incorretos, "
              "servidor fora do ar, ou MQTT bloqueado (ex.: por tras da Cloudflare).")


def ao_desconectar(client, userdata, *args):
    print("Desconectado do broker.")


def main():
    cfg = carregar_secrets(CAMINHO_SECRETS)

    try:
        broker = cfg["BROKER_MQTT"]
        porta = int(cfg["PORTA_MQTT"])
        token = cfg["TOKEN_BROKER"]
        topico = cfg["TOPICO_MQTT"]
    except KeyError as faltando:
        sys.exit(f"Faltou definir {faltando} no secrets.h")

    # paho-mqtt 2.x exige declarar a versao da API; try/except cobre versoes antigas.
    try:
        cliente = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    except (AttributeError, TypeError):
        cliente = mqtt.Client()

    # No ThingsBoard: token = usuario, senha vazia
    cliente.username_pw_set(token)
    cliente.on_connect = ao_conectar
    cliente.on_disconnect = ao_desconectar

    print(f"Conectando ao broker {broker}:{porta} ...")
    cliente.connect(broker, porta, keepalive=60)
    cliente.loop_start()

    try:
        while True:
            volume = round(random.uniform(0, 5), 1)  # mesmo dado de teste do .ino
            payload = json.dumps({"ident": "pluviometro_teste", "dado": volume})

            resultado = cliente.publish(topico, payload)
            if resultado.rc == mqtt.MQTT_ERR_SUCCESS:
                print(f"Publicado: {payload}")
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