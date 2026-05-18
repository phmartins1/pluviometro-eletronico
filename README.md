# Pluviômetro Eletrônico

Projeto de um **pluviômetro eletrônico** utilizando **sensores piezoelétricos**.

* **Hardware**: Circuito elétrico integrado com um microcontrolador ESP32
* **Firmware**: Captura dos dados, cálculo do volume de chuva e sistema de transmissão de dados baseado no protocolo MQTT, operando sobre conexão WiFi

## Estrutura do Repositório

```
pluviometro-eletronico/
├── basic_functionalities/  # Código do cálculo de volume de chuva
├── conexao_wifi_basica/    # Código de integração ESP32-BrokerMQTT
├── hardware/               # Esquemáticos elétricos do pluviômetro
├── .gitignore                 
└── README.md
```

## Segurança

Credenciais sensíveis (Rede WiFi, Broker MQTT) não são incluídas no repositório.

Utilize o arquivo `secrets_example.h` como base para criar seu próprio `secrets.h`.