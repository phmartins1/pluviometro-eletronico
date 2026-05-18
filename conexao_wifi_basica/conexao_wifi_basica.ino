//Wifi Base
#include <WiFi.h>
// Biblioteca MQTT
#include <PubSubClient.h>
// Configurações
#include "secrets.h"

// tentativa inicial de conexao com tempo de espera
const unsigned long TIMEOUT_CONEXAO = 20000;

// Intervalo de verificacao de status
const unsigned long INTERVALO_VERIFICACAO = 10000;

unsigned long ultimaVerificacao = 0;

WiFiClient clienteWiFi;
PubSubClient clienteMQTT(clienteWiFi);

// TESTE
const unsigned long INTERVALO_TESTE = 1000;
unsigned long ultimaPublicacao = 0;

bool conectarWiFi() {
  Serial.print("Conectando à rede: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);                  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long inicio = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - inicio > TIMEOUT_CONEXAO) {
      Serial.println("\nFalha: tempo de conexão esgotado.");
      return false;
    }
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConexão estabelecida!");
  Serial.print("Endereço IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("RSSI (intensidade do sinal): ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
  return true;
}

void conectarBrokerMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;

  Serial.println("Conectando ao Broker MQTT...");

  unsigned long inicio = millis();
  while (!clienteMQTT.connected()) {
    if (millis() - inicio > TIMEOUT_CONEXAO) {
      Serial.println("\nFalha: tempo de conexão ao Broker MQTT esgotado.");
      return;
    }

    char IDCliente[50];
    sprintf(IDCliente, "PEC-Pluviometro-%06X", (uint32_t)ESP.getEfuseMac());

    if (clienteMQTT.connect(IDCliente, TOKEN_BROKER, "")) {
      Serial.println("Conectado ao Broker MQTT.");
      return;
    }
  }
}

//verificar e reconectar se necessário
void verificarConexao() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Conexão WiFi perdida. Tentando reconectar...");
    WiFi.disconnect();
    conectarWiFi();
  } else {
    Serial.print("Wi-Fi ativo | IP: ");
    Serial.print(WiFi.localIP());
    Serial.print(" | RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  }

  if (!clienteMQTT.connected()) {
    Serial.println("Conexão ao Broker MQTT perdida. Tentando reconectar...");
    clienteMQTT.disconnect();
    conectarBrokerMQTT();
  }
}

//Setup
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nInicializando módulo Wi-Fi");
  conectarWiFi();

  clienteMQTT.setServer(BROKER_MQTT, PORTA_MQTT);
  conectarBrokerMQTT();
}

void loop() {
  if (millis() - ultimaVerificacao >= INTERVALO_VERIFICACAO) {
    ultimaVerificacao = millis();
    verificarConexao();
  }
  
  if (millis() - ultimaPublicacao >= INTERVALO_TESTE) {
    ultimaPublicacao = millis();
    if (clienteMQTT.connected()) {
      bool publicado = clienteMQTT.publish(TOPICO_MQTT, "teste");
      if (!publicado) Serial.println("Falha ao publicar");
    }
  }

  clienteMQTT.loop();

  // Aqui entrarão futuramente as rotinas de leitura do ADS1115
  // e o envio dos dados para o servidor do CEMADEN ou computador.
}
