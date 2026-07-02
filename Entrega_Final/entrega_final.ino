/*
   OBJETIVO: O ESP32 realiza as medições brutas e envia via MQTT.
   Os cálculos matemáticos e geração do volume total ocorrem na Rule Chain.
*/

// Wifi Base
#include <WiFi.h>
// Biblioteca MQTT
#include <PubSubClient.h>
// Montar JSON
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
// Configurações (credenciais de WiFi e broker)
#include "secrets.h"

Adafruit_ADS1115 ads;

// CONFIGURAÇÕES
#define SDA_PIN 21
#define SCL_PIN 22
#define CHANNEL 0
#define CHANNEL_1 1
#define CHANNEL_2 2
#define CHANNEL_3 3

const float JANELA_SEGUNDOS = 5.0;
const int   BUFFER_SIZE = 300;        // N amostras por canal antes de publicar
const float ADC_LSB     = 0.000125;   // GAIN_ONE = 0,125 mV/bit
const float NOISE_FLOOR = 0.003;      // abaixo disso é ruído, vira 0 (V)

// Removido cálculo redundante que não era usado no código

// Buffers de leitura (tensão em V)
float samples_PiezoA0[BUFFER_SIZE];
float samples_PiezoA1[BUFFER_SIZE];
float samples_PiezoA2[BUFFER_SIZE];
float samples_PiezoA3[BUFFER_SIZE];

const uint16_t MQTT_BUFFER_SIZE = 20480;

// Conexões
const unsigned long TIMEOUT_CONEXAO = 20000;
const unsigned long INTERVALO_VERIFICACAO = 10000;
unsigned long ultimaVerificacao = 0;

WiFiClient clienteWiFi;
PubSubClient clienteMQTT(clienteWiFi);

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
  return true;
}

void conectarBrokerMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;
  Serial.println("Conectando ao ThingsBoard via MQTT...");

  unsigned long inicio = millis();
  while (!clienteMQTT.connected()) {
    if (millis() - inicio > TIMEOUT_CONEXAO) {
      Serial.println("\nFalha: tempo de conexão esgotado.");
      return;
    }

    char IDCliente[50];
    sprintf(IDCliente, "ESP32-Pluviometro-%06X", (uint32_t)ESP.getEfuseMac());

    if (clienteMQTT.connect(IDCliente, TOKEN_BROKER, "")) {
      Serial.println("Conectado ao ThingsBoard com sucesso!");
      return;
    } else {
      Serial.print("Falha na conexão, rc=");
      Serial.println(clienteMQTT.state());
      delay(2000);
    }
  }
}

void verificarConexao() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    conectarWiFi();
  }
  if (!clienteMQTT.connected()) {
    conectarBrokerMQTT();
  }
}

void publicarBuffers(float T) {
  JsonDocument doc;
  doc["T"] = T;
  doc["n"] = BUFFER_SIZE;

  JsonArray a0 = doc["buf_A0"].to<JsonArray>();
  JsonArray a1 = doc["buf_A1"].to<JsonArray>();
  JsonArray a2 = doc["buf_A2"].to<JsonArray>();
  JsonArray a3 = doc["buf_A3"].to<JsonArray>();

  for (int i = 0; i < BUFFER_SIZE; i++) {
    a0.add(serialized(String(samples_PiezoA0[i], 4)));
    a1.add(serialized(String(samples_PiezoA1[i], 4)));
    a2.add(serialized(String(samples_PiezoA2[i], 4)));
    a3.add(serialized(String(samples_PiezoA3[i], 4)));
  }

  String payload;
  serializeJson(doc, payload);

  if (clienteMQTT.connected()) {
    bool publicado = clienteMQTT.publish(TOPICO_MQTT, payload.c_str());
    if (publicado) {
      Serial.printf("Enviado: T=%.3f s | %u bytes\n", T, (unsigned)payload.length());
    } else {
      Serial.printf("Falha ao enviar. %u bytes\n", (unsigned)payload.length());
    }
  } else {
    Serial.println("Sem conexão MQTT. Dados perdidos.");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // ⚡ CORREÇÃO: Força o barramento I2C a 400kHz (Fast Mode) para reduzir o tempo de leitura

  if (!ads.begin()) {
     Serial.println("ERRO: ADS1115 não encontrado!");
     while (1);
  }

  ads.setGain(GAIN_ONE);          
  ads.setDataRate(RATE_ADS1115_860SPS);  

  conectarWiFi();
  clienteMQTT.setBufferSize(MQTT_BUFFER_SIZE);
  clienteMQTT.setServer(BROKER_MQTT, PORTA_MQTT);
  conectarBrokerMQTT();

  Serial.println("Sistema Pronto.");
}

void loop() {
  unsigned long t0 = micros();
  
  // 5 segundos / 300 amostras = 16666 us por ciclo
  const uint32_t INTERVALO_US = (uint32_t)(JANELA_SEGUNDOS * 1000000.0 / BUFFER_SIZE);
  uint32_t proximaLeitura = micros();

  for (int i = 0; i < BUFFER_SIZE; ) {
    if ((int32_t)(micros() - proximaLeitura) >= 0) {
      
      // Realiza a leitura sequencial nos 4 canais
      int16_t raw_PiezoA0 = ads.readADC_SingleEnded(CHANNEL);
      int16_t raw_PiezoA1 = ads.readADC_SingleEnded(CHANNEL_1);
      int16_t raw_PiezoA2 = ads.readADC_SingleEnded(CHANNEL_2);
      int16_t raw_PiezoA3 = ads.readADC_SingleEnded(CHANNEL_3);

      float v0 = raw_PiezoA0 * ADC_LSB;
      float v1 = raw_PiezoA1 * ADC_LSB;
      float v2 = raw_PiezoA2 * ADC_LSB;
      float v3 = raw_PiezoA3 * ADC_LSB;

      samples_PiezoA0[i] = (v0 < NOISE_FLOOR) ? 0.0f : v0;
      samples_PiezoA1[i] = (v1 < NOISE_FLOOR) ? 0.0f : v1;
      samples_PiezoA2[i] = (v2 < NOISE_FLOOR) ? 0.0f : v2;
      samples_PiezoA3[i] = (v3 < NOISE_FLOOR) ? 0.0f : v3;

      i++;
      proximaLeitura += INTERVALO_US;
    }

    clienteMQTT.loop();
    yield();
  }

  // 🛠️ CORREÇÃO: Divisão por 1.000.000.0f para obter o tempo correto em SEGUNDOS
  float T = (micros() - t0) / 1000000.0f; 

  // Validação da integridade temporal da janela
  if (T > JANELA_SEGUNDOS * 1.1f) { // Margem de 10% de estouro tolerada
    Serial.printf("Coleta descartada por estouro de tempo: %.3f s\n", T);
  } else {
    publicarBuffers(T);
  }

  if (millis() - ultimaVerificacao >= INTERVALO_VERIFICACAO) {
    ultimaVerificacao = millis();
    verificarConexao();
  }
  clienteMQTT.loop();
}
