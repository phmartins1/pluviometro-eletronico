/*
   OBJETIVO (nova abordagem: fazer o mínimo na placa, calcular na nuvem):
   - O ESP32 só MEDE. Ele lê os 4 piezos, preenche um buffer de N amostras
     por canal e publica os buffers BRUTOS (tensão em V) junto com o tempo
     real de coleta (T) e a geometria (áreas das faces).
   - TODO o cálculo de chuva (integral S, mm, mm/h, RMS, picos, impactos)
     passa a ser feito na NUVEM, que guarda o histórico e a forma de onda
     completa. Assim dá para recalibrar e mudar os algoritmos sem reprogramar
     o sensor.

   POR QUE MANDAR O SINAL CRU:
   - A nuvem recebe a forma de onda inteira, então pode recalcular qualquer
     métrica (integral, RMS, contagem de impactos, filtragem de artefatos)
     a qualquer momento, sobre todo o histórico.
   - A placa gasta menos: sem timer, sem acumuladores, sem multiplicação por
     constante de calibração. Entre coletas o ESP poderia até dormir.

   GEOMETRIA / ÁREA (para maior precisão na nuvem):
   - Faces maiores (A0, A2): 144 cm² cada.
   - Faces menores (A1, A3): 47,6 cm² cada.
   - Área total de captação: 2*144 + 2*47,6 = 383,2 cm².
   - As áreas vão no payload. A nuvem trata os 4 piezos como UM coletor maior
     (soma os integrais e divide pela área total). Isso reduz o ruído e dá uma
     lâmina (mm) mais precisa do que um piezo isolado. As faces maiores captam
     mais gotas e pesam mais; as menores pesam menos, na proporção da área.

   OBS de hardware:
   - Ligação single-ended (0 V a 3,3 V); o sinal negativo é cortado pelos diodos.

   Detalhes do cálculo na nuvem: ver calculo_chuva_piezo_thingsboard.md e o
   simulador de referência simulador_telemetria_v2.py.
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

// Pinos I2C no ESP32
#define SDA_PIN 21
#define SCL_PIN 22
#define CHANNEL 0
#define CHANNEL_1 1
#define CHANNEL_2 2
#define CHANNEL_3 3

const int   SAMPLE_RATE = 860;        // máximo do ADS1115 (SPS)
const int   BUFFER_SIZE = 300;        // N amostras por canal antes de publicar
const float ADC_LSB     = 0.000125;   // GAIN_ONE = 0,125 mV/bit
const float NOISE_FLOOR = 0.003;      // abaixo disso é ruído, vira 0 (V)

// Geometria das faces (cm²). Vão no payload para a nuvem ponderar por área.
const float AREA_MAIOR = 144.0;       // A0, A2
const float AREA_MENOR = 47.6;        // A1, A3

// Descarte por timeout: se a coleta dos N pontos demorar mais que isso,
// provavelmente houve travamento do I2C ou leitura anômala. Descarta sem
// publicar. (A filtragem de artefatos "de verdade", como folha/inseto que
// geram sinal constante, é feita na nuvem, que tem a forma de onda completa.)
const float TIMEOUT_BUFFER = 5.0;     // s

// Buffers de leitura (tensão em V), um por canal.
float samples_PiezoA0[BUFFER_SIZE];
float samples_PiezoA1[BUFFER_SIZE];
float samples_PiezoA2[BUFFER_SIZE];
float samples_PiezoA3[BUFFER_SIZE];

// O payload com 4 buffers de N floats é grande (alguns KB). O PubSubClient
// trunca acima do seu buffer interno (256 B por padrão), então é preciso
// aumentá-lo. Calculado folgado para 4 x BUFFER_SIZE amostras.
const uint16_t MQTT_BUFFER_SIZE = 20480;

// Conexões
const unsigned long TIMEOUT_CONEXAO = 20000;
const unsigned long INTERVALO_VERIFICACAO = 10000;
unsigned long ultimaVerificacao = 0;

WiFiClient clienteWiFi;
PubSubClient clienteMQTT(clienteWiFi);

// CONEXÃO WIFI
bool conectarWiFi()
{
  Serial.print("Conectando à rede: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long inicio = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
    if (millis() - inicio > TIMEOUT_CONEXAO)
    {
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

// CONEXÃO BROKER MQTT
void conectarBrokerMQTT()
{
  if (WiFi.status() != WL_CONNECTED)
    return;

  Serial.println("Conectando ao Broker MQTT...");

  unsigned long inicio = millis();
  while (!clienteMQTT.connected())
  {
    if (millis() - inicio > TIMEOUT_CONEXAO)
    {
      Serial.println("\nFalha: tempo de conexão ao Broker MQTT esgotado.");
      return;
    }

    char IDCliente[50];
    sprintf(IDCliente, "PEC-Pluviometro-%06X", (uint32_t)ESP.getEfuseMac());

    if (clienteMQTT.connect(IDCliente, TOKEN_BROKER, ""))
    {
      Serial.println("Conectado ao Broker MQTT.");
      return;
    }
    else
    {
      Serial.print("Falha, rc=");
      Serial.println(clienteMQTT.state()); // diagnostico de falha
      delay(2000);
    }
  }
}

// verificar e reconectar se necessário
void verificarConexao()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Conexão WiFi perdida. Tentando reconectar...");
    WiFi.disconnect();
    conectarWiFi();
  }
  else
  {
    Serial.print("Wi-Fi ativo | IP: ");
    Serial.print(WiFi.localIP());
    Serial.print(" | RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  }

  if (!clienteMQTT.connected())
  {
    Serial.println("Conexão ao Broker MQTT perdida. Tentando reconectar...");
    clienteMQTT.disconnect();
    conectarBrokerMQTT();
  }
}

// Publica os 4 buffers brutos + tempo real + geometria.
// A nuvem usa T para escalar a taxa e as áreas para ponderar por face.
// O 'ident' é obrigatório: o stream flespi -> ThingsBoard usa ele para
// identificar o device. Sem ident a mensagem não chega no ThingsBoard.
void publicarBuffers(float T)
{
  JsonDocument doc;

  doc["ident"]          = "pluviometro_teste";
  doc["T"]              = T;             // duração real da coleta (s)
  doc["n"]              = BUFFER_SIZE;   // amostras por canal
  doc["area_maior_cm2"] = AREA_MAIOR;    // A0, A2
  doc["area_menor_cm2"] = AREA_MENOR;    // A1, A3

  JsonArray a0 = doc["buf_A0"].to<JsonArray>();
  JsonArray a1 = doc["buf_A1"].to<JsonArray>();
  JsonArray a2 = doc["buf_A2"].to<JsonArray>();
  JsonArray a3 = doc["buf_A3"].to<JsonArray>();

  for (int i = 0; i < BUFFER_SIZE; i++)
  {
    a0.add(samples_PiezoA0[i]);
    a1.add(samples_PiezoA1[i]);
    a2.add(samples_PiezoA2[i]);
    a3.add(samples_PiezoA3[i]);
  }

  String payload;
  serializeJson(doc, payload);

  if (clienteMQTT.connected())
  {
    bool publicado = clienteMQTT.publish(TOPICO_MQTT, payload.c_str());
    if (publicado)
      Serial.printf("Publicado: %d amostras/canal | T=%.3f s | %u bytes\n",
                    BUFFER_SIZE, T, (unsigned)payload.length());
    else
      Serial.printf("Falha ao publicar (payload=%u B; aumente MQTT_BUFFER_SIZE, atual=%u B)\n",
                    (unsigned)payload.length(), (unsigned)MQTT_BUFFER_SIZE);
  }
  else
  {
    Serial.println("Sem conexão MQTT, telemetria não enviada.");
  }
}

// SETUP
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!ads.begin()) {
    Serial.println("ERRO: ADS1115 não encontrado!");
    while (1);
  }

  ads.setGain(GAIN_ONE);                 // até 4.096 V
  ads.setDataRate(RATE_ADS1115_860SPS);  // máxima velocidade

  Serial.println("\nInicializando módulo Wi-Fi");
  conectarWiFi();

  // Aumenta o buffer interno do PubSubClient ANTES de conectar, senão o
  // payload com os 4 buffers é truncado e o publish falha.
  clienteMQTT.setBufferSize(MQTT_BUFFER_SIZE);
  clienteMQTT.setServer(BROKER_MQTT, PORTA_MQTT);
  conectarBrokerMQTT();

  Serial.println("Sistema iniciado (modo: envio de buffers brutos para a nuvem)");
}

// LOOP
void loop() {

  // --- preenche os 4 buffers em um único loop cronometrado ---
  // Os 4 canais são lidos intercalados (A0, A1, A2, A3 por iteração), então
  // o tempo T vale para os quatro. readADC_SingleEnded já espera a conversão,
  // logo o próprio ADS limita a taxa (não precisa de delay).
  unsigned long t0 = millis();
  for (int i = 0; i < BUFFER_SIZE; i++) {

    int16_t raw_PiezoA0 = ads.readADC_SingleEnded(CHANNEL);
    int16_t raw_PiezoA1 = ads.readADC_SingleEnded(CHANNEL_1);
    int16_t raw_PiezoA2 = ads.readADC_SingleEnded(CHANNEL_2);
    int16_t raw_PiezoA3 = ads.readADC_SingleEnded(CHANNEL_3);

    // Converte para volts
    float voltage_PiezoA0 = raw_PiezoA0 * ADC_LSB;
    float voltage_PiezoA1 = raw_PiezoA1 * ADC_LSB;
    float voltage_PiezoA2 = raw_PiezoA2 * ADC_LSB;
    float voltage_PiezoA3 = raw_PiezoA3 * ADC_LSB;

    // Remove ruído muito baixo (também encolhe o payload: vira 0)
    if (voltage_PiezoA0 < NOISE_FLOOR) voltage_PiezoA0 = 0;
    if (voltage_PiezoA1 < NOISE_FLOOR) voltage_PiezoA1 = 0;
    if (voltage_PiezoA2 < NOISE_FLOOR) voltage_PiezoA2 = 0;
    if (voltage_PiezoA3 < NOISE_FLOOR) voltage_PiezoA3 = 0;

    samples_PiezoA0[i] = voltage_PiezoA0;
    samples_PiezoA1[i] = voltage_PiezoA1;
    samples_PiezoA2[i] = voltage_PiezoA2;
    samples_PiezoA3[i] = voltage_PiezoA3;
  }

  float T = (millis() - t0) / 1000.0;  // duração REAL da coleta (s)

  // --- publica os buffers (ou descarta por timeout) ---
  if (T > TIMEOUT_BUFFER) {
    Serial.printf("Coleta descartada: T=%.2f s > %.2f s (possível travamento)\n",
                  T, TIMEOUT_BUFFER);
  } else {
    publicarBuffers(T);
  }

  // Reconexão periódica (não bloqueia a coleta)
  if (millis() - ultimaVerificacao >= INTERVALO_VERIFICACAO) {
    ultimaVerificacao = millis();
    verificarConexao();
  }

  clienteMQTT.loop();
}
