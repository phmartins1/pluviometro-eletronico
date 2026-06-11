/*
   OBJETIVO:
   - Ler tensão do piezo ao longo do tempo
   - Detectar impactos de chuva
   - Medir:
       * Pico de tensão
       * Energia RMS
       * Taxa de impactos
       * Integral do sinal (S) -> base do cálculo de chuva (mm e mm/h)
   - Publicar a telemetria no broker MQTT (ThingsBoard / flespi)

   OBS:
   ligação é single-ended (0V a 3.3V),
   o sinal negativo é cortado pelos diodos.
   detectar PULSOS/IMPACTOS.

   DIVISÃO DE RESPONSABILIDADES (ver calculo_chuva_piezo_thingsboard.md):
   - O ESP32 apenas MEDE: integra o sinal (S, em V·s) e acumula ~10 s.
   - O ThingsBoard INTERPRETA: aplica a constante de calibração c e agrega
     para gerar acumulado (mm), intensidade real (mm/h) e instantânea (mm/h).
   Por isso o ESP NÃO multiplica por c nem calcula mm/h: manda só o S cru.
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
#define CHANNEL_2 2

const int SAMPLE_RATE = 860;        // máximo ADS1115
const int BUFFER_SIZE = 300;        // ~348 ms de janela (N amostras)
const float THRESHOLD = 0.03;       // 30mV (mais sensível) podemos alterar aqui caso precisarmos de aumentar/diminuir a sensibilidade
const float ADC_LSB = 0.000125;     // GAIN_ONE = 0.125mV/bit
const float NOISE_FLOOR = 0.003;    // abaixo disso é ruído, vira 0

float samplesA0[BUFFER_SIZE];       //Buffer leityra canal 1
float samplesA2[BUFFER_SIZE];       //Buffer leityra canal 2

// Conexões
const unsigned long TIMEOUT_CONEXAO = 20000;
const unsigned long INTERVALO_VERIFICACAO = 10000;
unsigned long ultimaVerificacao = 0;

WiFiClient clienteWiFi;
PubSubClient clienteMQTT(clienteWiFi);

// Intervalo de reporte: acumula ~10 s e publica o S somado (ver doc, seção 7).
const unsigned long INTERVALO_PUBLICACAO = 10000;

// Acumuladores do intervalo de reporte (Entrada A0)
float    S_reportA0        = 0;   // soma dos S de cada janela (V·s) -> principal
uint32_t impactos_reportA0 = 0;   // contagem de impactos no intervalo
float    pico_reportA0     = 0;   // maior tensão no intervalo (diagnóstico)
double   somaQuad_reportA0 = 0;   // soma dos quadrados (para o RMS do intervalo)
uint32_t nAmostras_reportA0 = 0;  // total de amostras lidas no intervalo
unsigned long t_ini_reportA0 = 0; // marca o início do intervalo de reporte


// Acumuladores do intervalo de reporte (Entrada A2)
float    S_reportA2        = 0;   // soma dos S de cada janela (V·s) -> principal
uint32_t impactos_reportA2 = 0;   // contagem de impactos no intervalo
float    pico_reportA2     = 0;   // maior tensão no intervalo (diagnóstico)
double   somaQuad_reportA2 = 0;   // soma dos quadrados (para o RMS do intervalo)
uint32_t nAmostras_reportA2 = 0;  // total de amostras lidas no intervalo
unsigned long t_ini_reportA2 = 0; // marca o início do intervalo de reporte


// FUNÇÕES

// Calcula RMS (Root Mean Square / valor quadrático médio)
// Mede a energia média do sinal na janela de amostras.
// Útil para estimar intensidade geral da chuva:
// - RMS baixo = pouco impacto / chuva fraca
// - RMS alto = muitos impactos / chuva forte
//
// Etapas:
// 1. Eleva cada amostra ao quadrado
// 2. Soma tudo
// 3. Divide pelo número de amostras
// 4. Aplica raiz quadrada para voltar à unidade em volts
// Calcula RMS
float calculateRMS(float *buffer, int size) {
  double sum = 0;

  for (int i = 0; i < size; i++) {
    sum += buffer[i] * buffer[i];
  }

  return sqrt(sum / size);
}

// Pico máximo
// Calcula o pico máximo de tensão na janela de amostras
// Mede o maior valor detectado pelo sensor no período analisado.
// Útil para identificar o impacto mais forte (ex: gota maior ou batida mais intensa).
//
// Etapas:
// 1. Percorre todas as amostras
// 2. Compara cada valor com o maior já encontrado
// 3. Atualiza se encontrar um valor maior
// 4. Retorna o maior pico de tensão registrado
float calculatePeak(float *buffer, int size) {
  float peak = 0;

  for (int i = 0; i < size; i++) {
    if (buffer[i] > peak) {
      peak = buffer[i];
    }
  }

  return peak;
}

// Soma linear das tensões da janela (Σ|vi|).
// É a base do integral S: somaV / N é a tensão média, e multiplicada pelo
// tempo da janela vira a "área" sob o sinal (V·s).
float sumVoltage(float *buffer, int size) {
  float soma = 0;

  for (int i = 0; i < size; i++) {
    soma += buffer[i];
  }

  return soma;
}

/// Conta quantos impactos (gotas/pulsos) ocorreram na janela de amostras.
// Um impacto é registrado quando o sinal ultrapassa o threshold definido.
//
// Cooldown (período morto):
// Após detectar um impacto, o sistema ignora algumas leituras por alguns ms
// para evitar contar várias oscilações da mesma gota como impactos diferentes.
//
// Etapas:
// 1. Percorre todas as amostras
// 2. Verifica se está em cooldown
// 3. Se estiver, ignora a leitura
// 4. Se o valor passar do threshold, conta um novo impacto
// 5. Ativa cooldown para evitar múltiplas contagens do mesmo evento
int countImpacts(float *buffer, int size, float threshold) {
  int count = 0;
  int cooldown = 0;

  for (int i = 0; i < size; i++) {

    // Ignora leituras durante período morto
    if (cooldown > 0) {
      cooldown--;
      continue;
    }

    // Novo impacto
    if (buffer[i] > threshold) {
      count++;
      cooldown = 8; // ~9 ms em 860 SPS
    }
  }

  return count;
}

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

// Monta e publica a telemetria do intervalo de reporte.
// Campos (ver doc, seção 4):
//   S        -> integral acumulado do intervalo (V·s)  [PRINCIPAL]
//   T        -> duração real do intervalo (s)
//   impactos -> contagem de impactos no intervalo
//   pico     -> maior tensão no intervalo (diagnóstico)
//   rms      -> RMS do sinal no intervalo (diagnóstico)
// IMPORTANTE: envia o INCREMENTO do intervalo (não um total que só cresce),
// para o SUM do ThingsBoard funcionar direto.
void publicarTelemetria(float S, float T, uint32_t impactos, float pico, float rms)
{
  JsonDocument doc;

  // O 'ident' e obrigatorio: o stream flespi -> ThingsBoard usa ele para
  // identificar o device. Sem ident a mensagem nao chega no ThingsBoard.
  doc["ident"] = "pluviometro_teste";
  doc["S"] = S;
  doc["T"] = T;
  doc["impactos"] = impactos;
  doc["pico"] = pico;
  doc["rms"] = rms;

  String payload;
  serializeJson(doc, payload);

  if (clienteMQTT.connected())
  {
    bool publicado = clienteMQTT.publish(TOPICO_MQTT, payload.c_str());
    if (publicado)
      Serial.println("Publicado: " + payload);
    else
      Serial.println("Falha ao publicar");
  }
  else
  {
    Serial.println("Sem conexão MQTT, telemetria não enviada: " + payload);
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

  ads.setGain(GAIN_ONE);                  // até 4.096V
  ads.setDataRate(RATE_ADS1115_860SPS);  // máxima velocidade

  Serial.println("\nInicializando módulo Wi-Fi");
  conectarWiFi();

  clienteMQTT.setServer(BROKER_MQTT, PORTA_MQTT);
  conectarBrokerMQTT();

  Serial.println("Sistema iniciado");
  t_ini_reportA0 = millis();
  t_ini_reportA2 = millis();
}

// LOOP
void loop() {

  // --- coleta uma janela cronometrada de N amostras ---
  unsigned long t0 = millis();
  for (int i = 0; i < BUFFER_SIZE; i++) {

    //int16_t raw = ads.readADC_SingleEnded(CHANNEL);
    int16_t rawA0 = ads.readADC_SingleEnded(CHANNEL);
    int16_t rawA2 = ads.readADC_SingleEnded(CHANNEL_2);

    // Converte para volts
    //float voltage = raw * ADC_LSB;
    float voltageA0 = rawA0 * ADC_LSB;
    float voltageA2 = rawA2 * ADC_LSB;


    // Remove ruído muito baixo
    //if (voltage < NOISE_FLOOR) voltage = 0;
    if (voltageA0 < NOISE_FLOOR) voltageA0 = 0;
    if (voltageA2 < NOISE_FLOOR) voltageA2 = 0;

    //samples[i] = voltage;
    samplesA0[i] = voltageA0;
    samplesA2[i] = voltageA2;

    // ADS1115 já limita pela taxa interna
    delay(1);
  }
  float T_janela = (millis() - t0) / 1000.0;  // duração REAL da janela (s)

  // --- análises da janela ---
  //float somaV = sumVoltage(samples, BUFFER_SIZE);
  //float peak = calculatePeak(samples, BUFFER_SIZE);
  //int impacts = countImpacts(samples, BUFFER_SIZE, THRESHOLD);
  float peakA0 = calculatePeak(samplesA0, BUFFER_SIZE);
  float peakA2 = calculatePeak(samplesA2, BUFFER_SIZE);

  int impactsA0 = countImpacts(samplesA0, BUFFER_SIZE, THRESHOLD);
  int impactsA2 = countImpacts(samplesA2, BUFFER_SIZE, THRESHOLD);

  float somaVA0 = sumVoltage(samplesA0, BUFFER_SIZE);
  float somaVA2 = sumVoltage(samplesA2, BUFFER_SIZE);

  // Integral da janela: S = (tensão média) * tempo da janela = somaV * T / N
  //float S_janela = somaV * T_janela / BUFFER_SIZE;
  float S_janelaA0 = somaVA0 * T_janela / BUFFER_SIZE;
  float S_janelaA2 = somaVA2 * T_janela / BUFFER_SIZE;

  // --- acúmulo no intervalo de reporte ---
  S_reportA0 += S_janelaA0;
  S_reportA2 += S_janelaA2;

  impactos_reportA0 += impactsA0;
  impactos_reportA2 += impactsA2;

  if (peakA0 > pico_reportA0) pico_reportA0 = peakA0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    somaQuad_reportA0 += (double)samplesA0[i] * samplesA0[i];
  }
  nAmostras_reportA0 += BUFFER_SIZE;

  if (peakA2 > pico_reportA2) pico_reportA2 = peakA2;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    somaQuad_reportA2 += (double)samplesA2[i] * samplesA2[i];
  }
  nAmostras_reportA2 += BUFFER_SIZE;

  // Classificação de chuva (exemplo simples; refinar futuramente)
  const char* chuva;
  int impacts = (impactsA0 + impactsA2);
  if (impacts < 1)        chuva = "Sem Chuva";
  else if (impacts < 8)   chuva = "Chuva Fraca";
  else if (impacts < 20)  chuva = "Chuva Moderada";
  else                    chuva = "Chuva Forte";

  // SERIAL MONITOR (diagnóstico da janela)
  Serial.println("--------------------------------");

  Serial.print("Pico Piezo 1: ");      Serial.print(peakA0, 4);      Serial.println(" V");
  Serial.print("Pico Piezo 2: ");      Serial.print(peakA2, 4);      Serial.println(" V");


  Serial.print("Impactos Somados (Piezo 1 + Piezo 2): ");  Serial.println(impacts);

  Serial.print("S janela Piezo 1: ");  Serial.print(S_janelaA0, 6);  Serial.println(" V.s");
  Serial.print("S janela Piezo 2: ");  Serial.print(S_janelaA2, 6);  Serial.println(" V.s");

  Serial.print("Status: ");    Serial.println(chuva);

  // --- a cada INTERVALO_PUBLICACAO, publica o acumulado e zera ---
  if (millis() - t_ini_reportA0 >= INTERVALO_PUBLICACAO) {
    float T_reportA0 = (millis() - t_ini_reportA0) / 1000.0;
    float rms_reportA0 = nAmostras_reportA0 > 0
                         ? sqrt(somaQuad_reportA0 / nAmostras_reportA0)
                         : 0;
               
    publicarTelemetria(S_reportA0, T_reportA0, impactos_reportA0, pico_reportA0, rms_reportA0);
    

    // zera os acumuladores do próximo intervalo
    S_reportA0 = 0;
    impactos_reportA0 = 0;
    pico_reportA0 = 0;
    somaQuad_reportA0 = 0;
    nAmostras_reportA0 = 0;
    t_ini_reportA0 = millis();

  }

  if (millis() - t_ini_reportA2 >= INTERVALO_PUBLICACAO) {
    float T_reportA2 = (millis() - t_ini_reportA2) / 1000.0;
    float rms_reportA2 = nAmostras_reportA2 > 0
                         ? sqrt(somaQuad_reportA2 / nAmostras_reportA2)
                         : 0;

    publicarTelemetria(S_reportA2, T_reportA2, impactos_reportA2, pico_reportA2, rms_reportA2);

    
    S_reportA2 = 0;
    impactos_reportA2 = 0;
    pico_reportA2 = 0;
    somaQuad_reportA2 = 0;
    nAmostras_reportA2 = 0;
    t_ini_reportA2 = millis();
  }     

  // Reconexão periódica (não bloqueia a coleta)
  if (millis() - ultimaVerificacao >= INTERVALO_VERIFICACAO) {
    ultimaVerificacao = millis();
    verificarConexao();
  }

  clienteMQTT.loop();
}
