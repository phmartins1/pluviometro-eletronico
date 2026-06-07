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

const int SAMPLE_RATE = 860;        // máximo ADS1115
const int BUFFER_SIZE = 300;        // ~348 ms de janela (N amostras)
const float THRESHOLD = 0.03;       // 30mV (mais sensível) podemos alterar aqui caso precisarmos de aumentar/diminuir a sensibilidade
const float ADC_LSB = 0.000125;     // GAIN_ONE = 0.125mV/bit
const float NOISE_FLOOR = 0.003;    // abaixo disso é ruído, vira 0

float samples[BUFFER_SIZE];

// Conexões
const unsigned long TIMEOUT_CONEXAO = 20000;
const unsigned long INTERVALO_VERIFICACAO = 10000;
unsigned long ultimaVerificacao = 0;

WiFiClient clienteWiFi;
PubSubClient clienteMQTT(clienteWiFi);

// Intervalo de reporte: acumula ~10 s e publica o S somado (ver doc, seção 7).
const unsigned long INTERVALO_PUBLICACAO = 10000;

// Acumuladores do intervalo de reporte
float    S_report        = 0;   // soma dos S de cada janela (V·s) -> principal
uint32_t impactos_report = 0;   // contagem de impactos no intervalo
float    pico_report     = 0;   // maior tensão no intervalo (diagnóstico)
double   somaQuad_report = 0;   // soma dos quadrados (para o RMS do intervalo)
uint32_t nAmostras_report = 0;  // total de amostras lidas no intervalo
unsigned long t_ini_report = 0; // marca o início do intervalo de reporte

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
  t_ini_report = millis();
}

// LOOP
void loop() {

  // --- coleta uma janela cronometrada de N amostras ---
  unsigned long t0 = millis();
  for (int i = 0; i < BUFFER_SIZE; i++) {

    int16_t raw = ads.readADC_SingleEnded(CHANNEL);

    // Converte para volts
    float voltage = raw * ADC_LSB;

    // Remove ruído muito baixo
    if (voltage < NOISE_FLOOR) {
      voltage = 0;
    }

    samples[i] = voltage;

    // ADS1115 já limita pela taxa interna
    delay(1);
  }
  float T_janela = (millis() - t0) / 1000.0;  // duração REAL da janela (s)

  // --- análises da janela ---
  float somaV = sumVoltage(samples, BUFFER_SIZE);
  float peak = calculatePeak(samples, BUFFER_SIZE);
  int impacts = countImpacts(samples, BUFFER_SIZE, THRESHOLD);

  // Integral da janela: S = (tensão média) * tempo da janela = somaV * T / N
  float S_janela = somaV * T_janela / BUFFER_SIZE;

  // --- acúmulo no intervalo de reporte ---
  S_report += S_janela;
  impactos_report += impacts;
  if (peak > pico_report) pico_report = peak;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    somaQuad_report += (double)samples[i] * samples[i];
  }
  nAmostras_report += BUFFER_SIZE;

  // Classificação de chuva (exemplo simples; refinar futuramente)
  const char* chuva;
  if (impacts < 1)        chuva = "Sem Chuva";
  else if (impacts < 8)   chuva = "Chuva Fraca";
  else if (impacts < 20)  chuva = "Chuva Moderada";
  else                    chuva = "Chuva Forte";

  // SERIAL MONITOR (diagnóstico da janela)
  Serial.println("--------------------------------");
  Serial.print("Pico: ");      Serial.print(peak, 4);      Serial.println(" V");
  Serial.print("Impactos: ");  Serial.println(impacts);
  Serial.print("S janela: ");  Serial.print(S_janela, 6);  Serial.println(" V.s");
  Serial.print("Status: ");    Serial.println(chuva);

  // --- a cada INTERVALO_PUBLICACAO, publica o acumulado e zera ---
  if (millis() - t_ini_report >= INTERVALO_PUBLICACAO) {
    float T_report = (millis() - t_ini_report) / 1000.0;
    float rms_report = nAmostras_report > 0
                         ? sqrt(somaQuad_report / nAmostras_report)
                         : 0;

    publicarTelemetria(S_report, T_report, impactos_report, pico_report, rms_report);

    // zera os acumuladores do próximo intervalo
    S_report = 0;
    impactos_report = 0;
    pico_report = 0;
    somaQuad_report = 0;
    nAmostras_report = 0;
    t_ini_report = millis();
  }

  // Reconexão periódica (não bloqueia a coleta)
  if (millis() - ultimaVerificacao >= INTERVALO_VERIFICACAO) {
    ultimaVerificacao = millis();
    verificarConexao();
  }

  clienteMQTT.loop();
}
