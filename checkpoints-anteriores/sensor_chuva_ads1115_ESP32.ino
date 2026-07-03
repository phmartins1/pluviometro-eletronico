/*
   OBJETIVO:
   - Ler tensão do piezo ao longo do tempo
   - Detectar impactos de chuva
   - Medir:
       * Pico de tensão
       * Energia RMS
       * Taxa de impactos
       * Taxa de pulsos (threshold crossings)
   - Saída compatível com Serial Plotter

   OBS:
   ligação é single-ended (0V a 3.3V),
   o sinal negativo é cortado pelos diodos.
   detectar PULSOS/IMPACTOS.
*/

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#include "esp_sleep.h"
#include "esp_wifi.h"
#include "esp_bt.h"
#include <WiFi.h>

Adafruit_ADS1115 ads;

// CONFIGURAÇÕES

// Pinos I2C no ESP32
#define SDA_PIN 21
#define SCL_PIN 22
#define CHANNEL 0

const int SAMPLE_RATE = 860;        // máximo ADS1115
const int BUFFER_SIZE = 300;        // ~348 ms de janela
const float THRESHOLD = 0.03;       // 30mV (mais sensível) podemos alterar aqui caso precisarmos de aumentar/diminuir a sensibilidade
const float ADC_LSB = 0.000125;     // GAIN_ONE = 0.125mV/bit
float samples[BUFFER_SIZE];


// =============================
// CONFIGURAÇÕES DE ECONOMIA
// =============================

const unsigned long NO_RAIN_SLEEP_TIME = 30000; // 30 segundos // Tempo sem chuva antes de dormir
const uint64_t SLEEP_TIME_US = 60000000; // 60 segundos // Tempo do LIGHT SLEEP
unsigned long lastRainTime = 0; // Controle de última chuva

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

// Estima a taxa de pulsos (Pulse Rate) contando quantas vezes o sinal
// cruza o threshold de baixo para cima dentro da janela de amostras.
//
// Cada cruzamento representa o início de um novo pulso detectado.
// Isso ajuda a medir a frequência de impactos ao longo do tempo.
//
// Etapas:
// 1. Percorre as amostras comparando a atual com a anterior
// 2. Detecta quando o sinal sobe de abaixo do threshold para acima dele
// 3. Conta cada cruzamento como um pulso
// 4. Calcula a duração total da janela (amostras / taxa de amostragem)
// 5. Divide cruzamentos pelo tempo para obter pulsos por segundo (Hz)
float estimatePulseRate(float *buffer, int size, float threshold, float sampleRate) {
  int crossings = 0;

  for (int i = 1; i < size; i++) {
    if (buffer[i - 1] < threshold && buffer[i] >= threshold) {
      crossings++;
    }
  }

  float duration = (float)size / sampleRate;

  return crossings / duration;
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

  // ECONOMIA DE ENERGIA
  
  btStop(); // Desliga Bluetooth
  lastRainTime = millis(); // Inicializa contador de chuva

  Serial.println("Sistema iniciado");
  Serial.println("tensao,pico,rms,impactos,pulseRate");

}

// LOOP
void loop() {

  
  // Coleta de amostras
  for (int i = 0; i < BUFFER_SIZE; i++) {

    int16_t raw = ads.readADC_SingleEnded(CHANNEL);

    // Converte para volts
    float voltage = raw * ADC_LSB;

    // Remove ruído muito baixo
    if (voltage < 0.003) {
      voltage = 0;
    }

    samples[i] = voltage;

    // ADS1115 já limita pela taxa interna
    delay(1);
  }

  // Análises
  float peak = calculatePeak(samples, BUFFER_SIZE);
  float rms = calculateRMS(samples, BUFFER_SIZE);
  int impacts = countImpacts(samples, BUFFER_SIZE, THRESHOLD);
  float pulseRate = estimatePulseRate(samples, BUFFER_SIZE, THRESHOLD, SAMPLE_RATE);

 // Detectou chuva
  if (impacts > 0) {
    lastRainTime = millis();
  }
  // Classificação de chuva (Aqui coloquei um exemplo simples, precisa ser mais elaborado)
  const char* chuva;

  if (impacts < 1) {
    chuva = "Sem Chuva";
  }
  else if (impacts < 8) {
    chuva = "Chuva Fraca";
  }
  else if (impacts < 20) {
    chuva = "Chuva Moderada";
  }
  else {
    chuva = "Chuva Forte";
  }


  // SERIAL MONITOR
  Serial.println("--------------------------------");

  Serial.print("Pico: ");
  Serial.print(peak, 4);
  Serial.println(" V");

  Serial.print("RMS: ");
  Serial.print(rms, 4);
  Serial.println(" V");

  Serial.print("Impactos: ");
  Serial.println(impacts);

  Serial.print("PulseRate: ");
  Serial.print(pulseRate, 2);
  Serial.println(" Hz");

  Serial.print("Status: ");
  Serial.println(chuva);

  // =============================
  // LIGHT SLEEP AUTOMÁTICO
  // =============================

  // Se ficou muito tempo sem chuva
  if (millis() - lastRainTime > NO_RAIN_SLEEP_TIME) {

    Serial.println("Entrando em LIGHT SLEEP...");
    Serial.flush(); 
    esp_sleep_enable_timer_wakeup(SLEEP_TIME_US); // acorda automaticamente após tempo definido
    esp_light_sleep_start(); // entra em light sleep
    Serial.println("Sistema acordou!");

  }
}