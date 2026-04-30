//Wifi Base
#include <WiFi.h>

//config rede
const char* WIFI_SSID     = "NOME_DA_REDE";
const char* WIFI_PASSWORD = "SENHA_DA_REDE";

// tentativa inicial de conexao com tempo de espera
const unsigned long TIMEOUT_CONEXAO = 20000;

// Intervalo de verificacao de status
const unsigned long INTERVALO_VERIFICACAO = 10000;

unsigned long ultimaVerificacao = 0;

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

//verificar e reconectar se necessário
void verificarConexao() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Conexão perdida. Tentando reconectar...");
    WiFi.disconnect();
    conectarWiFi();
  } else {
    Serial.print("Wi-Fi ativo | IP: ");
    Serial.print(WiFi.localIP());
    Serial.print(" | RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  }
}

//Setup
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nInicializando módulo Wi-Fi");
  conectarWiFi();
}

void loop() {
  if (millis() - ultimaVerificacao >= INTERVALO_VERIFICACAO) {
    ultimaVerificacao = millis();
    verificarConexao();
  }

  // Aqui entrarão futuramente as rotinas de leitura do ADS1115
  // e o envio dos dados para o servidor do CEMADEN ou computador.
}
