/*
  Projeto ESP32: Monitoramento de Respiração com Guias de Respiração,
  LEDs Indicadores de Estado, GPS NEO-6M e Envio via Bluetooth

  Modificações:
    • Envia sempre a localização (última lat/lon válida) + uma variável booleana
      “rapid30s” que é true se houver respiração rápida contínua por 30s;
      enquanto não ocorrer ou a respiração voltar a ficar “tranquila” (verde),
      rapid30s fica false.
    • Lógica de detecção de 30s contínuos de respiração rápida:
        – Se em cada janela de 3s o número de picos >= 5, conta como “rápida”.
        – Quando a primeira janela rápida é detectada, inicia timer.
        – Se mantiver “rápida” em todas as janelas subsequentes e chegar a 30s,
          rapid30s = true.
        – Se em qualquer janela o número de picos < 5, reset do timer e rapid30s = false.

  - Lê picos de pressão de um sensor (GPIO34) para detectar respirações.
  - Classifica a respiração como "rápida", "média" ou "tranquila" e acende LEDs:
      • Vermelho (rápida)
      • Amarelo (média)
      • Verde (tranquila)
  - Guia de respiração independente:
      • Inalar: LED “Inhale” aceso por 3 segundos
      • Exalar: LED “Exhale” aceso por 5 segundos
  - Módulo GPS NEO-6M (Serial2 em RX2=16, TX2=17) para obter coordenadas
  - Variável boolean “rapid30s” que indica 30s contínuos de respiração rápida
  - Envio via Bluetooth (SerialBT) dos dados de:
      • Taxa de respiração (status e BPM estimado)
      • Última localização válida (lat/lon)
      • Estado da variável rapid30s (true/false)

  Mapeamento de pinos (ESP32 DevKit V1):
    - sensorPin (entrada analógica): GPIO 34
    - ledRedPin     : GPIO 25
    - ledYellowPin  : GPIO 26
    - ledGreenPin   : GPIO 27
    - ledInhalePin  : GPIO 14
    - ledExhalePin  : GPIO 12
    - GPS RX2 (ESP32 lê do TX do NEO-6M): GPIO 16
    - GPS TX2 (ESP32 manda para RX do NEO-6M): GPIO 17
    - Bluetooth usa a interface interna SerialBT (sem pinos externos)

  Bibliotecas necessárias:
    - BluetoothSerial (para Bluetooth Classic no ESP32)
    - TinyGPSPlus      (para parse de NMEA do NEO-6M)
*/

#include <BluetoothSerial.h>
#include <TinyGPSPlus.h>

// ----- Configuração de pinos -----
// Sensor de pressão (ADC somente entrada):
const int sensorPin    = 34; // GPIO34 (ADC1_CH6) — apenas entrada analógica

// LEDs de status de respiração:
const int ledRedPin    = 25; // LED vermelho (respiração rápida)
const int ledYellowPin = 26; // LED amarelo (respiração média)
const int ledGreenPin  = 27; // LED verde (respiração tranquila)

// LEDs de guia de respiração:
const int ledInhalePin = 14; // LED “Inhale” (acende 3s)
const int ledExhalePin = 12; // LED “Exhale” (acende 5s)

// Limite para detectar “pico” de respiração (ADC raw):
const int THRESHOLD_RAW = 2000; 
// --- Ajuste esse valor conforme seu sensor/calibração.

// ----- Variáveis de controle de respirações -----
unsigned long lastWindowStart = 0;       // timestamp do início da janela de 3s
const unsigned long WINDOW_MS = 3000;    // janela de tempo para contar picos (3 segundos)

int peakCount = 0;            // contador de picos nesta janela de 3s
bool wasAboveThreshold = false; // controle para detectar borda de subida

// Variáveis para detecção de 30s contínuos de respiração rápida:
bool rapid30s = false;              // variável booleana que vai para true após 30s contínuos de respiração rápida
unsigned long rapidStartTime = 0;   // timestamp em que começou a primeira janela rápida
const unsigned long RAPID_DURATION_MS = 30000; // 30.000 ms = 30 segundos

// Intervalo de 60s sem respiração (alerta não usado nessa versão):
unsigned long lastPeakTime = 0;     // timestamp do último pico detectado
const unsigned long NO_BREATH_MS = 60000; // 60.000 ms = 60 segundos

bool noBreathAlertSent = false; // indicador para não reenviar GPS várias vezes (pode manter, mas sem uso ativo)

// ----- Variáveis de guia de respiração (inhale/exhale) -----
enum BreathGuideState { INHALE, EXHALE };
BreathGuideState guideState = INHALE;

unsigned long guideStateChangeTime = 0; 
const unsigned long INHALE_MS = 3000; // 3 segundos para "Inhale"
const unsigned long EXHALE_MS = 5000; // 5 segundos para "Exhale"

// ----- Configuração do GPS (NEO-6M) -----
#include <HardwareSerial.h>
HardwareSerial GPSSerial(2); // Serial2: RX2=16, TX2=17 no ESP32 DevKit

TinyGPSPlus gps;
double lastLat = 0.0;
double lastLon = 0.0;

// ----- Bluetooth Classic ----- 
BluetoothSerial SerialBT;

// ----- Função de inicialização -----
void setup() {
  // Serial para debug
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== Iniciando Sistema de Monitoramento de Respiração ===");

  // Configuração pinos
  pinMode(sensorPin, INPUT);       // ADC
  pinMode(ledRedPin, OUTPUT);
  pinMode(ledYellowPin, OUTPUT);
  pinMode(ledGreenPin, OUTPUT);
  pinMode(ledInhalePin, OUTPUT);
  pinMode(ledExhalePin, OUTPUT);

  // Inicializa estado dos LEDs
  digitalWrite(ledRedPin, LOW);
  digitalWrite(ledYellowPin, LOW);
  digitalWrite(ledGreenPin, LOW);
  digitalWrite(ledInhalePin, LOW);
  digitalWrite(ledExhalePin, LOW);

  // Inicializa janelas e timers
  lastWindowStart = millis();
  lastPeakTime = millis();
  guideStateChangeTime = millis();
  rapidStartTime = 0;
  rapid30s = false;

  // Inicializa GPS em 9600 bauds (NEO-6M padrão)
  GPSSerial.begin(9600, SERIAL_8N1, 16, 17); // RX2=16, TX2=17

  // Inicializa Bluetooth (nome “ESP32_BreathMon”)
  if (!SerialBT.begin("ESP32_BreathMon")) {
    Serial.println("Falha ao iniciar Bluetooth");
  } else {
    Serial.println("Bluetooth iniciado: ESP32_BreathMon");
  }
}

// ----- Função principal de loop -----
void loop() {
  unsigned long now = millis();

  // ----- 1. Leituras contínuas do GPS (para atualização de dados) -----
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
    if (gps.location.isUpdated()) {
      lastLat = gps.location.lat();
      lastLon = gps.location.lng();
    }
  }

  // ----- 2. Monitorar guias de respiração (Inhale/Exhale) -----
  handleBreathGuide(now);

  // ----- 3. Contar picos de respiração no sensor (janela de 3s) + lógica de 30s -----
  handleBreathPeaksAndRapid(now);

  // ----- 4. Enviar status e localização + rapid30s via Bluetooth a cada janela de 3s -----
  if (now - lastWindowStart >= WINDOW_MS) {
    sendBreathStatusViaBT(peakCount);
    // Reset para nova janela:
    peakCount = 0;
    lastWindowStart = now;
  }

  // (Opcional) Se desejar manter alerta de 60s sem respiração:
  // if ((now - lastPeakTime >= NO_BREATH_MS) && !noBreathAlertSent) { ... }
}

// -------------------------------------------------------------
// Função para guiar respiração: alterna LEDs “Inhale”/“Exhale”
// -------------------------------------------------------------
void handleBreathGuide(unsigned long now) {
  switch (guideState) {
    case INHALE:
      digitalWrite(ledInhalePin, HIGH);
      digitalWrite(ledExhalePin, LOW);
      if (now - guideStateChangeTime >= INHALE_MS) {
        guideState = EXHALE;
        guideStateChangeTime = now;
      }
      break;

    case EXHALE:
      digitalWrite(ledInhalePin, LOW);
      digitalWrite(ledExhalePin, HIGH);
      if (now - guideStateChangeTime >= EXHALE_MS) {
        guideState = INHALE;
        guideStateChangeTime = now;
      }
      break;
  }
}

// -------------------------------------------------------------
// Função para contar picos de respiração, acender LEDs de estado
// e controlar a lógica de 30s contínuos de respiração rápida
// -------------------------------------------------------------
void handleBreathPeaksAndRapid(unsigned long now) {
  int raw = analogRead(sensorPin);

  // Detecta borda de subida acima do limiar
  if (raw >= THRESHOLD_RAW) {
    if (!wasAboveThreshold) {
      // É um pico: conta e registra timestamp
      peakCount++;
      lastPeakTime = now;
      noBreathAlertSent = false; // (sem uso ativo, mas mantido)
      wasAboveThreshold = true;
    }
  } else {
    wasAboveThreshold = false;
  }

  // Determina estado “rápido” nesta janela atual (baseado em peakCount)
  bool isRapidThisWindow = (peakCount >= 5);

  // Lógica de 30s contínuos de “rápido”:
  if (isRapidThisWindow) {
    if (rapidStartTime == 0) {
      // Primeira janela rápida detectada
      rapidStartTime = now;
      rapid30s = false;
    } else {
      // Já havia começado a contar janelas rápidas
      if (!rapid30s && (now - rapidStartTime >= RAPID_DURATION_MS)) {
        // Passou 30s contínuos em estado rápido
        rapid30s = true;
      }
    }
  } else {
    // Não está mais rápido nesta janela → reseta
    rapidStartTime = 0;
    rapid30s = false;
  }

  // Durante a janela, indicar visualmente:
  // - Rápida: peakCount >= 5 → vermelho
  // - Média : 2 <= peakCount < 5 → amarelo
  // - Tranquila: peakCount < 2 → verde
  if (peakCount >= 5) {
    setStateLEDs(HIGH, LOW, LOW); // Vermelho ligado
  } else if (peakCount >= 2) {
    setStateLEDs(LOW, HIGH, LOW); // Amarelo ligado
  } else {
    setStateLEDs(LOW, LOW, HIGH); // Verde ligado
  }
}

// -------------------------------------------------------------
// Acende/apaga LEDs de estado (vermelho, amarelo, verde)
// -------------------------------------------------------------
void setStateLEDs(bool redOn, bool yellowOn, bool greenOn) {
  digitalWrite(ledRedPin, redOn);
  digitalWrite(ledYellowPin, yellowOn);
  digitalWrite(ledGreenPin, greenOn);
}

// -------------------------------------------------------------
// Calcula taxa, envia status de respiração + localização + rapid30s via Bluetooth
// -------------------------------------------------------------
void sendBreathStatusViaBT(int peaksInWindow) {
  // Interpretação simplificada:
  // peaksInWindow = número de picos em 3s
  // Se 5 ou mais em 3s -> rápida (~>=100 cpm)
  // Se 2 a 4 em 3s -> média (~40-80 cpm)
  // Se 0 a 1 em 3s -> tranquila (~<=20 cpm)

  String status;
  int breathsPerMinute = (peaksInWindow * 60) / 3; // extrapola picos em 3s para 1 min

  if (peaksInWindow >= 5) {
    status = "RAPIDA";
  } 
  else if (peaksInWindow >= 2) {
    status = "MEDIA";
  } 
  else {
    status = "TRANQUILA";
  }

  // Monta a mensagem completa:
  // Exemplo: "Resp: MEDIA | Calc: 2 | BPM: 40 | Lat: -23.567890 | Lon: -46.123456 | Rapid30s: 0"
  String msg = "Resp: " + status +
               "Calc: " + peaksInWindow +
               " | BPM: " + String(breathsPerMinute) +
               " | Lat: " + String(lastLat, 6) +
               " | Lon: " + String(lastLon, 6) +
               " | Rapid30s: " + String(rapid30s ? 1 : 0);

  // Envia para o Serial (debug) e para o Bluetooth:
  Serial.println(msg);
  SerialBT.println(msg);
}
