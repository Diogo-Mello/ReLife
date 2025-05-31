#include <BluetoothSerial.h>

// --- Objeto de comunicação Bluetooth ---
BluetoothSerial SerialBT;

// --- Pinos ---
const int sensorPin    = 34; // Sensor de respiração (entrada analógica)
const int ledVerde     = 27;
const int ledAmarelo   = 23;
const int ledVermelho  = 22;
const int ledInspira   = 14;
const int ledExpira    = 12;

// --- Constantes e variáveis de lógica ---
const int limiarPico = 1000; // Ajuste conforme necessário
int contagemRespiracoes = 0;
bool emRespiracao = false;

const unsigned long duracaoJanela = 4000; // Janela de 4 segundos
unsigned long tempoInicioJanela = 0;

unsigned long tempoUltimaRespiracao = 0;
const unsigned long tempoRefratario = 1000; // 1 segundo de intervalo mínimo entre respirações

// LEDs de inspiração e expiração
unsigned long tempoRespiracao = 0;
bool inspirando = true;

// Controle de envio via Bluetooth
unsigned long ultimoEnvioBluetooth = 0;
const unsigned long intervaloEnvio = 200; // a cada 200ms

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("=== Iniciando ESP32 – Monitor de Respiração ===");

  if (!SerialBT.begin("ESP32-Saude")) {
    Serial.println("Erro ao iniciar Bluetooth");
  } else {
    Serial.println("Bluetooth iniciado: ESP32-Saude");
  }

  pinMode(ledVerde, OUTPUT);
  pinMode(ledAmarelo, OUTPUT);
  pinMode(ledVermelho, OUTPUT);
  pinMode(ledInspira, OUTPUT);
  pinMode(ledExpira, OUTPUT);

  digitalWrite(ledVerde, LOW);
  digitalWrite(ledAmarelo, LOW);
  digitalWrite(ledVermelho, LOW);
  digitalWrite(ledInspira, LOW);
  digitalWrite(ledExpira, LOW);

  tempoInicioJanela = millis();
  tempoRespiracao = millis();
}

void loop() {
  unsigned long agora = millis();

  // --- LEITURA DO SENSOR ---
  int leitura = analogRead(sensorPin);

  // Envia os dados em JSON por Bluetooth
  enviarDadosBluetooth(leitura);

  // Detecção de respiração
  if (leitura > limiarPico && !emRespiracao && (agora - tempoUltimaRespiracao > tempoRefratario)) {
  contagemRespiracoes++;
  emRespiracao = true;
  tempoUltimaRespiracao = agora;
  Serial.println(">> Respiração detectada!");
}
 else if (leitura <= limiarPico - 200 && emRespiracao) {
    emRespiracao = false;
  }

  // --- FIM DA JANELA DE 4 SEGUNDOS ---
  if (agora - tempoInicioJanela >= duracaoJanela) {
    Serial.print("Janela de 4s finalizada – Respirações detectadas: ");
    Serial.println(contagemRespiracoes);

    if (contagemRespiracoes == 0 || contagemRespiracoes == 1) {
      digitalWrite(ledVerde, HIGH);
      digitalWrite(ledAmarelo, LOW);
      digitalWrite(ledVermelho, LOW);
      SerialBT.println(">>> LED VERDE: Respiração calma ou ausente");
    } else if (contagemRespiracoes == 2) {
      digitalWrite(ledVerde, LOW);
      digitalWrite(ledAmarelo, HIGH);
      digitalWrite(ledVermelho, LOW);
      SerialBT.println(">>> LED AMARELO: Respiração moderada");
    } else {
      digitalWrite(ledVerde, LOW);
      digitalWrite(ledAmarelo, LOW);
      digitalWrite(ledVermelho, HIGH);
      SerialBT.println(">>> LED VERMELHO: Respiração acelerada");
    }

    contagemRespiracoes = 0;
    tempoInicioJanela = agora;
    Serial.println("---- Nova janela de 4s iniciada ----");
  }

  // --- GUIA DE RESPIRAÇÃO ---
  if (inspirando) {
    digitalWrite(ledInspira, HIGH);
    digitalWrite(ledExpira, LOW);
    if (agora - tempoRespiracao >= 4000UL) {
      inspirando = false;
      tempoRespiracao = agora;
    }
  } else {
    digitalWrite(ledInspira, LOW);
    digitalWrite(ledExpira, HIGH);
    if (agora - tempoRespiracao >= 4000UL) {
      inspirando = true;
      tempoRespiracao = agora;
    }
  }

  delay(50); // Suaviza leitura
}

// --- FUNÇÃO PARA ENVIAR JSON VIA BLUETOOTH ---
void enviarDadosBluetooth(int leitura) {
  unsigned long agora = millis();
  if (agora - ultimoEnvioBluetooth >= intervaloEnvio) {
    String json = "{";
    json += "\"sensor\":";
    json += leitura;
    json += ",\"respiracoes\":";
    json += contagemRespiracoes;
    json += "}";

    SerialBT.println(json);
    ultimoEnvioBluetooth = agora;
  }
}
