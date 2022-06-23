/************************* Inclusão das Bibliotecas *********************************/
#include "ESP8266WiFi.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "Servo.h"
#include "notes.h"

/************************* Conexão WiFi*********************************/

#define WIFI_SSID       "YOUR WIFI SSID" // nome de sua rede wifi
#define WIFI_PASS       "YOUR WIFI PASSWORD"     // senha de sua rede wifi

/********************* Credenciais Adafruit io *************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "YOUR ADAFRUIT USERNAME" // Seu usuario cadastrado na plataforma da Adafruit
#define AIO_KEY         "YOUR ADAFRUIT KEY"       // Sua key da dashboard

/********************** Variaveis globais *******************************/

//* DEFINIÇÃO DO SERVO MOTOR
Servo myservo;

WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

#define ledAmarelo D2
#define ledVerde D3
#define ledVermelho D4
#define buzzerPin D6
#define servoPin D7
#define sensorPir D8
#define sensorWater A0

long previousMillis = 0;

/*
 * SE O VALOR FOR 0 O PIR APENAS INFORMA A PRESENÇA
 * SE O VALOR FOR 1 O PIR CONTROLA O SERVO
 */
bool isPirController = 0;
bool isDetectedOld = 1;
int delayFeedTime = 2000;
/****************************** Declaração dos Feeds ***************************************/

/* feed responsavel por receber os dados da nossa dashboard */
Adafruit_MQTT_Subscribe _feed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/cat-feeder", MQTT_QOS_1);

/* feed responsavel por receber os dados da nossa dashboard */
Adafruit_MQTT_Subscribe _music = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/music", MQTT_QOS_1);

/* feed responsavel por receber os dados da nossa dashboard */
Adafruit_MQTT_Subscribe _controller = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/controller", MQTT_QOS_1);

/* feed responsavel por receber os dados da nossa dashboard */
Adafruit_MQTT_Subscribe _duration = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/duration", MQTT_QOS_1);

/* feed responsavel por enviar os dados do sensor para nossa dashboard */
Adafruit_MQTT_Publish _pirSensor = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/sensor", MQTT_QOS_1);

/* feed responsavel por enviar os dados do sensor para nossa dashboard */
Adafruit_MQTT_Publish _waterLevel = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/water", MQTT_QOS_1);


/*************************** Declaração dos Prototypes ************************************/

void initSerial();
void initPins();
void initWiFi();
void initMQTT();
void girarMotor();
void scanSensorPir();
void conectar_broker();
void playMusic();

/*************************** Laço de conexão MQTT ************************************/

void MQTT_connect();

/*************************** Sketch ************************************/

void setup() {
  initSerial();
  initPins();
  initWiFi();
  initMQTT();
}

uint32_t x = 0;

void loop() {
  MQTT_connect();

  bool sensorOutput = digitalRead(sensorPir);
  if (sensorOutput)
  {
     Serial.println("PRESENÇA");
     if(isDetectedOld == 0) {
      if (! _pirSensor.publish(1)) {
        Serial.println(F("Falha ao enviar o valor do sensor."));
      }
      isDetectedOld = 1;
     }
    if(isPirController == 1){
      girarMotor();
    }
    
  }
  else {
    if(isDetectedOld == 1) {
      if (! _pirSensor.publish(0)) {
        Serial.println(F("Falha ao enviar o valor do sensor."));
        delay(1000);
      }
      isDetectedOld = 0;
    }
  }

  int waterLevel = analogRead(sensorWater);
  Serial.println(waterLevel);
  if (! _waterLevel.publish(waterLevel)) {
      Serial.println(F("Falha ao enviar o valor do sensor de agua."));
      delay(1000);
  }
  
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1500))) {
    if (subscription == &_feed) {
      Serial.println("GIRAR MOTOR");
      girarMotor();
    } else if (subscription == &_music) {
      Serial.println("TOCAR MUSICA");
      playMusic();
    } 
    else if (subscription == &_controller) {
      Serial.print("MUDANÇA CONTROLE - ");
      if (strcmp((char *)_controller.lastread, "PIR") == 0) { 
      Serial.println("CONTROLE PIR");
        isPirController = 1;                        
      }
      if (strcmp((char *)_controller.lastread, "APP") == 0) { 
      Serial.println("CONTROLE APP");
        isPirController = 0;                           
      }
    } else if(subscription == &_duration) {
      Serial.print((char *)_controller.lastread);
    }
  }

  if (! mqtt.ping()) {
    mqtt.disconnect();
  }
}

/*************************** Implementação dos Prototypes ************************************/

/* Conexao Serial */
void initSerial() {
  Serial.begin(9600);
  delay(10);
}

/* Configuração dos pinos */
void initPins() {
  pinMode(sensorPir, INPUT);
  myservo.attach(servoPin);
  delay(200);
  pinMode(ledVerde, OUTPUT);
  pinMode(ledAmarelo, OUTPUT);
  pinMode(ledVermelho, OUTPUT);
}

/* Configuração da conexão WiFi */
void initWiFi() {
  Serial.print("Conectando-se na rede "); Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long startTime = millis();
  digitalWrite(ledVermelho, LOW);
  digitalWrite(ledVerde, LOW);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(ledAmarelo, !digitalRead(ledAmarelo));
    if (millis() - startTime > 10000)
      break;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("Conectado à rede com sucesso");
    digitalWrite(ledAmarelo, LOW);
    digitalWrite(ledVermelho, LOW);
    digitalWrite(ledVerde, HIGH);
    Serial.println("Endereço IP: "); Serial.println(WiFi.localIP());
  } else {
    digitalWrite(ledAmarelo, LOW);
    digitalWrite(ledVerde, LOW);
    digitalWrite(ledVermelho, HIGH);
    Serial.println("Não foi possível conectar ao WIFI escolhido");
  }
}

/* Configuração da conexão MQTT */
void initMQTT() {
  //_feed.setCallback(girarMotor);
  //_music.setCallback(playMusic);
  mqtt.subscribe(&_feed);
  mqtt.subscribe(&_music);
  mqtt.subscribe(&_controller);
  mqtt.subscribe(&_duration);
}

/*************************** Implementação dos Callbacks ************************************/

void girarMotor()
{
  delay(500);
  digitalWrite(ledAmarelo, HIGH);
  tone(buzzerPin, 1500);
  delay(500);
  int pos;
  for (pos = 0; pos <= 180; pos += 25)
  { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);
    delay(5);            // waits 15ms for the servo to reach the position
  }
  noTone(buzzerPin);
  delay(500);
  digitalWrite(ledAmarelo, LOW);
  delay(delayFeedTime);
  digitalWrite(ledAmarelo, HIGH);
  tone(buzzerPin, 1500);
  delay(500);
  for (pos = 180; pos >= 0; pos -= 20)
  { // goes from 180 degrees to 0 degrees
    myservo.write(pos);
    delay(15);
  }
  noTone(buzzerPin);
  delay(500);
  digitalWrite(ledAmarelo, LOW);
  delay(500);
  delay(1000);
}

//char *data, uint16_t len

void playMusic() {
  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {

    // calculates the duration of each note
    divider = melody[thisNote + 1];
    if (divider > 0) {
      // regular note, just proceed
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      // dotted notes are represented with negative durations!!
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; // increases the duration in half for dotted notes
    }

    // we only play the note for 90% of the duration, leaving 10% as a pause
    tone(buzzerPin, melody[thisNote], noteDuration * 0.9);

    // Wait for the specief duration before playing the next note.
    delay(noteDuration);

    // stop the waveform generation before the next note.
    noTone(buzzerPin);
  }
}

void scanSensorPir() {
  bool sensorOutput = digitalRead(sensorPir);
  if (sensorOutput)
  {
    Serial.println("DETECTADO PRESENÇA");
    if (! _pirSensor.publish(1)) {
      Serial.println("Falha ao enviar o valor do sensor.");
    }
  }
  else {
    Serial.println("NÃO DETECTADO PRESENÇA");
    if (! _pirSensor.publish(0)) {
      Serial.println("Falha ao enviar o valor do sensor.");
    }
  }
}

/*************************** Demais implementações ************************************/

void MQTT_connect() {
  int8_t ret;

  if (mqtt.connected()) {
    return;
  }

  Serial.print("Conectando... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Reconectando em 5s...");
    mqtt.disconnect();
    delay(5000);
    retries--;
    if (retries == 0) {
      while (1);
    }
  }
  Serial.println("Conectado!");
}

/* Conexão com o broker e também servirá para reestabelecer a conexão caso caia */
void conectar_broker() {
  int8_t ret;

  if (mqtt.connected()) {
    return;
  }

  Serial.println("Conectando-se ao broker mqtt...");

  uint8_t num_tentativas = 3;
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Falha ao se conectar. Tentando se reconectar em 5 segundos.");
    mqtt.disconnect();
    delay(5000);
    num_tentativas--;
    if (num_tentativas == 0) {
      Serial.println("Seu ESP será resetado.");
      while (1);
    }
  }

  Serial.println("Conectado ao broker com sucesso.");
}
