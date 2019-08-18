#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <AutoConnect.h>

ESP8266WebServer Server;
AutoConnect Portal(Server);

// kalman variables
float varVolt = 1.12184278324081E-05;  // variance determined using excel and reading samples of raw sensor data
float varProcess = 1e-8;
float Pc = 0.0;
float G = 0.0;
float P = 1.0;
float Xp = 0.0;
float Zp = 0.0;
float Xe = 0.0;


unsigned long previousMillis = 0;
unsigned long wait_previousMillis = 0;
unsigned long refill_previousMillis = 0;
unsigned long waitLedOnTime;
unsigned long glassEmergence;
unsigned long startRefill;

const long refillDelay = 3000;
const long refillInterval = 2550;
//const long refillInterval = 9000;

const int PUMP_PIN = 14;
const int LED_R_PIN = 4;
const int LED_G_PIN = 16;
const int LED_B_PIN = 5;
const int trigPin = 2;
const int echoPin = 0;

int currentMode = 0;
/* 0 - init, 1 - waitingGlass, 2 - refillDelaying,
  3 - refilling, 4 - refillComplete*/

void rootPage() {
  char content[] = "Hello, world";
  Server.send(200, "text/plain", content);
}

void setup() {
  pinMode(PUMP_PIN,  OUTPUT);
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  digitalWrite(PUMP_PIN,  LOW);
  digitalWrite(LED_R_PIN, LOW);
  digitalWrite(LED_G_PIN, LOW);
  digitalWrite(LED_B_PIN, LOW);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(115200);
  Server.on("/", rootPage);
  if (Portal.begin()) {
    Serial.println("WiFi connected: " + WiFi.localIP().toString());
  }
}


int Get_distance() {
  long duration;
  int distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  Serial.print("Distance: ");
  Serial.println(distance);
  return distance;
}

void updateLedState() {
  switch (currentMode) {
    case 1:
      analogWrite(LED_B_PIN, 50);
      digitalWrite(LED_R_PIN, LOW);
      digitalWrite(LED_G_PIN, LOW);
      break;
    case 2:
      digitalWrite(LED_B_PIN, HIGH);
      digitalWrite(LED_R_PIN, LOW);
      digitalWrite(LED_G_PIN, LOW);
      break;
    case 3:
      digitalWrite(LED_B_PIN, LOW);
      digitalWrite(LED_R_PIN, HIGH);
      digitalWrite(LED_G_PIN, LOW);
      break;
    case 4:
      digitalWrite(LED_B_PIN, LOW);
      digitalWrite(LED_R_PIN, LOW);
      digitalWrite(LED_G_PIN, HIGH);
      break;
    default:
      digitalWrite(LED_B_PIN, LOW);
      digitalWrite(LED_R_PIN, LOW);
      digitalWrite(LED_G_PIN, LOW);
      break;
  }
}

void loop() {
  int distance;
  distance = Get_distance();

  // kalman process
  Pc = P + varProcess;
  G = Pc / (Pc + varVolt);  // kalman gain
  P = (1 - G) * Pc;
  Xp = Xe;
  Zp = Xp;
  Xe = G * (distance - Zp) + Xp; // the kalman estimate of the sensor voltage  // kalman process
  distance = Xe;

  unsigned long currentMillis = millis();

  if (distance >= 10 ) {
    currentMode = 1;
    glassEmergence = 4294967295;
    digitalWrite(PUMP_PIN, LOW);
  }

  if (distance < 10 ) {
    if (currentMode == 1) {
      glassEmergence = currentMillis;
      currentMode = 2;
    }
    if ((currentMode == 2) & (currentMillis - glassEmergence >= refillDelay)) {
      startRefill = currentMillis;
      digitalWrite(PUMP_PIN, HIGH);
      currentMode = 3;
    }
    if ((currentMode == 3) & (currentMillis - startRefill  >= refillInterval)) {
      digitalWrite(PUMP_PIN, LOW);
      currentMode = 4;
    }

  }
  Serial.print("Mode: ");
  Serial.println(currentMode);

  updateLedState();
  Portal.handleClient();


  //delay(1000);

}
