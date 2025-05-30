#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// === OLED Setup ===
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// === PIN SETUP ===
#define trigPinFront 32
#define echoPinFront 33
#define trigPinRight 18
#define echoPinRight 19
#define trigPinLeft  5
#define echoPinLeft  23

#define in1 12
#define in2 14
#define in3 27
#define in4 26
#define ena 13
#define enb 25

#define flamePin 15
#define buzzerPin 4

// === KALMAN FILTER ===
float kalmanGain = 0.6;
float estFront = 0, estLeft = 0, estRight = 0;

float kalmanFilter(float raw, float prevEst) {
  return kalmanGain * raw + (1 - kalmanGain) * prevEst;
}

// === FUZZY FUNCTIONS ===
float trapezoid(float x, float a, float b, float c, float d) {
  if (x <= a || x >= d) return 0;
  else if (x >= b && x <= c) return 1;
  else if (x > a && x < b) return (x - a) / (b - a);
  else return (d - x) / (d - c);
}

float triangle(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0;
  else if (x == b) return 1;
  else if (x > a && x < b) return (x - a) / (b - a);
  else return (c - x) / (c - b);
}

int fuzzySpeed(float dist) {
  float dekat = trapezoid(dist, 0, 0, 10, 20);
  float sedang = triangle(dist, 10, 25, 40);
  float jauh = trapezoid(dist, 30, 40, 80, 100);

  float z1 = 100, z2 = 180, z3 = 255;
  float num = dekat*z1 + sedang*z2 + jauh*z3;
  float den = dekat + sedang + jauh;
  return (den == 0) ? 100 : (int)(num / den);
}

int fuzzyArah(float depan, float kiri, float kanan) {
  float dktDepan = trapezoid(depan, 0, 0, 10, 20);
  float sdgDepan = triangle(depan, 10, 25, 40);
  float jauhDepan = trapezoid(depan, 30, 40, 80, 100);

  float dktKiri = trapezoid(kiri, 0, 0, 10, 20);
  float dktKanan = trapezoid(kanan, 0, 0, 10, 20);

  float z[4], w[4]; int rule = 0;

  w[rule] = dktDepan; z[rule] = 0.0; rule++;
  w[rule] = min(sdgDepan, dktKanan); z[rule] = -1.0; rule++;
  w[rule] = min(sdgDepan, dktKiri); z[rule] = 1.0; rule++;
  w[rule] = jauhDepan; z[rule] = 0.0; rule++;

  float num = 0, den = 0;
  for (int i = 0; i < rule; i++) {
    num += w[i] * z[i];
    den += w[i];
  }

  float output = (den == 0) ? 0 : num / den;
  if (output < -0.5) return -1;
  else if (output > 0.5) return 1;
  else return 0;
}

// === SETUP ===
void setup() {
  Serial.begin(115200);

  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(ena, OUTPUT); pinMode(enb, OUTPUT);
  digitalWrite(ena, HIGH); digitalWrite(enb, HIGH);

  pinMode(trigPinFront, OUTPUT); pinMode(echoPinFront, INPUT);
  pinMode(trigPinRight, OUTPUT); pinMode(echoPinRight, INPUT);
  pinMode(trigPinLeft, OUTPUT); pinMode(echoPinLeft, INPUT);

  pinMode(flamePin, INPUT);
  pinMode(buzzerPin, OUTPUT); digitalWrite(buzzerPin, LOW);

  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED gagal diinisialisasi");
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Robot siap berjalan");
  display.display();
  delay(1000);
}

// === ULTRASONIK ===
long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  long distance = duration * 0.034 / 2;
  return (distance == 0 || distance > 300) ? 300 : distance;
}

// === MOTOR CONTROL ===
void maju() {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
}
void mundur() {
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
}
void belokKiri() {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
}
void belokKanan() {
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
}
void berhenti() {
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}

// === LOOP ===
void loop() {
  long dFront = readUltrasonic(trigPinFront, echoPinFront);
  long dLeft = readUltrasonic(trigPinLeft, echoPinLeft);
  long dRight = readUltrasonic(trigPinRight, echoPinRight);

  estFront = kalmanFilter(dFront, estFront);
  estLeft  = kalmanFilter(dLeft, estLeft);
  estRight = kalmanFilter(dRight, estRight);

  int speed = fuzzySpeed(estFront);
  int arah = fuzzyArah(estFront, estLeft, estRight);

  bool apiTerdeteksi = (digitalRead(flamePin) == LOW);
  digitalWrite(buzzerPin, apiTerdeteksi ? HIGH : LOW);

  Serial.print("Depan: "); Serial.print(estFront);
  Serial.print(" | Kiri: "); Serial.print(estLeft);
  Serial.print(" | Kanan: "); Serial.print(estRight);
  Serial.print(" | Speed: "); Serial.print(speed);
  Serial.print(" | Arah: "); Serial.print(arah);
  Serial.print(" | Api: "); Serial.println(apiTerdeteksi ? "🔥" : "Aman");

  display.clearDisplay();
  display.setCursor(0,0); display.print("Depan: "); display.print(estFront);
  display.setCursor(0,10); display.print("Kiri : "); display.print(estLeft);
  display.setCursor(0,20); display.print("Kanan: "); display.print(estRight);
  display.setCursor(0,30); display.print("Speed: "); display.print(speed);
  display.setCursor(0,40); display.print(apiTerdeteksi ? "🔥 Api Terdeteksi!" : "Aman dari api");
  display.display();

  if (apiTerdeteksi) {
    berhenti();
  } else {
    switch (arah) {
      case -1: belokKiri(); break;
      case 1: belokKanan(); break;
      default: maju(); break;
    }
  }

  delay(100);
}
