/*
 * Robot Navigasi Cerdas dengan Fuzzy Logic
 * -----------------------------------------
 * Menggunakan sensor ultrasonik untuk mendeteksi obstacle
 * Sensor api untuk deteksi kebakaran
 * Sistem kendali fuzzy untuk navigasi halus
 * 
 * By: Wahyunugroho99 (Improved version)
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// === KONFIGURASI OLED ===
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// === PIN KONFIGURASI ===
// Sensor ultrasonik
#define TRIG_FRONT 32
#define ECHO_FRONT 33
#define TRIG_RIGHT 18
#define ECHO_RIGHT 19
#define TRIG_LEFT  5
#define ECHO_LEFT  23

// Motor driver L298N
#define MOTOR_RIGHT_FWD 12  // in1
#define MOTOR_RIGHT_BWD 14  // in2
#define MOTOR_LEFT_FWD  27  // in3
#define MOTOR_LEFT_BWD  26  // in4
#define MOTOR_RIGHT_EN  13  // ena
#define MOTOR_LEFT_EN   25  // enb

// Sensor api dan buzzer
#define FLAME_PIN 15
#define BUZZER_PIN 4

// === KONSTANTA SISTEM ===
#define MAX_DISTANCE 300          // Batas maksimum pembacaan sensor (cm)
#define OBSTACLE_CRITICAL 10      // Jarak kritis untuk hambatan (cm)
#define SCAN_DELAY 100            // Delay antar scan (ms)
#define MAX_SPEED 255             // Kecepatan PWM maksimum
#define NUM_READINGS 5            // Jumlah pembacaan untuk rata-rata bergerak

// === VARIABEL GLOBAL ===
// Status robot
enum RobotState {
  STATE_NORMAL,
  STATE_OBSTACLE_DETECTED,
  STATE_FLAME_DETECTED,
  STATE_MANEUVERING
};
RobotState currentState = STATE_NORMAL;

// Data sensor
struct SensorData {
  float front;
  float left;
  float right;
  bool flameDetected;
};
SensorData sensorData;

// Konfigurasi arah robot
enum TurnDirection {
  TURN_NONE = 0,
  TURN_LEFT = -1,
  TURN_RIGHT = 1,
  TURN_BACK = 2
};

// Struktur data untuk hasil fuzzy jarak
struct FuzzyDistance {
  float sangat_dekat;  // Kategori baru untuk jarak sangat dekat
  float dekat;
  float sedang;
  float jauh;
  float sangat_jauh;  // Kategori baru untuk jarak sangat jauh
};

// Struktur data untuk output fuzzy
struct MotorCommands {
  int leftSpeed;
  int rightSpeed;
  TurnDirection turnDirection;
};

// Deklarasi variabel global untuk motor commands
MotorCommands motorCommands;

// Buffer untuk filter rata-rata bergerak
float frontReadings[NUM_READINGS] = {0};
float leftReadings[NUM_READINGS] = {0};
float rightReadings[NUM_READINGS] = {0};
int readIndex = 0;

// Buffer untuk menampilkan teks debugging
char debugBuffer[64];

// === FILTER SENSOR ===
// Implementasi Exponential Moving Average (EMA) - lebih responsif daripada Kalman sederhana
float emaAlpha = 0.3;  // Faktor smoothing (0.0-1.0) - semakin besar semakin responsif
float frontFiltered = 0, leftFiltered = 0, rightFiltered = 0;

// Filter EMA untuk data sensor
float emaFilter(float newValue, float prevFiltered) {
  return emaAlpha * newValue + (1.0 - emaAlpha) * prevFiltered;
}

// Filter rata-rata bergerak untuk mengurangi noise
float movingAverage(float newValue, float* readings) {
  // Geser dan simpan nilai baru
  float sum = 0;
  for (int i = NUM_READINGS - 1; i > 0; i--) {
    readings[i] = readings[i-1];
    sum += readings[i];
  }
  readings[0] = newValue;
  sum += newValue;
  
  return sum / NUM_READINGS;
}

// === FUZZY MEMBERSHIP FUNCTIONS ===
// Fungsi keanggotaan trapesium yang disempurnakan - menangani semua kasus
float trapezoid(float x, float a, float b, float c, float d) {
  if (x <= a || x >= d) return 0.0;
  else if (x >= b && x <= c) return 1.0;
  else if (x > a && x < b) return (x - a) / (b - a);
  else return (d - x) / (d - c);
}

// Fungsi keanggotaan segitiga yang disempurnakan
float triangle(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0;
  else if (x == b) return 1.0;
  else if (x > a && x < b) return (x - a) / (b - a);
  else return (c - x) / (c - b);
}

// Fungsi untuk mencari nilai maksimum dari beberapa float
float maxOf(float a, float b, float c) {
  float max = a;
  if (b > max) max = b;
  if (c > max) max = c;
  return max;
}

float minOf(float a, float b) {
  return (a < b) ? a : b;
}

// === FUZZY LOGIC - ENHANCED ===
// Fuzzifikasi jarak dengan 5 kategori - lebih detail daripada sebelumnya
FuzzyDistance fuzzyJarak(float jarak) {
  FuzzyDistance hasil;
  hasil.sangat_dekat = trapezoid(jarak, 0, 0, 5, 10);
  hasil.dekat = triangle(jarak, 5, 15, 25);
  hasil.sedang = triangle(jarak, 15, 30, 45);
  hasil.jauh = triangle(jarak, 35, 50, 75);
  hasil.sangat_jauh = trapezoid(jarak, 60, 80, 200, 300);
  return hasil;
}

// Kategori kecepatan motor dengan 5 level
const float SPEED_VERY_SLOW = 60;   // Sangat lambat
const float SPEED_SLOW = 120;       // Lambat
const float SPEED_MEDIUM = 170;     // Sedang
const float SPEED_FAST = 220;       // Cepat
const float SPEED_VERY_FAST = 255;  // Sangat cepat

// Fungsi inferensi dan defuzzifikasi fuzzy yang ditingkatkan
MotorCommands inferensiFuzzy(float front, float left, float right) {
  MotorCommands result;
  
  // Fuzzifikasi jarak sensor
  FuzzyDistance fFront = fuzzyJarak(front);
  FuzzyDistance fLeft = fuzzyJarak(left);
  FuzzyDistance fRight = fuzzyJarak(right);
  
  // === RULE BASE YANG DIPERLUAS ===
  float ruleWeight[10] = {0};   // Bobot untuk setiap rule
  float leftOutput[10] = {0};   // Output kiri untuk tiap rule
  float rightOutput[10] = {0};  // Output kanan untuk tiap rule
  TurnDirection direction[10];  // Arah belok untuk tiap rule
  int ruleCount = 0;            // Penghitung rule yang aktif
  
  // Rule 1: Jika depan sangat dekat → berhenti/mundur darurat
  ruleWeight[ruleCount] = fFront.sangat_dekat;
  leftOutput[ruleCount] = SPEED_VERY_SLOW;
  rightOutput[ruleCount] = SPEED_VERY_SLOW;
  direction[ruleCount] = TURN_BACK;
  ruleCount++;
  
  // Rule 2: Jika depan dekat dan kanan lebih jauh dari kiri → belok kanan tajam
  ruleWeight[ruleCount] = minOf(fFront.dekat, fRight.jauh);
  leftOutput[ruleCount] = SPEED_SLOW;
  rightOutput[ruleCount] = SPEED_VERY_SLOW;
  direction[ruleCount] = TURN_RIGHT;
  ruleCount++;
  
  // Rule 3: Jika depan dekat dan kiri lebih jauh dari kanan → belok kiri tajam
  ruleWeight[ruleCount] = minOf(fFront.dekat, fLeft.jauh);
  leftOutput[ruleCount] = SPEED_VERY_SLOW;
  rightOutput[ruleCount] = SPEED_SLOW;
  direction[ruleCount] = TURN_LEFT;
  ruleCount++;
  
  // Rule 4: Jika depan sedang dan kanan dekat → belok kiri sedang
  ruleWeight[ruleCount] = minOf(fFront.sedang, fRight.dekat);
  leftOutput[ruleCount] = SPEED_MEDIUM;
  rightOutput[ruleCount] = SPEED_FAST;
  direction[ruleCount] = TURN_LEFT;
  ruleCount++;
  
  // Rule 5: Jika depan sedang dan kiri dekat → belok kanan sedang
  ruleWeight[ruleCount] = minOf(fFront.sedang, fLeft.dekat);
  leftOutput[ruleCount] = SPEED_FAST;
  rightOutput[ruleCount] = SPEED_MEDIUM;
  direction[ruleCount] = TURN_RIGHT;
  ruleCount++;
  
  // Rule 6: Jika depan sedang dan kiri/kanan jauh → maju normal
  ruleWeight[ruleCount] = minOf(fFront.sedang, minOf(fLeft.jauh, fRight.jauh));
  leftOutput[ruleCount] = SPEED_MEDIUM;
  rightOutput[ruleCount] = SPEED_MEDIUM;
  direction[ruleCount] = TURN_NONE;
  ruleCount++;
  
  // Rule 7: Jika depan jauh dan kiri/kanan jauh → maju cepat
  ruleWeight[ruleCount] = minOf(fFront.jauh, minOf(fLeft.jauh, fRight.jauh));
  leftOutput[ruleCount] = SPEED_FAST;
  rightOutput[ruleCount] = SPEED_FAST;
  direction[ruleCount] = TURN_NONE;
  ruleCount++;
  
  // Rule 8: Jika depan sangat jauh dan kiri/kanan jauh → maju sangat cepat
  ruleWeight[ruleCount] = minOf(fFront.sangat_jauh, minOf(fLeft.jauh, fRight.jauh));
  leftOutput[ruleCount] = SPEED_VERY_FAST;
  rightOutput[ruleCount] = SPEED_VERY_FAST;
  direction[ruleCount] = TURN_NONE;
  ruleCount++;
  
  // Rule 9: Jika kanan sangat dekat → belok kiri tajam
  ruleWeight[ruleCount] = fRight.sangat_dekat;
  leftOutput[ruleCount] = SPEED_FAST;
  rightOutput[ruleCount] = SPEED_VERY_SLOW;
  direction[ruleCount] = TURN_LEFT;
  ruleCount++;
  
  // Rule 10: Jika kiri sangat dekat → belok kanan tajam
  ruleWeight[ruleCount] = fLeft.sangat_dekat;
  leftOutput[ruleCount] = SPEED_VERY_SLOW;
  rightOutput[ruleCount] = SPEED_FAST;
  direction[ruleCount] = TURN_RIGHT;
  ruleCount++;
  
  // === DEFUZZIFIKASI - WEIGHTED AVERAGE ===
  float totalWeight = 0.0;
  float leftSum = 0.0;
  float rightSum = 0.0;
  TurnDirection finalDirection = TURN_NONE;
  float maxDirectionWeight = 0.0;
  
  for (int i = 0; i < ruleCount; i++) {
    if (ruleWeight[i] > 0) {
      leftSum += leftOutput[i] * ruleWeight[i];
      rightSum += rightOutput[i] * ruleWeight[i];
      totalWeight += ruleWeight[i];
      
      // Ambil arah yang memiliki bobot tertinggi
      if (ruleWeight[i] > maxDirectionWeight) {
        maxDirectionWeight = ruleWeight[i];
        finalDirection = direction[i];
      }
    }
  }
  
  // Hindari pembagian dengan nol
  if (totalWeight == 0) {
    result.leftSpeed = SPEED_MEDIUM;
    result.rightSpeed = SPEED_MEDIUM;
    result.turnDirection = TURN_NONE;
  } else {
    // Hasil akhir defuzzifikasi
    result.leftSpeed = constrain((int)(leftSum / totalWeight), 0, MAX_SPEED);
    result.rightSpeed = constrain((int)(rightSum / totalWeight), 0, MAX_SPEED);
    result.turnDirection = finalDirection;
  }
  
  return result;
}

// === FUNGSI BACA SENSOR ===
// Baca ultrasonik dengan timeout dan validasi 
long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Gunakan timeout untuk mencegah program hang
  long duration = pulseIn(echoPin, HIGH, 30000);
  long distance = duration * 0.034 / 2;
  
  // Validasi jarak
  if (distance == 0 || distance > MAX_DISTANCE) {
    distance = MAX_DISTANCE;  // Default ke jarak maksimum jika tidak valid
  }
  
  return distance;
}

// === MOTOR CONTROL ===
// Kontrol motor dengan banyak mode gerakan
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  analogWrite(MOTOR_LEFT_EN, leftSpeed);
  analogWrite(MOTOR_RIGHT_EN, rightSpeed);
}

void moveForward() {
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
}

void moveBackward() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, HIGH);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, HIGH);
}

void turnLeft() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, HIGH);
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
}

void turnRight() {
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, HIGH);
}

void stopMotors() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  setMotorSpeeds(0, 0);
}

// Fungsi maneuver yang lebih canggih 
void applyMotorCommands(MotorCommands commands) {
  switch (commands.turnDirection) {
    case TURN_NONE:
      moveForward();
      break;
    case TURN_LEFT:
      turnLeft();
      break;
    case TURN_RIGHT:
      turnRight();
      break;
    case TURN_BACK:
      moveBackward();
      break;
  }
  
  setMotorSpeeds(commands.leftSpeed, commands.rightSpeed);
}

// Baca dan proses semua sensor
void updateSensors() {
  // Baca sensor ultrasonik dan filter nilai
  long rawFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long rawLeft = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
  long rawRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
  
  // Terapkan moving average dan EMA filter secara berurutan
  float frontAvg = movingAverage(rawFront, frontReadings);
  float leftAvg = movingAverage(rawLeft, leftReadings);
  float rightAvg = movingAverage(rawRight, rightReadings);
  
  frontFiltered = emaFilter(frontAvg, frontFiltered);
  leftFiltered = emaFilter(leftAvg, leftFiltered);
  rightFiltered = emaFilter(rightAvg, rightFiltered);
  
  // Update data sensor global
  sensorData.front = frontFiltered;
  sensorData.left = leftFiltered;
  sensorData.right = rightFiltered;
  sensorData.flameDetected = (digitalRead(FLAME_PIN) == LOW);
  
  // Update status robot
  if (sensorData.flameDetected) {
    currentState = STATE_FLAME_DETECTED;
  } else if (sensorData.front < OBSTACLE_CRITICAL) {
    currentState = STATE_OBSTACLE_DETECTED;
  } else {
    currentState = STATE_NORMAL;
  }
  
  // Aktifkan buzzer jika api terdeteksi
  digitalWrite(BUZZER_PIN, sensorData.flameDetected ? HIGH : LOW);
}

// === DISPLAY FUNCTIONS ===
// Fungsi UI yang lebih bagus untuk OLED
void updateDisplay() {
  display.clearDisplay();
  
  // Status bar di atas
  display.drawRect(0, 0, SCREEN_WIDTH, 10, SSD1306_WHITE);
  display.setCursor(3, 2);
  
  // Tampilkan state robot
  switch (currentState) {
    case STATE_NORMAL:
      display.print("NORMAL"); 
      break;
    case STATE_OBSTACLE_DETECTED:
      display.print("OBSTACLE!"); 
      break;
    case STATE_FLAME_DETECTED:
      display.print("FIRE DETECTED!");
      break;
    case STATE_MANEUVERING:
      display.print("MANEUVERING");
      break;
  }
  
  // Tampilkan data sensor
  display.setCursor(0, 12);
  display.print("F:"); display.print(sensorData.front, 1);
  display.print(" L:"); display.print(sensorData.left, 1);
  display.print(" R:"); display.print(sensorData.right, 1);
  
  // Visualisasi posisi hambatan
  int centerX = SCREEN_WIDTH/2;
  int centerY = 35;
  int robotRadius = 5;
  
  display.drawCircle(centerX, centerY, robotRadius, SSD1306_WHITE);  // Robot
  
  // Gambar hambatan relatif terhadap robot
  // (skala dari cm ke pixel display)
  float scale = 0.3;  // Skala sensor ke pixel
  int frontX = centerX;
  int frontY = centerY - min((int)(sensorData.front * scale), 25);
  int leftX = centerX - min((int)(sensorData.left * scale), 25);
  int leftY = centerY;
  int rightX = centerX + min((int)(sensorData.right * scale), 25);
  int rightY = centerY;
  
  display.drawLine(centerX, centerY, frontX, frontY, SSD1306_WHITE);  // Garis ke hambatan depan
  display.drawLine(centerX, centerY, leftX, leftY, SSD1306_WHITE);    // Garis ke hambatan kiri
  display.drawLine(centerX, centerY, rightX, rightY, SSD1306_WHITE);  // Garis ke hambatan kanan
  
  // Info motor
  display.setCursor(0, 54);
  sprintf(debugBuffer, "L:%d R:%d %s", 
    motorCommands.leftSpeed, 
    motorCommands.rightSpeed,
    sensorData.flameDetected ? "FIRE!" : "");
  display.print(debugBuffer);
  
  display.display();
}

// === DEBUG OUTPUT ===
void printDebug() {
  Serial.print("F:"); Serial.print(sensorData.front);
  Serial.print(" L:"); Serial.print(sensorData.left);
  Serial.print(" R:"); Serial.print(sensorData.right);
  Serial.print(" | L-PWM:"); Serial.print(motorCommands.leftSpeed);
  Serial.print(" R-PWM:"); Serial.print(motorCommands.rightSpeed);
  Serial.print(" | State:"); 
  
  switch (currentState) {
    case STATE_NORMAL: Serial.print("NORMAL"); break;
    case STATE_OBSTACLE_DETECTED: Serial.print("OBSTACLE"); break;
    case STATE_FLAME_DETECTED: Serial.print("FIRE"); break;
    case STATE_MANEUVERING: Serial.print("MANEUVER"); break;
  }
  
  Serial.print(" | Dir:");
  switch (motorCommands.turnDirection) {
    case TURN_NONE: Serial.println("FWD"); break;
    case TURN_LEFT: Serial.println("LEFT"); break;
    case TURN_RIGHT: Serial.println("RIGHT"); break;
    case TURN_BACK: Serial.println("BACK"); break;
  }
}

// === Variabel global tambahan ===
unsigned long lastSensorUpdate = 0;

// === SETUP ===
void setup() {
  Serial.begin(115200);
  Serial.println("Robot Navigasi Fuzzy v2.0 - Starting up");
  
  // Konfigurasi pin motor
  pinMode(MOTOR_LEFT_FWD, OUTPUT); 
  pinMode(MOTOR_LEFT_BWD, OUTPUT);
  pinMode(MOTOR_RIGHT_FWD, OUTPUT); 
  pinMode(MOTOR_RIGHT_BWD, OUTPUT);
  pinMode(MOTOR_LEFT_EN, OUTPUT); 
  pinMode(MOTOR_RIGHT_EN, OUTPUT);
  
  // Konfigurasi pin sensor
  pinMode(TRIG_FRONT, OUTPUT); 
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT); 
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); 
  pinMode(ECHO_RIGHT, INPUT);
  
  pinMode(FLAME_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Inisialisasi OLED
  Wire.begin(21, 22);  // SDA, SCL pins
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED initialization failed");
    while(true) delay(10);
  }
  
  // Splash screen
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(15, 10);
  display.println("ROBOT NAVIGASI");
  display.setCursor(20, 25);
  display.println("FUZZY LOGIC");
  display.setCursor(20, 40);
  display.println("SYSTEM v2.0");
  display.display();
  delay(2000);
  
  // Inisialisasi awal motor
  stopMotors();
  
  // Inisialisasi data sensor
  updateSensors();
}

// === LOOP ===
void loop() {
  // Update sensor dengan rate terbatas
  if (millis() - lastSensorUpdate > SCAN_DELAY) {
    updateSensors();
    lastSensorUpdate = millis();
  
    // Proses logika fuzzy
    motorCommands = inferensiFuzzy(sensorData.front, sensorData.left, sensorData.right);
  
    // Jalankan robot 
    if (sensorData.flameDetected) {
      stopMotors();  // Berhenti total jika api terdeteksi
    } else {
      applyMotorCommands(motorCommands);
    }
  
    // Update visualisasi dan debug
    updateDisplay();
    printDebug();
  }
}
