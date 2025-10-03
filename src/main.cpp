#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ==================== PCA9685 Ayarları ====================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVO_MIN  150  // 0° için PWM değeri
#define SERVO_MAX  600  // 180° için PWM değeri

// ==================== Servo Kontrol Sistemi ====================
#define SERVO_COUNT 8  // 8 servo

struct Servo {
  uint8_t channel;
  uint16_t currentAngle;
  uint16_t targetAngle;
  uint16_t minAngle;
  uint16_t maxAngle;
};

Servo servos[SERVO_COUNT];

// ==================== Seri Haberleşme Değişkenleri ====================
const uint8_t BUFFER_SIZE = 128;
char serialBuffer[BUFFER_SIZE];
uint8_t bufferIndex = 0;
bool commandReady = false;

// ==================== Yardımcı Fonksiyonlar ====================
uint16_t angleToPWM(uint16_t angle) {
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

void writeServo(uint8_t channel, uint16_t angle) {
  uint16_t pulse = angleToPWM(angle);
  pwm.setPWM(channel, 0, pulse);
}

void setupServo(uint8_t index, uint8_t channel, uint16_t startAngle,
                uint16_t minAngle, uint16_t maxAngle) {
  if (index < SERVO_COUNT) {
    servos[index].channel = channel;
    servos[index].currentAngle = startAngle;
    servos[index].targetAngle = startAngle;
    servos[index].minAngle = minAngle;
    servos[index].maxAngle = maxAngle;

    writeServo(channel, startAngle);
  }
}

void updateServos() {
  static uint32_t lastUpdateTime = 0;
  uint32_t currentTime = millis();

  if (currentTime - lastUpdateTime < 20) return; // 50Hz
  lastUpdateTime = currentTime;

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    if (servos[i].currentAngle != servos[i].targetAngle) {
      if (servos[i].currentAngle < servos[i].targetAngle) {
        servos[i].currentAngle++;
      } else {
        servos[i].currentAngle--;
      }
      // Her servo kendi min/max değerleri içinde
      servos[i].currentAngle = constrain(servos[i].currentAngle,
                                         servos[i].minAngle,
                                         servos[i].maxAngle);
      writeServo(servos[i].channel, servos[i].currentAngle);
    }
  }
}

void setServoAngle(uint8_t servoIndex, int angle) {
  if (servoIndex < SERVO_COUNT) {
    // Min/Max kontrolü
    angle = constrain(angle, servos[servoIndex].minAngle, servos[servoIndex].maxAngle);
    servos[servoIndex].targetAngle = angle;
  }
}

// ==================== Seri Haberleşme Fonksiyonları ====================
void checkSerial() {
  while (Serial.available() > 0 && !commandReady) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufferIndex > 0) {
        serialBuffer[bufferIndex] = '\0';
        commandReady = true;
      }
    } else if (bufferIndex < BUFFER_SIZE - 1) {
      serialBuffer[bufferIndex++] = c;
    }
  }
}

void processCommand() {
  if (commandReady) {
    // Format: "8,120 4,90 6,110 9,90 10,145 13,70 11,120 12,100"
    char *token = strtok(serialBuffer, " ");
    while (token != NULL) {
      int servoIndex, angle;
      if (sscanf(token, "%d,%d", &servoIndex, &angle) == 2) {
        setServoAngle(servoIndex, angle);
      }
      token = strtok(NULL, " ");
    }
    bufferIndex = 0;
    commandReady = false;
  }
}

// ==================== Setup ====================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  pwm.begin();
  pwm.setPWMFreq(60);

  // Servoları kur (Index, Kanal, Başlangıç, Min, Max)
  setupServo(0, 8, 100, 100, 150);   // Servo 0 (kanal 8)
  setupServo(1, 4, 72, 72, 122);     // Servo 1 (kanal 4)
  setupServo(2, 6, 126, 68, 126);    // Servo 2 (kanal 6)
  setupServo(3, 9, 50, 50, 120);     // Servo 3 (kanal 9)
  setupServo(4, 10, 130, 130, 160);  // Servo 4 (kanal 10)
  setupServo(5, 13, 92, 40, 92);     // Servo 5 (kanal 13)
  setupServo(6, 11, 133, 105, 133);  // Servo 6 (kanal 11)
  setupServo(7, 12, 50, 50, 145);    // Servo 7 (kanal 12)

  delay(1000);
}

// ==================== Loop ====================
void loop() {
  checkSerial();
  processCommand();
  updateServos();
}
