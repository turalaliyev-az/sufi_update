

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ==================== PCA9685 Ayarları ====================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVO_MIN  150  // 0° için PWM değeri
#define SERVO_MAX  600  // 180° için PWM değeri

// ==================== Servo Kontrol Sistemi ====================
#define SERVO_COUNT 10  // Toplam servo sayısı

struct Servo {
  uint8_t channel;
  uint16_t currentAngle;
  uint16_t targetAngle;
  uint16_t startAngle;
  uint16_t moveSpeed;
  uint16_t minAngle;
  uint16_t maxAngle;
};

Servo servos[SERVO_COUNT];

// Servo Indexleri
enum {
  SAG_KUREK, SAG_CIYIN, SAG_DIRSEK, SAG_EL, SAG_BASMARMAQ,
  SOL_KUREK, SOL_CIYIN, SOL_DIRSEK, BOYUN, BAS
};

// ==================== Seri Haberleşme Değişkenleri ====================
const uint8_t BUFFER_SIZE = 32;
char serialBuffer[BUFFER_SIZE];
uint8_t bufferIndex = 0;
bool commandReady = false;
char currentCommand = 0;
bool commandProcessed = true;

// ==================== Yardımcı Fonksiyonlar ====================
uint16_t angleToPWM(uint16_t angle) {
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

void writeServo(uint8_t channel, uint16_t angle) {
  uint16_t pulse = angleToPWM(angle);
  pwm.setPWM(channel, 0, pulse);
}

void setupServo(uint8_t index, uint8_t channel, uint16_t startAngle, 
                uint16_t speed = 10, uint16_t minAngle = 0, uint16_t maxAngle = 180) {
  if (index < SERVO_COUNT) {
    servos[index].channel = channel;
    servos[index].currentAngle = startAngle;
    servos[index].targetAngle = startAngle;
    servos[index].startAngle = startAngle;
    servos[index].moveSpeed = speed;
    servos[index].minAngle = minAngle;
    servos[index].maxAngle = maxAngle;
    
    writeServo(channel, startAngle);
    

  }
}

void updateServos() {
  static uint32_t lastUpdateTime = 0;
  uint32_t currentTime = millis();
  
  // 20ms'de bir güncelle (50Hz)
  if (currentTime - lastUpdateTime < 20) {
    return;
  }
  lastUpdateTime = currentTime;
  
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    if (servos[i].currentAngle != servos[i].targetAngle) {
      if (servos[i].currentAngle < servos[i].targetAngle) {
        servos[i].currentAngle++;
      } else {
        servos[i].currentAngle--;
      }
      
      // Açı sınırlarını kontrol et
      servos[i].currentAngle = constrain(servos[i].currentAngle, 
                                       servos[i].minAngle, servos[i].maxAngle);
      
      writeServo(servos[i].channel, servos[i].currentAngle);
    }
  }
}

void setServoAngle(uint8_t servoIndex, uint16_t angle, uint16_t speed = 0) {
  if (servoIndex < SERVO_COUNT) {
    servos[servoIndex].targetAngle = constrain(angle, 
                                             servos[servoIndex].minAngle, 
                                             servos[servoIndex].maxAngle);
    if (speed > 0) {
      servos[servoIndex].moveSpeed = speed;
    }
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
    currentCommand = serialBuffer[0];
    Serial.println(currentCommand);
    commandProcessed = false;
    bufferIndex = 0;
    commandReady = false;
  }
}

// ==================== Bekleme Fonksiyonu ====================
void waitMs(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) {
    updateServos();
    checkSerial();
    if (commandReady) return;
  }
}





// ==================== Hareket Fonksiyonları ====================
void salam() {
  setServoAngle(SAG_DIRSEK, 70);
  setServoAngle(SAG_KUREK, 70);
  waitMs(1000);
  
  setServoAngle(SAG_EL, 70);
  waitMs(1000);
  
  setServoAngle(SAG_EL, 180);
  waitMs(1000);
  
  setServoAngle(SAG_KUREK, 50);
  setServoAngle(SAG_DIRSEK, 90);
  setServoAngle(SAG_EL, 90);
  waitMs(1000);
}

void sagol() {
  setServoAngle(SAG_DIRSEK, 50);
  setServoAngle(SAG_KUREK, 90);
  setServoAngle(SAG_CIYIN, 105);
  waitMs(2000);
  
  for (uint8_t i = 0; i < 2; i++) {
    setServoAngle(SAG_DIRSEK, 90);
    waitMs(2000);
    setServoAngle(SAG_DIRSEK, 50);
    waitMs(2000);
  }
  
  setServoAngle(SAG_KUREK, 50);
  setServoAngle(SAG_CIYIN, 133);
  setServoAngle(SAG_DIRSEK, 90);
  waitMs(1000);
}

void saga() {
  setServoAngle(SAG_CIYIN, 120);
  waitMs(2000);
  setServoAngle(SAG_CIYIN, 133);
}

void sola() {
  setServoAngle(SOL_CIYIN, 100);
  waitMs(3000);
  setServoAngle(SOL_CIYIN, 72);
  waitMs(1000);
}

void beli() {
  setServoAngle(BAS, 170);
  waitMs(500);
  setServoAngle(BAS, 160);
  waitMs(500);
  setServoAngle(BAS, 170);
  waitMs(500);
  setServoAngle(BAS, 160);
  waitMs(500);

}

void xeyr() {
  setServoAngle(BOYUN, 45);
  waitMs(300);
  setServoAngle(BOYUN, 90);
  waitMs(300);
  setServoAngle(BOYUN, 120);
  waitMs(300);
  setServoAngle(BOYUN, 90);
  waitMs(300);
}

void ela() {
  setServoAngle(SAG_DIRSEK, 0);
  setServoAngle(SAG_EL, 160);
  setServoAngle(SAG_BASMARMAQ, 40);
  waitMs(2000);
  
  setServoAngle(SAG_DIRSEK, 90);
  setServoAngle(SAG_EL, 60);
  setServoAngle(SAG_BASMARMAQ, 180);
  waitMs(500);
}

void pis() {
  setServoAngle(SAG_DIRSEK, 0);
  setServoAngle(SAG_EL, 0);
  setServoAngle(SAG_BASMARMAQ, 0);
  waitMs(2000);
  
  setServoAngle(SAG_DIRSEK, 90);
  setServoAngle(SAG_EL, 90);
  setServoAngle(SAG_BASMARMAQ, 70);
  waitMs(1000);
}

void helelik() {
  setServoAngle(SAG_DIRSEK, 0);
  setServoAngle(SAG_KUREK, 96);
  setServoAngle(SAG_CIYIN, 105);
  
  setServoAngle(SOL_DIRSEK, 140);
  setServoAngle(SOL_CIYIN, 118);
  setServoAngle(SOL_KUREK, 100);
  
  waitMs(2000);
  
  for (uint8_t i = 0; i < 2; i++) {
    setServoAngle(SAG_DIRSEK, 90);
    setServoAngle(SOL_DIRSEK, 100);
    waitMs(2000);
    
    setServoAngle(SAG_DIRSEK, 0);
    setServoAngle(SOL_DIRSEK, 140);
    waitMs(2000);
  }
  
  setServoAngle(SAG_KUREK, 50);
  setServoAngle(SAG_CIYIN, 133);
  setServoAngle(SAG_DIRSEK, 90);
  
  setServoAngle(SOL_DIRSEK, 100);
  setServoAngle(SOL_CIYIN, 72);
  setServoAngle(SOL_KUREK, 126);
  waitMs(1000);
}


void reqs() {
  
  setServoAngle(SAG_KUREK, 90, 5);
  setServoAngle(SAG_CIYIN, 105, 5);
  setServoAngle(SOL_CIYIN, 118, 5);
  setServoAngle(SOL_KUREK, 100, 5);
  waitMs(1500);
  
  for (uint8_t i = 0; i < 2; i++) {
    setServoAngle(SAG_DIRSEK, 0, 8);
    setServoAngle(SOL_DIRSEK, 140, 8);
    waitMs(1500);
    
    setServoAngle(SAG_DIRSEK, 92, 8);
    setServoAngle(SOL_DIRSEK, 100, 8);
    waitMs(1500);
  }
  //sagkurek

  setServoAngle(SAG_DIRSEK, 92, 5);
  setServoAngle(SAG_EL, 120, 5);
  waitMs(1000);
  
  setServoAngle(SOL_DIRSEK, 100, 5);
  waitMs(1000);
  
  setServoAngle(SAG_KUREK, 50, 5);
  setServoAngle(SAG_CIYIN, 133, 5);
  setServoAngle(SAG_DIRSEK, 92, 5);
  setServoAngle(SAG_EL, 90, 5);
  
  setServoAngle(SOL_DIRSEK, 100, 5);
  setServoAngle(SOL_CIYIN, 72, 5);
  setServoAngle(SOL_KUREK, 126, 5);
  waitMs(1000);

    for(uint8_t i=0; i<2; i++){
    setServoAngle(SAG_KUREK, 120, 5); 
    waitMs(1500);
    setServoAngle(SAG_KUREK, 50, 5);
    waitMs(1500);
    setServoAngle(SOL_KUREK, 70, 5);
    waitMs(1500);
    setServoAngle(SOL_KUREK, 126, 5);
    waitMs(1500);
  }

  waitMs(1000);
  setServoAngle(SAG_KUREK, 50, 5);
  setServoAngle(SAG_CIYIN, 133, 5);
  setServoAngle(SAG_DIRSEK, 92, 5);
  setServoAngle(SAG_EL, 90, 5);
  
  setServoAngle(SOL_DIRSEK, 100, 5);
  setServoAngle(SOL_CIYIN, 72, 5);
  setServoAngle(SOL_KUREK, 126, 5);
  waitMs(1000);

}
// ==================== Setup ====================
void setup() {
  Serial.begin(115200);
  
  Wire.begin();

  // I2C adres taraması
  bool pwmFound = false;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {

      if (addr == 0x40) {
        pwmFound = true;
      }
    }
  }
  
  if (!pwmFound) {
    while(1);
  }
  

  pwm.begin();
  pwm.setPWMFreq(60);

  // Servoları kur (Index, Kanal, Başlangıç açısı, Hız, Min açı, Max açı)
  
  setupServo(SAG_KUREK, 12, 50, 8, 0, 180);
  setupServo(SAG_CIYIN, 11, 133, 8, 0, 180);
  setupServo(SAG_DIRSEK, 13, 92, 8, 0, 180);
  setupServo(SAG_EL, 15, 90, 5, 0, 180);

  // Sol kol
  setupServo(SOL_KUREK, 6, 126, 8, 0, 180);
  setupServo(SOL_CIYIN, 4, 72, 8, 0, 180);
  setupServo(SOL_DIRSEK, 8, 100, 8, 0, 180);

  // Gövde
  setupServo(BOYUN, 9, 90, 5, 0, 180);
  setupServo(BAS, 10, 160, 5, 0, 180);


  Serial.print(SERVO_COUNT);

  // Başlangıç pozisyonuna getir
  waitMs(1000);
  
}

// ==================== Loop ====================
void loop() {
  checkSerial();
  processCommand();
  
  if (!commandProcessed) {
    switch (currentCommand) {
      case '1': salam(); break;
      case '2': sagol(); break;
      case '3': saga(); break;
      case '4': sola(); break;
      case '5': beli(); break;
      case '6': xeyr(); break;
      case '7': ela(); break;
      case '8': pis(); break;
      case '9': helelik(); break;

      case 'r': reqs(); break; // Yeni komut eklendi
      default: 
        break;
    }
    commandProcessed = true;
    currentCommand = 0;
  }
  
  updateServos();
}