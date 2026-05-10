#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <EEPROM.h>  

// ============================================================
// SPEED RADAR
// 4 buttons (A/B/C/D) + joystick + OLED + buzzer + IR
// ============================================================

// OLED SPI pins
#define OLED_DC    PA8
#define OLED_RST   PB10
#define OLED_CS    PB6
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &SPI, OLED_DC, OLED_RST, OLED_CS);

// Joystick + buttons
#define JOY_X    A0      // PA0
#define JOY_Y    A1      // PA1
#define JOY_SW   PA9     // D8
#define BTN_A    PA10    // D2
#define BTN_B    PB3     // D3
#define BTN_C    PB5     // D4
#define BTN_D    PB4     // D5
#define BTN_F    PB8     // D15 -> reset button

// IR sensors
#define IR1      PC1     // A4
#define IR2      A3      // A3
#define DEFAULT_SENSOR_DISTANCE_CM  20.0f
#define MIN_SENSOR_DISTANCE_CM       5.0f
#define MAX_SENSOR_DISTANCE_CM     100.0f
#define SENSOR_DISTANCE_STEP_CM      1.0f
#define DEFAULT_SPEED_LIMIT_CMS     30.0f
#define MIN_SPEED_LIMIT_CMS          5.0f
#define MAX_SPEED_LIMIT_CMS        300.0f
#define SPEED_LIMIT_STEP_CMS         5.0f
#define MEASURE_TIMEOUT_MS  5000
#define COOLDOWN_MS         1500    // Do not accept a new measurement for 1.5s after a measurement
#define F_LONG_PRESS_MS     1500
#define F_DEBOUNCE_MS       40
// Buzzer
#define BUZZER   PC7     // D9

//LED
#define RED      PC9    // D13
#define GREEN      PC8    // D13
#define BLUE      PC6    // D13

unsigned long lastUpdate = 0;
unsigned long lastBeep = 0;

bool lastBtnC = false;
bool lastBtnB = false;
bool lastBtnA = false;

bool systemOff = false;
bool fWasDown = false;
bool fLongHandled = false;
unsigned long fPressStarted = 0;

int statsView = 0;   // 0 = text statistics, 1 = histogram graph (toggle with A)

int menuSelect = 0;
int currentMenu = -1;
int maxMenu = 3;     // 0=SPEED RADAR, 1=STATISTICS, 2=HARDWARE TEST, 3=SETTINGS

// SPEED RADAR state
unsigned long t1 = 0;        // 1st sensor trigger time
unsigned long t2 = 0;        // 2nd sensor trigger time
float lastSpeed = 0.0;       // last measured speed (cm/s)
bool overLimit = false;      // did the speed exceed the limit?

bool lastIr1 = HIGH;         // previous value for edge detection
bool lastIr2 = HIGH;

int firstSensor = 0;         // 0=none, 1=IR1 first, 2=IR2 first
int radarState = 0;          // 0=IDLE, 1=ARMED, 2=MEASURING, 3=RESULT
unsigned long resultTime = 0; // time captured when entering RESULT (for 4s LED/buzzer cap)
unsigned long sensorFlashUntil = 0; // blue LED flash end time when a sensor is triggered
#define ALARM_DURATION_MS 4000
#define SENSOR_FLASH_MS   80

// EEPROM layout:
//   offset 0..3   : saveNum       (total record count, 0..100)
//   offset 4..7   : saveIndex     (next write position, 0..99)
//   offset 8..407 : records       (100 floats, 4 bytes each)
//   offset 408    : ledSetting    (4 byte int)
//   offset 412    : buzzerSetting (4 byte int)
//   offset 416    : sensorDistanceCm (4 byte float)
//   offset 420    : speedLimitCms     (4 byte float)
//   offset 424    : speedUnitSetting  (4 byte int, 0=cm/s, 1=km/h)
int saveNum = 0;     // total record count (stays fixed after reaching max)
int saveIndex = 0;   // ring buffer write position
const int maxSave = 100;
#define EEPROM_RECORDS_OFFSET 8
#define EEPROM_LED_OFFSET     408
#define EEPROM_BUZZER_OFFSET  412
#define EEPROM_DISTANCE_OFFSET 416
#define EEPROM_SPEED_LIMIT_OFFSET 420
#define EEPROM_SPEED_UNIT_OFFSET  424

int settingSelect = 0;
int currentSetting = 0;
int maxSetting = 5;     // 0=Distance, 1=Limit, 2=Unit, 3=LED, 4=BUZZER, 5=Reset
int ledSetting = 1;
int buzzerSetting = 1;
float sensorDistanceCm = DEFAULT_SENSOR_DISTANCE_CM;
float speedLimitCms = DEFAULT_SPEED_LIMIT_CMS;
int speedUnitSetting = 0;

// EEPROM buffer
void bufWriteFloat(uint32_t addr, float value) {
  uint8_t* p = (uint8_t*)&value;
  for (int i = 0; i < 4; i++) eeprom_buffered_write_byte(addr + i, p[i]);
}
float bufReadFloat(uint32_t addr) {
  float value;
  uint8_t* p = (uint8_t*)&value;
  for (int i = 0; i < 4; i++) p[i] = eeprom_buffered_read_byte(addr + i);
  return value;
}
void bufWriteInt(uint32_t addr, int value) {
  uint8_t* p = (uint8_t*)&value;
  for (int i = 0; i < 4; i++) eeprom_buffered_write_byte(addr + i, p[i]);
}
int bufReadInt(uint32_t addr) {
  int value;
  uint8_t* p = (uint8_t*)&value;
  for (int i = 0; i < 4; i++) p[i] = eeprom_buffered_read_byte(addr + i);
  return value;
}

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

float speedForDisplay(float speedCms) {
  return (speedUnitSetting == 1) ? (speedCms * 0.036f) : speedCms;
}

const char* speedUnitLabel() {
  return (speedUnitSetting == 1) ? "km/h" : "cm/s";
}

int speedDecimals() {
  return (speedUnitSetting == 1) ? 2 : 1;
}

void printSpeed(float speedCms) {
  display.print(speedForDisplay(speedCms), speedDecimals());
  display.print(" ");
  display.print(speedUnitLabel());
}

void drawSlider(int x, int y, int w, int h, float value, float minValue, float maxValue) {
  display.drawRect(x, y, w, h, SH110X_WHITE);
  float ratio = (value - minValue) / (maxValue - minValue);
  if (ratio < 0.0f) ratio = 0.0f;
  if (ratio > 1.0f) ratio = 1.0f;
  int fillW = (int)((w - 2) * ratio);
  if (fillW > 0) display.fillRect(x + 1, y + 1, fillW, h - 2, SH110X_WHITE);
}

void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  if (ledSetting == 1) {
    analogWrite(RED, r);
    analogWrite(GREEN, g);
    analogWrite(BLUE, b);
  } else {
    analogWrite(RED, 0);
    analogWrite(GREEN, 0);
    analogWrite(BLUE, 0);
  }
}

void forceOutputsOff() {
  noTone(BUZZER);
  analogWrite(RED, 0);
  analogWrite(GREEN, 0);
  analogWrite(BLUE, 0);
}

void softResetFromF() {
  Serial.println("[RESET] F short press - resetting board");
  eeprom_buffer_flush();
  forceOutputsOff();
  delay(100);
  NVIC_SystemReset();
}

void enterOffMode() {
  Serial.println("[POWER] F long press - off mode");
  eeprom_buffer_flush();
  forceOutputsOff();

  radarState = 0;
  firstSensor = 0;
  currentMenu = -1;

  display.clearDisplay();
  display.display();
  display.oled_command(SH110X_DISPLAYOFF);
  systemOff = true;
}

void exitOffMode() {
  Serial.println("[POWER] F long press - on mode");
  systemOff = false;
  lastIr1 = digitalRead(IR1);
  lastIr2 = digitalRead(IR2);
  lastUpdate = 0;

  display.oled_command(SH110X_DISPLAYON);
  display.clearDisplay();
  display.display();
  if (buzzerSetting == 1) tone(BUZZER, 1200, 80);
}

void handleFButton() {
  bool fDown = !digitalRead(BTN_F);
  unsigned long now = millis();

  if (fDown && !fWasDown) {
    fWasDown = true;
    fLongHandled = false;
    fPressStarted = now;
  }

  if (fDown && fWasDown && !fLongHandled &&
      (now - fPressStarted >= F_LONG_PRESS_MS)) {
    fLongHandled = true;
    if (systemOff) {
      exitOffMode();
    } else {
      enterOffMode();
    }
  }

  if (!fDown && fWasDown) {
    unsigned long pressDuration = now - fPressStarted;
    bool wasLongPress = fLongHandled || (pressDuration >= F_LONG_PRESS_MS);
    fWasDown = false;

    if (!wasLongPress && pressDuration >= F_DEBOUNCE_MS && !systemOff) {
      softResetFromF();
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("SPEED RADAR STARTING...");

  pinMode(JOY_SW, INPUT_PULLUP);
  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);
  pinMode(BTN_C, INPUT_PULLUP);
  pinMode(BTN_D, INPUT_PULLUP);
  pinMode(BTN_F, INPUT_PULLUP);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  digitalWrite(RED, LOW);
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, LOW);

  // Fill the EEPROM buffer from flash once, then read from RAM
  eeprom_buffer_fill();
  // Load previous counters; sanity checks clear junk values on first boot
  saveNum   = bufReadInt(0);
  saveIndex = bufReadInt(4);
  if (saveNum < 0 || saveNum > maxSave)      saveNum = 0;
  if (saveIndex < 0 || saveIndex >= maxSave) saveIndex = 0;
  // Load settings; boolean settings must be 0 or 1, otherwise use default 1
  ledSetting    = bufReadInt(EEPROM_LED_OFFSET);
  buzzerSetting = bufReadInt(EEPROM_BUZZER_OFFSET);
  sensorDistanceCm = bufReadFloat(EEPROM_DISTANCE_OFFSET);
  speedLimitCms = bufReadFloat(EEPROM_SPEED_LIMIT_OFFSET);
  speedUnitSetting = bufReadInt(EEPROM_SPEED_UNIT_OFFSET);
  if (ledSetting != 0 && ledSetting != 1)       ledSetting = 1;
  if (buzzerSetting != 0 && buzzerSetting != 1) buzzerSetting = 1;
  if (!(sensorDistanceCm >= MIN_SENSOR_DISTANCE_CM && sensorDistanceCm <= MAX_SENSOR_DISTANCE_CM)) {
    sensorDistanceCm = DEFAULT_SENSOR_DISTANCE_CM;
  }
  if (!(speedLimitCms >= MIN_SPEED_LIMIT_CMS && speedLimitCms <= MAX_SPEED_LIMIT_CMS)) {
    speedLimitCms = DEFAULT_SPEED_LIMIT_CMS;
  }
  if (speedUnitSetting != 0 && speedUnitSetting != 1) speedUnitSetting = 0;
  Serial.print("[EEPROM] saveNum="); Serial.print(saveNum);
  Serial.print(" saveIndex="); Serial.print(saveIndex);
  Serial.print(" LED="); Serial.print(ledSetting);
  Serial.print(" BUZZER="); Serial.print(buzzerSetting);
  Serial.print(" DIST="); Serial.print(sensorDistanceCm);
  Serial.print(" LIMIT="); Serial.print(speedLimitCms);
  Serial.print(" UNIT="); Serial.println(speedUnitSetting);

  display.begin(0, true);
  display.setRotation(1);
  display.clearDisplay();
  display.display();

  // Splash frame
  auto drawSplashFrame = []() {
    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);
    display.setTextSize(1);

    display.setCursor(34, 0);
    display.print("SPEED RADAR");

    // "STARTING"
    display.setCursor(34, 10);
    display.print("STARTING");
    int dotCount = (millis() / 400) % 4;
    for (int i = 0; i < dotCount; i++) display.print(".");

    // Radar
    const int cx = 64, cy = 42, r = 18;
    display.drawCircle(cx, cy, r, SH110X_WHITE);
    display.drawCircle(cx, cy, (r * 2) / 3, SH110X_WHITE);
    display.drawCircle(cx, cy, r / 3, SH110X_WHITE);
    display.drawPixel(cx, cy, SH110X_WHITE);

    // Sweep line
    float angle = (millis() % 1500) / 1500.0f * 2.0f * PI;
    int x2 = cx + (int)(r * cos(angle));
    int y2 = cy + (int)(r * sin(angle));
    display.drawLine(cx, cy, x2, y2, SH110X_WHITE);

    // Trail
    for (int i = 1; i <= 3; i++) {
      float ta = angle - (i * 0.18f);
      if (ta < 0) ta += 2.0f * PI;
      int tr = r - i * 4;
      if (tr > 0) {
        int tx = cx + (int)(tr * cos(ta));
        int ty = cy + (int)(tr * sin(ta));
        display.drawPixel(tx, ty, SH110X_WHITE);
      }
    }

    display.display();
  };

  // Play the animation while waiting instead of using delay
  auto splashDelay = [&drawSplashFrame](unsigned long ms) {
    unsigned long start = millis();
    while (millis() - start < ms) {
      drawSplashFrame();
      delay(20);
    }
  };

  // Intro music + radar sweep animation
  tone(BUZZER, 660, 100);  splashDelay(150);
  tone(BUZZER, 660, 100);  splashDelay(150);
  tone(BUZZER, 1318, 100); splashDelay(150);
  tone(BUZZER, 660, 100);  splashDelay(150);
  tone(BUZZER, 660, 100);  splashDelay(150);
  tone(BUZZER, 1174, 100); splashDelay(150);
  tone(BUZZER, 660, 100);  splashDelay(150);
  tone(BUZZER, 660, 100);  splashDelay(150);
  tone(BUZZER, 1046, 100); splashDelay(150);
  tone(BUZZER, 660, 100);  splashDelay(150);
  tone(BUZZER, 660, 100);  splashDelay(150);
  tone(BUZZER, 932, 100);  splashDelay(150);
  tone(BUZZER, 660, 100);  splashDelay(150);
  tone(BUZZER, 660, 100);  splashDelay(150);
  tone(BUZZER, 987, 100);  splashDelay(150);
  tone(BUZZER, 1046, 100); splashDelay(150);
  tone(BUZZER, 660, 100);  splashDelay(150);
  tone(BUZZER, 660, 100);  splashDelay(150);
  tone(BUZZER, 1318, 100); splashDelay(150);
  tone(BUZZER, 660, 100);  splashDelay(150);
  tone(BUZZER, 660, 100);  splashDelay(150);
  tone(BUZZER, 1174, 100); splashDelay(150);
  tone(BUZZER, 660, 100);  splashDelay(150);
  tone(BUZZER, 660, 100);  splashDelay(150);
  tone(BUZZER, 1046, 100); splashDelay(150);
  tone(BUZZER, 660, 100);  splashDelay(150);
  tone(BUZZER, 660, 100);  splashDelay(150);
  tone(BUZZER, 932, 500);  splashDelay(150);

  splashDelay(2000);

}

void loop() {
  handleFButton();
  if (systemOff) {
    forceOutputsOff();
    delay(10);
    return;
  }

  // HOT Path
  bool ir1Now = digitalRead(IR1);
  bool ir2Now = digitalRead(IR2);
  bool ir1Falling = (lastIr1 == HIGH && ir1Now == LOW);  // 1->0 transition
  bool ir2Falling = (lastIr2 == HIGH && ir2Now == LOW);
  lastIr1 = ir1Now;
  lastIr2 = ir2Now;

  // Blue LED: one 80ms flash when an object is detected. The latch prevents a second trigger and resets only after both sensors are really HIGH, with a short stable filter against bouncing.
  static bool blueLatch = false;
  static unsigned long bothHighSince = 0;
  if (ir1Now == HIGH && ir2Now == HIGH) {
    if (bothHighSince == 0) bothHighSince = millis();
    if (millis() - bothHighSince > 150) blueLatch = false;
  } else {
    bothHighSince = 0;
  }
  if ((ir1Now == LOW || ir2Now == LOW) && !blueLatch) {
    sensorFlashUntil = millis() + 80;
    blueLatch = true;
  }

  // Measure only while the speeed radar menu is open
  if (currentMenu == 0) {

    // Helper that completes a measurement
    auto finishMeasurement = [&](const char* dirLabel) {
      t2 = millis();
      unsigned long delta = (t2 > t1) ? (t2 - t1) : 1;
      lastSpeed = sensorDistanceCm / (delta / 1000.0f);
      overLimit = (lastSpeed > speedLimitCms);
      resultTime = millis();
      radarState = 3;       // RESULT: C is required for a new measurement
      Serial.print("[RADAR] DONE "); Serial.print(dirLabel);
      Serial.print(" delta="); Serial.print(delta);
      Serial.print("ms speed="); Serial.println(lastSpeed);

      //Put the new speed into the current write slot in the RAM buffer
      bufWriteFloat(EEPROM_RECORDS_OFFSET + saveIndex * 4, lastSpeed);
      //Advance the ring buffer index (wrap from 99 to 0)
      saveIndex = (saveIndex + 1) % maxSave;
      //Increase the counter (stays fixed after reaching max)
      if (saveNum < maxSave) saveNum++;
      //Write the counter and index into the RAM buffer too
      bufWriteInt(0, saveNum);
      bufWriteInt(4, saveIndex);
      //Flash transfer (eeprom_buffer_flush) is done when exiting the menu
    };

    // IR1 falling edge
    if (ir1Falling) {
      if (radarState == 1) {
        // ARMED: IR1 triggered as the first sensor
        t1 = millis();
        firstSensor = 1;
        radarState = 2;
        Serial.print("[RADAR] IR1 first t1="); Serial.println(t1);
      } else if (radarState == 2 && firstSensor == 2) {
        // MEASURING: IR2 fell first, IR1 is second; finish the measurement
        finishMeasurement("IR2->IR1");
      }
      // Ignore IR1 in other states (IDLE/RESULT or the same sensor again)
    }

    // IR2 falling edge
    if (ir2Falling) {
      if (radarState == 1) {
        // ARMED: IR2 triggered as the first sensor
        t1 = millis();
        firstSensor = 2;
        radarState = 2;
        Serial.print("[RADAR] IR2 first t1="); Serial.println(t1);
      } else if (radarState == 2 && firstSensor == 1) {
        // MEASURING: IR1 fell first, IR2 is second; finish the measurement
        finishMeasurement("IR1->IR2");
      }
      // Ignore IR2 in other states (IDLE/RESULT or the same sensor again)
    }

    // Timeout: the second sensor did not trigger within MEASURE_TIMEOUT_MS
    if (radarState == 2 && (millis() - t1 > MEASURE_TIMEOUT_MS)) {
      Serial.println("[RADAR] TIMEOUT");
      radarState = 0;       // Return to IDLE and wait for C again
      firstSensor = 0;
    }
  }

  // COLD PATH: UI/log every 100ms
  if (millis() - lastUpdate > 100) {
    lastUpdate = millis();

    int   jx    = analogRead(JOY_X);
    int   jy    = analogRead(JOY_Y);
    bool  jsw   = !digitalRead(JOY_SW);
    bool  btnA  = !digitalRead(BTN_A);
    bool  btnB  = !digitalRead(BTN_B);
    bool  btnC  = !digitalRead(BTN_C);
    bool  btnD  = !digitalRead(BTN_D);
    bool  ir1   = digitalRead(IR1);
    bool  ir2   = digitalRead(IR2);

    bool cPressed = btnC && !lastBtnC;
    bool bPressed = btnB && !lastBtnB;
    bool aPressed = btnA && !lastBtnA;

    // A toggles graph/text in the STATISTICS menu
    if (currentMenu == 1 && aPressed) {
      statsView = 1 - statsView;
      if (buzzerSetting == 1) tone(BUZZER, 1500, 50);
    }

    // Enter the selected menu when C is newly pressed
    if (currentMenu == -1 && cPressed) {
      currentMenu = menuSelect;
       cPressed = false;
      if (currentMenu == 0) {
        // Reset the state when entering SPEED RADAR
        radarState = 0;
        firstSensor = 0;
        t1 = 0;
        lastSpeed = 0.0;
        overLimit = false;
      }
      if (buzzerSetting == 1) {
        tone(BUZZER, 1200, 80);
      }
    }
    // C starts a new measurement while inside SPEED RADAR
    else if (currentMenu == 0 && cPressed) {
      if (radarState == 0 || radarState == 3) {
        radarState = 1;       // ARMED: wait for the first sensor
        firstSensor = 0;
        t1 = 0;
        Serial.println("[RADAR] ARMED - waiting for first sensor");
        if (buzzerSetting == 1) {
          tone(BUZZER, 1200, 80);
        }
      }
    }

    // Return to the main menu when B is newly pressed
    if (currentMenu != -1 && bPressed) {
      // Flush pending changes when exiting radar or settings, instead of flushing after every change
      if (currentMenu == 0 || currentMenu == 3) {
        eeprom_buffer_flush();
        Serial.print("[EEPROM] flush (menu="); Serial.print(currentMenu);
        Serial.println(" exit)");
      }
      currentMenu = -1;

      if (buzzerSetting == 1) {
        tone(BUZZER, 800, 80);
      }
    }

    if (jy < 100 && currentMenu == -1 && menuSelect < maxMenu){
      menuSelect = menuSelect + 1;
      delay(150); // Short delay to prevent fast scrolling
    }else if (jy > 1000 && currentMenu == -1 && menuSelect > 0){
      menuSelect = menuSelect - 1;
      delay(150); // Short delay to prevent fast scrolling
    }

    display.clearDisplay();
    display.setCursor(0, 0);

    // =========================
    // MAIN MENU
    // =========================
    if (currentMenu == -1) {

      setRGB(0, 255, 0);
      display.setCursor(24, 0);
      display.println("-- MAIN MENU --");

      display.setCursor(0, 10);
      display.print("SPEED RADAR");
      if (menuSelect == 0) display.println("<");
      else                 display.println("  ");

      display.print("STATISTICS");
      if (menuSelect == 1) display.println("<");
      else                 display.println("  ");

      display.print("HARDWARE TEST");
      if (menuSelect == 2) display.println("<");
      else                 display.println("  ");

      display.print("SETTINGS");
      if (menuSelect == 3) display.println("<");
      else                 display.println("  ");

      display.println();
      display.println("C = Enter");
    }
    else if (currentMenu == 0) {

      display.setCursor(18, 0);
      display.println("-- SPEED RADAR --");
      display.setCursor(0, 10);
      display.print("Status: ");
      if (radarState == 0)      display.println("NOT READY");
      else if (radarState == 1) display.println("WAITING");
      else if (radarState == 2) display.println("MEASURING");
      else                       display.println(overLimit ? "OVER LIMIT!" : "MEASURED");

      display.println();

      display.print("Last speed: ");
      if (lastSpeed > 0) {
        printSpeed(lastSpeed);
        display.println();
      } else {
        display.println("---");
      }

      display.print("Limit:   ");
      printSpeed(speedLimitCms);
      display.println();

      display.println();
      display.print("IR1:"); display.print(ir1);
      display.print(" IR2:"); display.println(ir2);

      display.println();
      display.println("C = Start");
      display.println("B = Back");

      // Speed based color
      bool resultActive = (radarState == 3 &&
                           (millis() - resultTime) < ALARM_DURATION_MS);
      bool sensorFlashActive = (millis() < sensorFlashUntil);
      if (sensorFlashActive) {
        setRGB(0, 0, 255);      // blue: single flash
      } else if (resultActive) {
        if (lastSpeed > speedLimitCms) {
          setRGB(255, 0, 0);    // red
        } else if (lastSpeed >= (speedLimitCms * 2.0f / 3.0f)) {
          setRGB(255, 30, 0);   // orange
        } else {
          setRGB(255, 255, 255); // white
        }
      } else {
        setRGB(0, 0, 0);
      }

      // Buzzer alarm: 4s when the limit is exceeded
      if (resultActive && overLimit) {
        if (buzzerSetting == 1 && (millis() - lastBeep > 500)) {
          tone(BUZZER, 1500, 200);
          lastBeep = millis();
        }
      }
    }
    
    else if (currentMenu == 1) {

      setRGB(255, 255, 255);

      // text and graph views use this data
      // Histogram bins: 6 bins, each 10 cm/s wide
      //   [0-10) [10-20) [20-30) [30-40) [40-50) [50+]
      const int BIN_COUNT = 6;
      int counts[BIN_COUNT] = {0,0,0,0,0,0};
      float totalSpeed = 0, maxSpeed = 0;
      int overLimitCount = 0;
      for (int i = 0; i < saveNum; i++) {
        float h = bufReadFloat(EEPROM_RECORDS_OFFSET + i * 4);
        totalSpeed += h;
        if (h > maxSpeed) maxSpeed = h;
        if (h > speedLimitCms) overLimitCount++;
        int bin = (int)(h / 10.0f);
        if (bin < 0) bin = 0;
        if (bin >= BIN_COUNT) bin = BIN_COUNT - 1;
        counts[bin]++;
      }
      float averageSpeed = (saveNum > 0) ? (totalSpeed / saveNum) : 0.0f;

      if (statsView == 0) {
        // TEXT VIEW
        display.setCursor(18, 0);
        display.println("-- STATISTICS --");
        display.setCursor(0, 10);
        display.print("Total:   "); display.println(saveNum);
        display.print("Avg:     ");
        if (saveNum > 0) { printSpeed(averageSpeed); display.println(); }
        else             { display.println("---"); }
        display.print("Max:     ");
        if (saveNum > 0) { printSpeed(maxSpeed); display.println(); }
        else             { display.println("---"); }
        display.print("Over Limit: "); display.println(overLimitCount);

        display.println();
        display.println("A = Graph B = Back");
      } else {
        // HISTOGRAM VIEW
        display.setCursor(0, 0);
        display.print("-- HISTOGRAM (");
        display.print(saveNum); display.println(") --");

        if (saveNum == 0) {
          display.setCursor(0, 24);
          display.println("No records");
        } else {
          // Highest bin count for bar scaling
          int maxCount = 1;
          for (int i = 0; i < BIN_COUNT; i++) {
            if (counts[i] > maxCount) maxCount = counts[i];
          }

          const int baseline   = 38;   // bar baseline (horizontal line here)
          const int histHeight = 20;   // maximum bar height
          const int binStride  = 21;   // distance between bin centers
          const int barWidth   = 18;

          // Draw bars
          for (int i = 0; i < BIN_COUNT; i++) {
            int barH = (counts[i] * histHeight) / maxCount;
            int x = i * binStride;
            if (barH > 0) {
              display.fillRect(x, baseline - barH, barWidth, barH, SH110X_WHITE);
            }
          }

          // Horizontal baseline
          display.drawLine(0, baseline, 126, baseline, SH110X_WHITE);

          // Limit line, positioned dynamically on the cm/s histogram scale.
          int limitX = (int)((speedLimitCms / 10.0f) * binStride - 2);
          if (limitX < 0) limitX = 0;
          if (limitX > 126) limitX = 126;
          for (int y = baseline - histHeight; y <= baseline; y += 2) {
            display.drawPixel(limitX, y, SH110X_WHITE);
          }
        }

        // X-axis labels (y=42)
        display.setCursor(0,   42); display.print("0");
        display.setCursor(21,  42); display.print("10");
        display.setCursor(42,  42); display.print("20");
        display.setCursor(63,  42); display.print("30");
        display.setCursor(84,  42); display.print("40");
        display.setCursor(105, 42); display.print("50+");

        // Footer
        display.setCursor(0, 56);
        display.print("A=Text  B=Back");
      }
    }
    // =========================
    // HARDWARE TEST MENU
    // =========================
    else if (currentMenu == 2) {

      setRGB(255, 255, 255);
      display.setCursor(8, 0);
      display.println("-- HARDWARE TEST --");
      display.setCursor(0, 8);
      display.print("JX:");   display.print(jx);
      display.print(" JY:");  display.println(jy);

      display.print("SW:");   display.println(jsw);

      display.print("A:");    display.print(btnA);
      display.print(" B:");   display.print(btnB);
      display.print(" C:");   display.print(btnC);
      display.print(" D:");   display.println(btnD);

      display.print("IR1:");  display.print(ir1);
      display.print(" IR2:"); display.println(ir2);

      display.println();
      display.println("D = Back");
      display.println("SW = Beep");

      Serial.print("JX:"); Serial.print(jx);
      Serial.print(" JY:"); Serial.print(jy);
      Serial.print(" SW:"); Serial.print(jsw);
      Serial.print(" A:"); Serial.print(btnA);
      Serial.print(" B:"); Serial.print(btnB);
      Serial.print(" C:"); Serial.print(btnC);
      Serial.print(" D:"); Serial.print(btnD);
      Serial.print(" IR1:"); Serial.print(ir1);
      Serial.print(" IR2:"); Serial.println(ir2);
    }
    else if (currentMenu == 3) {

      setRGB(0, 255, 0);

      if (jy < 100 && settingSelect < maxSetting){
        settingSelect = settingSelect + 1;
        delay(150);
      }else if (jy > 1000 && settingSelect > 0){
        settingSelect = settingSelect - 1;
        delay(150);
      }

      if (settingSelect == 0 || settingSelect == 1) {
        int dir = 0;
        if (jx < 100) dir = -1;
        else if (jx > 1000) dir = 1;

        if (dir != 0) {
          if (settingSelect == 0) {
            sensorDistanceCm = clampFloat(sensorDistanceCm + dir * SENSOR_DISTANCE_STEP_CM,
                                          MIN_SENSOR_DISTANCE_CM, MAX_SENSOR_DISTANCE_CM);
            bufWriteFloat(EEPROM_DISTANCE_OFFSET, sensorDistanceCm);
          } else {
            speedLimitCms = clampFloat(speedLimitCms + dir * SPEED_LIMIT_STEP_CMS,
                                       MIN_SPEED_LIMIT_CMS, MAX_SPEED_LIMIT_CMS);
            bufWriteFloat(EEPROM_SPEED_LIMIT_OFFSET, speedLimitCms);
          }
          delay(150);
        }
      }

      if (cPressed && settingSelect == 2) {
        speedUnitSetting = 1 - speedUnitSetting;
        bufWriteInt(EEPROM_SPEED_UNIT_OFFSET, speedUnitSetting);
        if (buzzerSetting == 1) tone(BUZZER, 1200, 80);
        delay(150);
      }

      if (cPressed && settingSelect == 3) {
        ledSetting = !ledSetting;
        bufWriteInt(EEPROM_LED_OFFSET, ledSetting);  // Write to RAM, flush on B
        if (buzzerSetting == 1) tone(BUZZER, 1200, 80);
        delay(150);
      }

      if (cPressed && settingSelect == 4) {
        buzzerSetting = !buzzerSetting;
        bufWriteInt(EEPROM_BUZZER_OFFSET, buzzerSetting);
        if (buzzerSetting == 1) tone(BUZZER, 1200, 80);
        delay(150);
      }

      if (cPressed && settingSelect == 5) {
        // Reset counters; no need to physically erase records
        // because we only read up to saveNum
        saveNum = 0;
        saveIndex = 0;
        bufWriteInt(0, saveNum);
        bufWriteInt(4, saveIndex);
        eeprom_buffer_flush();   // Write to flash immediately so the result is visible
        Serial.println("[EEPROM] Memory reset");
        if (buzzerSetting == 1) {
          tone(BUZZER, 600, 100); delay(120);
          tone(BUZZER, 800, 100);
        }
        delay(200);
      }

      display.setCursor(0, 0);
      display.setCursor(24, 0);
      display.println("-- SETTINGS --");
      display.setCursor(0, 10);

      display.setCursor(0, 10);
      display.print(settingSelect == 0 ? ">" : " ");
      display.print("DIST:");
      display.print((int)sensorDistanceCm);
      display.print("cm");
      drawSlider(78, 10, 48, 7, sensorDistanceCm, MIN_SENSOR_DISTANCE_CM, MAX_SENSOR_DISTANCE_CM);

      display.setCursor(0, 19);
      display.print(settingSelect == 1 ? ">" : " ");
      display.print("LIM:");
      if (speedUnitSetting == 1) {
        display.print(speedForDisplay(speedLimitCms), 1);
        display.print("kmh");
      } else {
        display.print((int)speedLimitCms);
        display.print("cms");
      }
      drawSlider(78, 19, 48, 7, speedLimitCms, MIN_SPEED_LIMIT_CMS, MAX_SPEED_LIMIT_CMS);

      display.setCursor(0, 28);
      display.print(settingSelect == 2 ? ">" : " ");
      display.print("UNIT:");
      display.print(speedUnitLabel());

      display.setCursor(0, 37);
      display.print(settingSelect == 3 ? ">" : " ");
      display.print("LED:");
      display.print(ledSetting == 1 ? "ON" : "OFF");

      display.setCursor(0, 46);
      display.print(settingSelect == 4 ? ">" : " ");
      display.print("BUZZER:");
      display.print(buzzerSetting == 1 ? "ON" : "OFF");

      display.setCursor(0, 55);
      display.print(settingSelect == 5 ? ">" : " ");
      display.print("RESET MEMORY");
    }
    display.display();

    lastBtnC = btnC;
    lastBtnB = btnB;
    lastBtnA = btnA;

    if (jsw && (millis() - lastBeep > 300)) {
      if (buzzerSetting == 1) {
      tone(BUZZER, 1200, 80);
      }
      lastBeep = millis();
    }
  }
}
