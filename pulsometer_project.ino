#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h" // SparkFun knihovna (funguje pro MAX30102)

// --- PINY (UPRAVTE DLE SVÉ DESKY, toto je pro ESP32) ---
#define I2C_SDA 21
#define I2C_SCL 22
#define LED_PIN 2       // Vestavěná LED
#define BUTTON_PIN 4    // Tlačítko proti zemi

// --- DISPLEJ ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C 
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// --- SENZOR ---
MAX30105 sensor;

// --- PROMĚNNÉ PRO ZPRACOVÁNÍ SIGNÁLU ---
float ir_dc_filter = 0;
float red_dc_filter = 0;

// Průměrování BPM
const byte RATE_SIZE = 4; 
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeatTime = 0; 
float beatsPerMinute;
int beatAvg = 0;
int SPO2 = 99; 

// Logika ovládání
#define OPTIONS_ADDR 0
volatile bool buttonPressed = false;
uint8_t displayMode = 3; // 1=FINGER, 2=GRAF, 3=START, 4=OFF, 5=INFO
uint8_t sleep_counter = 0;
long lastDisplayUpdate = 0;

// Logika detekce tepu (Fix pro double counting)
bool beatDetected = false; 

// Grafika
const uint8_t MAXWAVE = 128; 
uint8_t waveform[MAXWAVE];
uint8_t waveIndex = 0;

// --- FUNKCE ---

void drawWave() {
  uint8_t minVal = 255;
  uint8_t maxVal = 0;
  for(int i=0; i<MAXWAVE; i++) {
    if(waveform[i] < minVal) minVal = waveform[i];
    if(waveform[i] > maxVal) maxVal = waveform[i];
  }
  
  int scale = maxVal - minVal;
  if (scale < 10) scale = 10; 

  uint8_t ptr = waveIndex;
  for (int x = 0; x < MAXWAVE - 1; x++) {
    int y1 = 63 - map(waveform[ptr], minVal, maxVal, 0, 30);
    ptr = (ptr + 1) % MAXWAVE;
    int y2 = 63 - map(waveform[ptr], minVal, maxVal, 0, 30);
    oled.drawLine(x, y1, x+1, y2, SSD1306_WHITE);
  }
}

void IRAM_ATTR button_isr() {
  buttonPressed = true;
}

void handleButton() {
  if (buttonPressed && !digitalRead(BUTTON_PIN)) {
    if (displayMode == 2) {
       displayMode = 5; // Info
    } else if (displayMode == 5) {
       displayMode = 2; // Graf
    }
    delay(200); 
  }
  buttonPressed = false;
}

void go_sleep() {
  oled.clearDisplay();
  oled.display();
  sensor.shutDown();
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_PIN, 0); 
  Serial.println(F("Sleep..."));
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Inicializace I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Inicializace Displeje
  if(!oled.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("OLED failed"));
    for(;;);
  }
  oled.clearDisplay();
  oled.setTextSize(2);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(20, 20);
  oled.print(F("START..."));
  oled.display();
  delay(1000);

  // Inicializace Senzoru - ZPOMALENO na STANDARD pro stabilitu
  if (!sensor.begin(Wire, I2C_SPEED_STANDARD)) {
    oled.clearDisplay();
    oled.setCursor(0,0);
    oled.print(F("SENSOR ERR"));
    oled.display();
    while (1);
  }

  // --- KLÍČOVÉ NASTAVENÍ PRO VYHLAZENÍ GRAFU ---
  byte ledBrightness = 50; // Nižší jas = méně šumu (zkuste max 60)
  byte sampleAverage = 8;  // PRŮMĚROVÁNÍ: 8 vzorků (vyhladí "strouhaný sýr")
  byte ledMode = 2;        // Red + IR
  int sampleRate = 100;    // 100Hz
  int pulseWidth = 411;    // Max šířka pulzu
  int adcRange = 4096;     

  sensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), button_isr, FALLING);
  
  // Vynulování grafu
  for(int i=0; i<MAXWAVE; i++) waveform[i] = 128;
}

void loop() {
  sensor.check();
  
  while(sensor.available()) {
    uint32_t irValue = sensor.getFIFOIR();
    uint32_t redValue = sensor.getFIFORed();
    sensor.nextSample();

    // 1. Detekce prstu (Práh 50 000 - zvyšte/snižte dle potřeby)
    if (irValue < 50000) {
      ir_dc_filter = 0; 
      beatAvg = 0;
      beatDetected = false;
      displayMode = 1; 
      
      if (millis() - lastDisplayUpdate > 200) {
        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setCursor(30, 10);
        oled.print(F("NO FINGER"));
        oled.setCursor(30, 30);
        oled.print(F("Sleep: "));
        oled.print(10 - sleep_counter/5);
        oled.display();
        sleep_counter++;
        lastDisplayUpdate = millis();
      }
      
      if (sleep_counter > 50) go_sleep();
      continue; 
    }
    
    sleep_counter = 0;
    if(displayMode == 1) displayMode = 2; 

    // 2. Filtrace signálu (DC Removal)
    const float ALPHA = 0.95;
    
    if (ir_dc_filter == 0) ir_dc_filter = irValue;
    if (red_dc_filter == 0) red_dc_filter = redValue;

    ir_dc_filter = (ALPHA * ir_dc_filter) + ((1.0 - ALPHA) * irValue);
    red_dc_filter = (ALPHA * red_dc_filter) + ((1.0 - ALPHA) * redValue);

    float current_ir_ac = irValue - ir_dc_filter;
    float current_red_ac = redValue - red_dc_filter;

    // Graf - Invertujeme a posuneme na střed
    int plotVal = -current_ir_ac * 0.2 + 128; 
    plotVal = constrain(plotVal, 0, 255);
    waveform[waveIndex] = (uint8_t)plotVal;
    waveIndex = (waveIndex + 1) % MAXWAVE;

    // 3. DETEKCE TEPU S HYSTEREZÍ (Fix 180 BPM)
    long now = millis();
    
    // Práh: -100 (filtruje šum). Hystereze: !beatDetected (čeká na návrat nahoru)
    if ((current_ir_ac < -100) && (now - lastBeatTime > 300) && !beatDetected) { 
       
       beatDetected = true; // Zámek
       
       long delta = now - lastBeatTime;
       lastBeatTime = now;
       
       beatsPerMinute = 60000.0 / delta;

       if (beatsPerMinute < 255 && beatsPerMinute > 40) { 
         rates[rateSpot++] = (byte)beatsPerMinute;
         rateSpot %= RATE_SIZE;

         beatAvg = 0;
         for (byte x = 0 ; x < RATE_SIZE ; x++) beatAvg += rates[x];
         beatAvg /= RATE_SIZE;
         
         digitalWrite(LED_PIN, HIGH); 

         // SpO2
         float ratio = (abs(current_red_ac) / red_dc_filter) / (abs(current_ir_ac) / ir_dc_filter);
         float spo2Calc = 104 - 17 * ratio; 
         SPO2 = (int)spo2Calc;
         if(SPO2 > 100) SPO2 = 100;
         if(SPO2 < 80) SPO2 = 80; 
       }
    } 
    
    // Odemknutí detekce, až když signál vyleze zpět nahoru (nad -20)
    // Tím se ignoruje druhý kmit tepové vlny
    if (current_ir_ac > -20 && beatDetected) {
       beatDetected = false;
       digitalWrite(LED_PIN, LOW); 
    }

    handleButton();

    // 4. Vykreslování (max 25 FPS)
    if (now - lastDisplayUpdate > 40) {
       lastDisplayUpdate = now;
       
       if (displayMode == 2) {
         oled.clearDisplay();
         
         oled.setTextSize(1);
         oled.setCursor(0,0); oled.print(F("BPM"));
         oled.setCursor(90,0); oled.print(F("SpO2"));
         
         oled.setTextSize(2);
         oled.setCursor(0,12); oled.print(beatAvg);
         oled.setCursor(90,12); 
         oled.print(SPO2); oled.print(F("%"));

         drawWave();
         oled.display();
       } 
       else if (displayMode == 5) {
         oled.clearDisplay();
         oled.setTextSize(2);
         oled.setCursor(10,10); oled.print(F("AVG BPM"));
         oled.setCursor(40,35); oled.print(beatAvg);
         oled.display();
       }
    }
  }
}