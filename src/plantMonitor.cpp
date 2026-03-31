// ============================================================
//  Project     :  Plant Habitat Monitor
//  Author      :  Nicole
//  Date        :  2/20/26  (updated 3/30)
//  Description :  Smart plant monitor — auto watering + misting,
//                 two DIY resistive moisture sensors, manual buttons,
//                 Adafruit IO dashboard, OLED display, NeoPixel ring.
//
//  Hardware confirmed working:
//    Device OS  : 5.8.0  (best for NeoPixels on Photon 2)
//    NeoPixel   : SPI1,  WS2812B, 12 LEDs (1 × 12-LED ring)
//    BME280     : I2C 0x76
//    OLED       : SSD1306 128×64, I2C 0x3C
//    Gravel sensor  : A5 (power pulse), A0 (analog read)  threshold 500–1500
//    Reservoir sensor: D8 (power pulse), A2 (analog read)  threshold 500
//    Air quality: A1 (Grove MP-503)
//    Pump MOSFET: D4
//    Mist MOSFET: D5
//    Mode button: D9  (short = next OLED page, long = AUTO/MANUAL toggle)
//    Mist button: D7  (short = mist burst,     long = pump cycle)
//
//  Adafruit IO feeds:
//    PUBLISH  -> temperature, humidity, plant-status, airreading, mode
//    SUBSCRIBE<- pumponoff, misteronoff, modetoggle

//   Zapier       (email/SMS webhooks via Particle.publish)
//
//  Future: reservoir level feed, SMS alert, OLED bitmap graphics
// ============================================================

#include "Particle.h"
#include "credentials.h"
#include "IoTClassroom_CNM.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "gfxDisplay.h"
#include "Adafruit_BME280.h"
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "neopixel.h"
#include "Colors.h"
#include "Air_Quality_Sensor.h"

// SYSTEM_THREAD(ENABLED) moves WiFi/cloud to a background thread
// so loop() never freezes waiting for network events.
SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

// ─────────────────────────────────────────────
//  PIN ASSIGNMENTS
// ─────────────────────────────────────────────
// Gravel dish sensor (air plants) — resistive, pulsed to prevent electrolysis
const int GRAVEL_POWER_PIN    = A5;  // OUTPUT — pulse HIGH to read
const int GRAVEL_ANALOG_PIN   = A0;  // INPUT  — analog read

// Water reservoir sensor — same resistive pulsed technique
const int RESERVOIR_POWER_PIN = D8;  // OUTPUT — pulse HIGH to read
const int RESERVOIR_ANALOG_PIN = A2; // INPUT  — analog read

const int AIR_QUALITY_PIN     = A1;  // Grove MP-503 analog out
const int PUMP_PIN            = D4;  // MOSFET gate — pump
const int MIST_PIN            = D5;  // MOSFET gate — TD-002 mister
const int MODE_BUTTON_PIN     = D9;  // INPUT_PULLUP — short/long press
const int MIST_BUTTON_PIN     = D7;  // INPUT_PULLUP — short/long press
// NeoPixel data -> SPI1 (confirmed working, Device OS 5.8.0)

// ─────────────────────────────────────────────
//  THRESHOLDS  (calibrated to sensors)
// ─────────────────────────────────────────────

// Gravel dish (air plants):
//   Calibration: dry gravel ~100-300, wet gravel ~1000+
//   Hysteresis: turn pump ON below 500, OFF above 1500
const int GRAVEL_ON_THRESH    = 500;   // below this -> auto-water
const int GRAVEL_OFF_THRESH   = 1500;  // above this -> stop pump
                                        // (wide hysteresis avoids chatter)

// Reservoir:
//   Probes immersed: reading > 500 = water present
//   Probes in air:   reading < 500 = reservoir low
const int RESERVOIR_THRESH    = 500;   // below -> reservoir low alert

// Auto-misting (BME280 driven)
const float MIST_TEMP_THRESH  = 30.0;  // °C above -> consider misting
const float MIST_HUM_THRESH   = 25.0;  // %RH below -> auto-mist fires

// Actuator timings
const unsigned long PUMP_RUN_MS      = 5000;   // 5 s per water cycle
const unsigned long PUMP_COOLDOWN_MS = 60000;  // 60 s min between cycles
const unsigned long MIST_RUN_MS      = 3000;   // 3 s per mist burst
const unsigned long MODE_LOCK_MS     = 5000;   // dashboard can't override button for 5 s
const unsigned long MIN_PUMP_ON_MS   = 3000;   // minimum pump ON time

// Hysteresis thresholds (used in autoWateringLogic)
const int SOIL_ON_THRESH      = GRAVEL_ON_THRESH;
const int SOIL_OFF_THRESH     = GRAVEL_OFF_THRESH;

// ─────────────────────────────────────────────
//  TIMING  (millis-based)
// ─────────────────────────────────────────────
const unsigned long SENSOR_INTERVAL_MS = 10000;
const unsigned long MQTT_INTERVAL_MS   = 1800000; // 30 min 
const unsigned long NEO_INTERVAL_MS    = 100;
const unsigned long OLED_PAGE_MS       = 5000;

unsigned long lastSensorMs             = 0;
unsigned long lastMqttMs               = 0;
unsigned long lastNeoMs                = 0;
unsigned long lastOledMs               = 0;
unsigned long pumpStartMs              = 0;
unsigned long pumpStopMs               = 0;
unsigned long mistStartMs              = 0;
unsigned long lastPhysicalModeChangeMs = 0;

// ─────────────────────────────────────────────
//  NEOPIXEL  (single 12-LED ring via SPI1)
// ─────────────────────────────────────────────
const int PIXEL_COUNT = 12;
Adafruit_NeoPixel pixel(PIXEL_COUNT, SPI1, WS2812B);
int sizeArray = sizeof(rainbow) / sizeof(rainbow[0]);

// ─────────────────────────────────────────────
//  HARDWARE OBJECTS
// ─────────────────────────────────────────────
const int OLED_RESET = -1;
Adafruit_SSD1306  display(OLED_RESET);
Adafruit_BME280   bme;
AirQualitySensor  airSensor(AIR_QUALITY_PIN);

// ─────────────────────────────────────────────
//  MQTT
// ─────────────────────────────────────────────
TCPClient           TheClient;
Adafruit_MQTT_SPARK mqtt(&TheClient, AIO_SERVER, AIO_SERVERPORT,
                          AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Publish feedTemperature (&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish feedHumidity    (&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish feedPlantStatus (&mqtt, AIO_USERNAME "/feeds/plant-status");
Adafruit_MQTT_Publish feedAirReading  (&mqtt, AIO_USERNAME "/feeds/airreading");
Adafruit_MQTT_Publish feedMode        (&mqtt, AIO_USERNAME "/feeds/mode");

Adafruit_MQTT_Subscribe subPumpButton (&mqtt, AIO_USERNAME "/feeds/pumponoff");
Adafruit_MQTT_Subscribe subMistButton (&mqtt, AIO_USERNAME "/feeds/misteronoff");
Adafruit_MQTT_Subscribe subModeToggle (&mqtt, AIO_USERNAME "/feeds/modetoggle");

// ─────────────────────────────────────────────
//  SENSOR DATA
// ─────────────────────────────────────────────
float tempC = 0, tempF = 0, pressPA = 0, pressInHg = 0, humidRH = 0;
int   gravelRaw = 0;        // raw ADC from gravel dish sensor
int   reservoirRaw = 0;     // raw ADC from reservoir sensor
bool  reservoirLow = false; // true when reservoir needs refill
float airReading = 0;
int   airQuality = 0;

// ─────────────────────────────────────────────
//  STATE
// ─────────────────────────────────────────────
bool modeAuto           = true;
bool pumpRunning        = false;
bool misterRunning      = false;
bool alertSentThisCycle = false;

// ─────────────────────────────────────────────
//  OLED PAGE STATE MACHINE
//
//  Normal cycle (auto-advances every 5 s, or short-press D9):
//    PAGE_BOOT    -- title + timestamp (shown once at startup)
//    PAGE_PLANT   -- gravel moisture + humidity
//    PAGE_CLIMATE -- temp F + air quality + pressure
//    PAGE_DEVICES -- mister + pump + reservoir status
//    PAGE_MODE    -- AUTO / MANUAL watering mode
//
//  Interrupt pages (override while active):
//    pumpRunning   -> "WATERING!" screen
//    misterRunning -> "MISTING!"  screen
//
//  GRAPHICS:
//  Next step, add in bitmaps.
//  Finished bitmaps in gfxDisplay.h:
//    BITMAP_TEMP_F, BITMAP_BAROMETER, BITMAP_RH_PERCENT
//  Still to create:
//    BITMAP_PLANT_HAPPY, BITMAP_PLANT_DRY
//    BITMAP_MIST, BITMAP_PUMP, BITMAP_AUTO, BITMAP_MANUAL
// ─────────────────────────────────────────────
enum OledPage {
    PAGE_BOOT    = 0,
    PAGE_PLANT   = 1,
    PAGE_CLIMATE = 2,
    PAGE_DEVICES = 3,
    PAGE_MODE    = 4,
    PAGE_COUNT   = 5
};
int currentPage = PAGE_BOOT;

// ─────────────────────────────────────────────
//  BUTTON DEBOUNCE STATE
// ─────────────────────────────────────────────
int           lastModeButtonState = HIGH;
unsigned long modeButtonPressMs   = 0;
int           lastMistButtonState = HIGH;
unsigned long mistButtonPressMs   = 0;
const int     DEBOUNCE_MS         = 50;

// ─────────────────────────────────────────────
//  FORWARD DECLARATIONS
// ─────────────────────────────────────────────
void  MQTT_connect();
bool  MQTT_ping();
void  readSensors();
int   readResistiveSensor(int powerPin, int analogPin);
void  drawOledPage();
void  advancePage();
void  handleModeButton();
void  handleMistButton();
void  handleSubscriptions();
void  publishToAdafruitIO();
void  autoWateringLogic();
void  autoMistingLogic();
void  startPump();
void  stopPump();
void  startMister();
void  stopMister();
void  updateNeoPixel();
String getAirQualityLabel(int val);
String getTimeStamp();

// ─────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────
void setup() {
    Serial.begin(9600);
    waitFor(Serial.isConnected, 10000);
    Serial.println("Plant Monitor booting...");

    // -- GPIO --------------------------------------
    pinMode(PUMP_PIN,             OUTPUT); digitalWrite(PUMP_PIN, LOW);
    pinMode(MIST_PIN,             OUTPUT); digitalWrite(MIST_PIN, LOW);
    pinMode(MODE_BUTTON_PIN,      INPUT_PULLUP);
    pinMode(MIST_BUTTON_PIN,      INPUT_PULLUP);
    pinMode(GRAVEL_POWER_PIN,     OUTPUT); digitalWrite(GRAVEL_POWER_PIN, LOW);
    pinMode(GRAVEL_ANALOG_PIN,    INPUT);
    pinMode(RESERVOIR_POWER_PIN,  OUTPUT); digitalWrite(RESERVOIR_POWER_PIN, LOW);
    pinMode(RESERVOIR_ANALOG_PIN, INPUT);

    // Suppress onboard RGB — prevents interference with NeoPixel SPI
    RGB.control(true);
    RGB.color(0, 0, 0);

    // -- NeoPixel ----------------------------------
    pixel.begin();
    pixel.setBrightness(50);
    pixel.setPixelColor(0, pixel.Color(0, 160, 140)); // single teal pixel on boot
    pixel.show();
    Serial.println("NeoPixel OK");

    // -- I2C — must come before BME280 and OLED ---
    Wire.begin();

    // -- BME280 ------------------------------------
    if (!bme.begin(0x76)) {
        Serial.println("BME280 not found at 0x76 -- check wiring");
    } else {
        Serial.println("BME280 OK");
    }

    // -- OLED --------------------------------------
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.clearDisplay();
    display.display();
    Serial.println("OLED OK");

    // -- WiFi + Particle cloud ---------------------
    // waitFor() gives each step a time limit then continues regardless.
    WiFi.on();
    WiFi.connect();
    waitFor(WiFi.ready, 15000);
    if (WiFi.ready()) {
        Serial.println("WiFi connected");
        Particle.connect();
        waitFor(Particle.connected, 10000);
        if (Particle.connected()) {
            Particle.syncTime();
            Time.zone(-6); // Mountain Time — adjust +1 for MDT (summer)
            Serial.println("Particle cloud connected — time synced");
        }
    } else {
        Serial.println("WiFi timeout -- continuing offline");
    }

    // -- Air quality sensor ------------------------
    if (airSensor.init()) {
        Serial.println("Air sensor OK");
    } else {
        Serial.println("Air sensor init failed");
    }

    // -- MQTT subscriptions ------------------------
    mqtt.subscribe(&subPumpButton);
    mqtt.subscribe(&subMistButton);
    mqtt.subscribe(&subModeToggle);

    // -- Boot OLED page — show title + timestamp ---
    // Displayed once at startup before cycling begins
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(4, 2);  display.print("Plant");
    display.setCursor(4, 22); display.print("Monitor");
    display.setTextSize(1);
    display.setCursor(4, 46); display.print("for happy plants");
    display.setCursor(4, 56); display.print(getTimeStamp());
    display.display();
    Serial.printf("Boot time: %s\n", getTimeStamp().c_str());

    // Seed timers
    lastSensorMs = millis() - SENSOR_INTERVAL_MS + 3000; // first read in 3 s
    lastMqttMs   = millis();
    lastNeoMs    = millis();
    lastOledMs   = millis();
    pumpStopMs   = millis() - PUMP_COOLDOWN_MS; // pump ready immediately

    Serial.println("Plant Monitor ready!");
}

// ─────────────────────────────────────────────
//  LOOP
//  Traffic controller — delegates, never blocks.
// ─────────────────────────────────────────────
void loop() {
    unsigned long now = millis();

    // Sensor read every 10 s — auto logic runs ONLY after fresh data.
    // Keeping auto logic here prevents it from re-evaluating stale
    // gravelRaw thousands of times per second, which caused the pump
    // to restart immediately after stopping before the next real read.
    if (now - lastSensorMs >= SENSOR_INTERVAL_MS) {
        lastSensorMs = now;
        readSensors();
        if (modeAuto) {
            autoWateringLogic();
            autoMistingLogic();
        }
    }

    // Auto-off: pump and mister — checked every loop for accurate timing.
    // These use millis() elapsed time, not sensor data, so they are safe
    // to check continuously without causing the re-trigger problem above.
    if (pumpRunning   && (now - pumpStartMs >= PUMP_RUN_MS)) stopPump();
    if (misterRunning && (now - mistStartMs >= MIST_RUN_MS)) stopMister();

    // Buttons
    handleModeButton();
    handleMistButton();

    // MQTT keep-alive + subscriptions
    MQTT_connect();
    MQTT_ping();
    mqtt.processPackets(10);
    handleSubscriptions();

    // Publish every 30 min
    if (now - lastMqttMs >= MQTT_INTERVAL_MS) {
        lastMqttMs = now;
        publishToAdafruitIO();
    }

    // OLED: auto-advance page every 5 s (skip boot page after first cycle)
    if (now - lastOledMs >= OLED_PAGE_MS) {
        lastOledMs = now;
        if (!pumpRunning && !misterRunning) {
            advancePage();
        }
    }
    drawOledPage();

    // NeoPixel animation tick
    if (now - lastNeoMs >= NEO_INTERVAL_MS) {
        lastNeoMs = now;
        updateNeoPixel();
    }
}

// ─────────────────────────────────────────────
//  RESISTIVE SENSOR READ  (shared helper btw both sensors)
//
//  Both DIY sensors use the same technique:
//    1. Pulse power pin HIGH for 20ms — enough to get a stable read
//    2. Read the analog voltage
//    3. Pull power LOW immediately — prevents electrolysis on the wires
//
//  Dry / no water : high resistance → little current → low ADC
//  Wet / has water : low resistance → more current → higher ADC
//  (opposite of capacitive sensor — no inversion needed)
// ─────────────────────────────────────────────
int readResistiveSensor(int powerPin, int analogPin) {
    digitalWrite(powerPin, HIGH);
    delay(20);                      // 20 ms stabilisation pulse
    int val = analogRead(analogPin);
    digitalWrite(powerPin, LOW);
    return val;
}

// ─────────────────────────────────────────────
//  SENSOR READS
// ─────────────────────────────────────────────
void readSensors() {
    // BME280
    tempC     = bme.readTemperature();
    humidRH   = bme.readHumidity();
    tempF     = tempC * 1.8 + 32.0;
    pressPA   = bme.readPressure();
    pressInHg = 0.0002953 * pressPA;

    // Gravel dish sensor (air plants)
    gravelRaw = readResistiveSensor(GRAVEL_POWER_PIN, GRAVEL_ANALOG_PIN);

    // Reservoir water level sensor
    reservoirRaw = readResistiveSensor(RESERVOIR_POWER_PIN, RESERVOIR_ANALOG_PIN);
    reservoirLow = (reservoirRaw < RESERVOIR_THRESH);

    // Air quality
    airReading = airSensor.getValue();
    airQuality = airSensor.slope();

    Serial.printf("[BME]   %.1fF  %.1f%%RH  %.2f inHg\n", tempF, humidRH, pressInHg);
    Serial.printf("[GRVL]  raw=%d  %s\n", gravelRaw,
                  gravelRaw >= GRAVEL_ON_THRESH ? "moist" : "dry");
    Serial.printf("[RESV]  raw=%d  %s\n", reservoirRaw,
                  reservoirLow ? "LOW" : "OK");
    Serial.printf("[AIR]   %.1f  %s\n", airReading,
                  getAirQualityLabel(airQuality).c_str());
}

String getAirQualityLabel(int val) {
    if (val == AirQualitySensor::FRESH_AIR)     return "Good";
    if (val == AirQualitySensor::LOW_POLLUTION)  return "Fair";
    if (val == AirQualitySensor::HIGH_POLLUTION) return "Poor";
    if (val == AirQualitySensor::FORCE_SIGNAL)   return "Alert";
    return "---";
}

// ─────────────────────────────────────────────
//  TIMESTAMP
//
//  Returns current time as "HH:MM:SS" string.
//  Uses Particle.syncTime() called in setup()
//  Daylight savings for Time.zone():
//  Time.zone(-6) = MST.  Use -7 for MDT (summer).
// ─────────────────────────────────────────────
String getTimeStamp() {
    String dateTime = Time.timeStr();
    return dateTime.substring(11, 19); // extract HH:MM:SS
}

// ─────────────────────────────────────────────
//  OLED STATE MACHINE
// ─────────────────────────────────────────────
void advancePage() {
    // Boot page shows once, then skips into normal cycle
    if (currentPage == PAGE_BOOT) {
        currentPage = PAGE_PLANT;
    } else {
        // Cycle PAGE_PLANT -> PAGE_CLIMATE -> PAGE_DEVICES -> PAGE_MODE -> back
        currentPage = PAGE_PLANT + (currentPage - PAGE_PLANT + 1) % (PAGE_COUNT - 1);
    }
}

void drawOledPage() {
    display.clearDisplay();

    // Interrupt pages — pump takes priority over mist
    if (pumpRunning) {
        display.setTextSize(2);
        display.setCursor(10, 4);  display.print("WATERING!");
        display.setTextSize(1);
        display.setCursor(10, 36); display.print("Auto-pump running");
        display.setCursor(10, 48); display.printf("Gravel: %d", gravelRaw);
        display.setCursor(10, 56); display.print(getTimeStamp());
        // -- BITMAP VERSION (uncomment when ready) --
        // display.drawBitmap(44, 4, BITMAP_PUMP, 40, 40, WHITE);
        // display.setTextSize(1);
        // display.setCursor(10, 50); display.printf("Gravel: %d", gravelRaw);
        display.display();
        return;
    }

    if (misterRunning) {
        display.setTextSize(2);
        display.setCursor(10, 4);  display.print("MISTING!");
        display.setTextSize(1);
        display.setCursor(10, 36); display.print("Mist cycle active");
        display.setCursor(10, 48); display.printf("Humid: %.0f%%", humidRH);
        display.setCursor(10, 56); display.print(getTimeStamp());
        // -- BITMAP VERSION --
        // display.drawBitmap(44, 4, BITMAP_MIST, 40, 40, WHITE);
        // display.setTextSize(1);
        // display.setCursor(10, 50); display.printf("Humid: %.0f%%", humidRH);
        display.display();
        return;
    }

    switch (currentPage) {

        case PAGE_BOOT: {
            // Boot page — title card + timestamp
            // Stays up until first 5-second cycle or button press
            display.setTextSize(2);
            display.setCursor(4, 2);  display.print("Plant");
            display.setCursor(4, 22); display.print("Monitor");
            display.setTextSize(1);
            display.setCursor(4, 46); display.print("Nicole  CNM IoT");
            display.setCursor(4, 56); display.print(getTimeStamp());
            break;
        }

        case PAGE_PLANT: {
            // Gravel moisture + humidity
            bool happy = (gravelRaw >= GRAVEL_ON_THRESH);
            display.setTextSize(2);
            display.setCursor(8, 4);
            display.print(happy ? ":) Happy" : ":( Dry!");
            display.setTextSize(1);
            display.setCursor(8, 36); display.printf("Gravel:%4d", gravelRaw);
            display.setCursor(8, 48); display.printf("Humid: %.0f%%", humidRH);
            display.setCursor(8, 56); display.print(getTimeStamp());
            // -- BITMAP VERSION --
            // const unsigned char* bmp = happy ? BITMAP_PLANT_HAPPY : BITMAP_PLANT_DRY;
            // display.drawBitmap(4, 12, bmp, 40, 40, WHITE);
            // display.setTextSize(1);
            // display.setCursor(50, 16); display.printf("Gravel %d", gravelRaw);
            // display.setCursor(50, 32); display.printf("Hum  %.0f%%", humidRH);
            // display.setCursor(50, 48); display.print(happy ? "Happy!" : "Needs water!");
            break;
        }

        case PAGE_CLIMATE: {
            display.setTextSize(2);
            display.setCursor(8, 4);  display.printf("%.1fF", tempF);
            display.setTextSize(1);
            display.setCursor(8, 36); display.printf("Air:   %s",
                                      getAirQualityLabel(airQuality).c_str());
            display.setCursor(8, 48); display.printf("Press: %.2f inHg", pressInHg);
            display.setCursor(8, 56); display.print(getTimeStamp());
            // -- BITMAP VERSION --
            // display.drawBitmap(4,  12, BITMAP_TEMP_F,     40, 40, WHITE);
            // display.drawBitmap(88, 12, BITMAP_BAROMETER,  40, 40, WHITE);
            // display.setTextSize(1);
            // display.setCursor(50, 16); display.printf("%.1fF", tempF);
            // display.setCursor(50, 32); display.printf("Air: %s", getAirQualityLabel(airQuality).c_str());
            // display.setCursor(50, 48); display.printf("%.2f inHg", pressInHg);
            break;
        }

        case PAGE_DEVICES: {
            // Mister, pump, reservoir status
            display.setTextSize(1);
            display.setCursor(8, 4);  display.print("-- Devices --");
            display.setCursor(8, 18); display.printf("Mister:   %s",
                                      misterRunning ? "ON " : "off");
            display.setCursor(8, 30); display.printf("Pump:     %s",
                                      pumpRunning   ? "ON " : "off");
            display.setCursor(8, 42); display.printf("Reservoir:%s",
                                      reservoirLow  ? "LOW!" : "OK ");
            display.setCursor(8, 56); display.print(getTimeStamp());
            // -- BITMAP VERSION --
            // display.drawBitmap(4,  12, BITMAP_MIST, 40, 40, WHITE);
            // display.drawBitmap(84, 12, BITMAP_PUMP, 40, 40, WHITE);
            // display.setTextSize(1);
            // display.setCursor(4,  54); display.print(misterRunning ? "MIST ON" : "mist off");
            // display.setCursor(84, 54); display.print(pumpRunning   ? "PUMP ON" : "pump off");
            break;
        }

        case PAGE_MODE: {
            display.setTextSize(2);
            display.setCursor(8, 4);
            display.print(modeAuto ? "AUTO" : "MANUAL");
            display.setTextSize(1);
            display.setCursor(8, 36); display.print(modeAuto
                                      ? "Auto-watering ON"
                                      : "Manual mode");
            display.setCursor(8, 48); display.print("Hold D9 to toggle");
            display.setCursor(8, 56); display.print(getTimeStamp());
            // -- BITMAP VERSION --
            // const unsigned char* bmp = modeAuto ? BITMAP_AUTO : BITMAP_MANUAL;
            // display.drawBitmap(44, 4, bmp, 40, 40, WHITE);
            // display.setTextSize(1);
            // display.setCursor(8, 50); display.print(modeAuto ? "Auto-watering ON" : "Manual mode");
            break;
        }
    }
    display.display();
}

// ─────────────────────────────────────────────
//  AUTO WATERING  (gravel dish sensor)
//
//  Hysteresis: ON below GRAVEL_ON_THRESH (500),
//              OFF above GRAVEL_OFF_THRESH (1500).
//  Wide gap prevents pump chatter from marginal readings.
//  Minimum ON time + cooldown protect pump motor.
// ─────────────────────────────────────────────
void autoWateringLogic() {
    unsigned long now = millis();

    if (pumpRunning) {
        if (now - pumpStartMs < MIN_PUMP_ON_MS) return;
        if (gravelRaw > SOIL_OFF_THRESH) {
            Serial.printf("[AUTO] Gravel moist (%d) -> stopping pump\n", gravelRaw);
            stopPump();
            // Return immediately — pumpStopMs just set, cooldown starts now.
        }
        return;
    }

    // Cooldown guard — PUMP_COOLDOWN_MS (60 s) >> SENSOR_INTERVAL_MS (10 s).
    // This blocks re-trigger even within the same sensor cycle.
    if (now - pumpStopMs < PUMP_COOLDOWN_MS) {
        Serial.printf("[AUTO] Cooldown: %lu s remaining\n",
                      (PUMP_COOLDOWN_MS - (now - pumpStopMs)) / 1000);
        return;
    }

    // Floating ADC guard — disconnected sensor reads ~4095.
    if (gravelRaw >= 4090) {
        Serial.println("[AUTO] Gravel sensor open circuit -- skipping");
        return;
    }

    // Don't run pump dry
    if (reservoirLow) {
        Serial.println("[AUTO] Reservoir low -- skipping water cycle");
        return;
    }

    if (gravelRaw < SOIL_ON_THRESH) {
        Serial.printf("[AUTO] Gravel dry (%d) -> starting pump\n", gravelRaw);
        startPump();
        if (!alertSentThisCycle) {
            Particle.publish("soil_dry", String(gravelRaw), PRIVATE);
            alertSentThisCycle = true;
        }
    } else {
        alertSentThisCycle = false;
    }
}

void autoMistingLogic() {
    if (misterRunning) return;
    if (tempC > MIST_TEMP_THRESH && humidRH < MIST_HUM_THRESH) {
        Serial.println("[AUTO] Hot+dry -- misting");
        startMister();
    }
}

// ─────────────────────────────────────────────
//  ACTUATOR CONTROL
// ─────────────────────────────────────────────
void startPump() {
    if (pumpRunning) return;
    if (reservoirLow) {
        Serial.println("[PUMP] Blocked — reservoir low");
        return;
    }
    digitalWrite(PUMP_PIN, HIGH);
    pumpRunning = true;
    pumpStartMs = millis();
    feedPlantStatus.publish("Pump ON");
    Serial.println("[PUMP] ON");
}

void stopPump() {
    digitalWrite(PUMP_PIN, LOW);
    pumpRunning = false;
    pumpStopMs  = millis();
    feedPlantStatus.publish("Pump OFF");
    Serial.println("[PUMP] OFF");
}

void startMister() {
    if (misterRunning) return;
    digitalWrite(MIST_PIN, HIGH);
    misterRunning = true;
    mistStartMs   = millis();
    feedPlantStatus.publish("Mist ON");
    Serial.println("[MIST] ON");
}

void stopMister() {
    digitalWrite(MIST_PIN, LOW);
    misterRunning = false;
    feedPlantStatus.publish("Mist OFF");
    Serial.println("[MIST] OFF");
}

// ─────────────────────────────────────────────
//  BUTTON HANDLERS
//
//  MODE button D9:
//    Short < 1s  -> advance OLED page
//    Long  >= 1s -> toggle AUTO / MANUAL, jump to PAGE_MODE
//
//  MIST button D7:
//    Short < 1s  -> manual mist burst
//    Long  >= 1s -> manual pump cycle
// ─────────────────────────────────────────────
void handleModeButton() {
    int state = digitalRead(MODE_BUTTON_PIN);  // int, not bool

    if (lastModeButtonState == HIGH && state == LOW) {
        modeButtonPressMs = millis();   // falling edge: record press time
    }

    if (lastModeButtonState == LOW && state == HIGH) {
        unsigned long held = millis() - modeButtonPressMs;
        if (held > DEBOUNCE_MS) {
            if (held >= 1000) {
                modeAuto = !modeAuto;
                feedMode.publish(modeAuto ? "AUTO" : "MANUAL");
                currentPage = PAGE_MODE;
                lastOledMs  = millis();
                lastPhysicalModeChangeMs = millis();
                Serial.printf("[BTN MODE] -> %s\n", modeAuto ? "AUTO" : "MANUAL");
            } else {
                advancePage();
                lastOledMs = millis();
                Serial.println("[BTN MODE] Next page");
            }
        }
    }
    lastModeButtonState = state;
}

void handleMistButton() {
    int state = digitalRead(MIST_BUTTON_PIN);  // int, not bool

    if (lastMistButtonState == HIGH && state == LOW) {
        mistButtonPressMs = millis();
    }

    if (lastMistButtonState == LOW && state == HIGH) {
        unsigned long held = millis() - mistButtonPressMs;
        if (held > DEBOUNCE_MS) {
            if (held >= 1000) {
                startPump();
                Serial.println("[BTN MIST] Long -> pump cycle");
            } else {
                startMister();
                Serial.println("[BTN MIST] Short -> mist burst");
            }
        }
    }
    lastMistButtonState = state;
}

// ─────────────────────────────────────────────
//  MQTT SUBSCRIPTIONS
//
//  Mode-lock: dashboard can't override a recent physical button
//  press for MODE_LOCK_MS milliseconds.
// ─────────────────────────────────────────────
void handleSubscriptions() {
    Adafruit_MQTT_Subscribe* sub = mqtt.readSubscription(10);

    if (sub == &subPumpButton) {
        String val = String((char*)subPumpButton.lastread);
        if (val == "1" || val.equalsIgnoreCase("ON")) {
            startPump();
        } else {
            stopPump();
        }
        Serial.printf("[SUB] pumponoff -> %s\n", val.c_str());
    }

    if (sub == &subMistButton) {
        String val = String((char*)subMistButton.lastread);
        if (val == "1" || val.equalsIgnoreCase("ON")) {
            startMister();
        } else {
            stopMister();
        }
        Serial.printf("[SUB] misteronoff -> %s\n", val.c_str());
    }

    if (sub == &subModeToggle) {
        if (millis() - lastPhysicalModeChangeMs < MODE_LOCK_MS) return;
        String val = String((char*)subModeToggle.lastread);
        modeAuto = (val == "1" || val.equalsIgnoreCase("AUTO")
                    || val.equalsIgnoreCase("ON"));
        feedMode.publish(modeAuto ? "AUTO" : "MANUAL");
        Serial.printf("[SUB] modetoggle -> %s\n", val.c_str());
    }
}

// ─────────────────────────────────────────────
//  PUBLISH TO ADAFRUIT IO  (every 30 min)
// ─────────────────────────────────────────────
void publishToAdafruitIO() {
    if (!mqtt.connected()) return;

    feedTemperature.publish(tempF);                         delay(150);
    feedHumidity.publish(humidRH);                          delay(150);
    feedAirReading.publish(airReading);                     delay(150);
    feedMode.publish(modeAuto ? "AUTO" : "MANUAL");         delay(150);

    char buf[96];
    snprintf(buf, sizeof(buf),
             "Grvl:%d %s  Resv:%s  Tmp:%.1fF  Hum:%.0f%%  Air:%s  %s",
             gravelRaw,
             (gravelRaw < GRAVEL_ON_THRESH) ? "DRY" : "OK",
             reservoirLow ? "LOW!" : "OK",
             tempF, humidRH,
             getAirQualityLabel(airQuality).c_str(),
             getTimeStamp().c_str());
    feedPlantStatus.publish(buf);

    Serial.println("[MQTT] Published all feeds.");
}

// ─────────────────────────────────────────────
//  MQTT CONNECT / PING
// ─────────────────────────────────────────────
void MQTT_connect() {
    int8_t  ret;
    uint8_t tries = 0;
    if (mqtt.connected()) return;
    Serial.print("Connecting to Adafruit IO...");
    while ((ret = mqtt.connect()) != 0 && tries < 3) {
        Serial.printf(" Error: %s -- retry %d/3\n",
                      mqtt.connectErrorString(ret), tries + 1);
        mqtt.disconnect();
        delay(3000);
        tries++;
    }
    if (mqtt.connected()) {
        Serial.println(" connected!");
    } else {
        Serial.println(" gave up -- will retry next loop.");
    }
}

bool MQTT_ping() {
    static unsigned long lastPing = 0;
    bool status = true;
    if (millis() - lastPing > 120000) {
        status = mqtt.ping();
        if (!status) {
            Serial.println("Ping failed -- disconnecting");
            mqtt.disconnect();
        }
        lastPing = millis();
    }
    return status;
}

// ─────────────────────────────────────────────
//  NEOPIXEL ANIMATION  (single 12-LED ring)
//
//  Priority: mist sparkle > pump chase > manual gold
//            > dry red pulse > idle rainbow breathe
//
//  Uses Colors.h
// ─────────────────────────────────────────────
void updateNeoPixel() {
    static uint8_t  step = 0;
    static uint16_t hue  = 0;
    pixel.clear();

    if (misterRunning) {
        for (int i = 0; i < PIXEL_COUNT; i++) {
            pixel.setPixelColor((i + step) % PIXEL_COUNT,
                                pixel.Color(0, 160 + (step * 3 % 60), 180));
        }
        step = (step + 3) % PIXEL_COUNT;

    } else if (pumpRunning) {
        for (int i = 0; i < 4; i++) {
            pixel.setPixelColor((step + i) % PIXEL_COUNT,
                                pixel.Color(0, 200, 60));
        }
        step = (step + 1) % PIXEL_COUNT;

    } else if (!modeAuto) {
        for (int i = 0; i < PIXEL_COUNT; i++) {
            pixel.setPixelColor(i, pixel.Color(180, 140, 0));
        }

    } else if (gravelRaw < GRAVEL_ON_THRESH) {
        // Red pulse — gravel dry alert
        uint8_t bright = (uint8_t)(80 + 60 * sin(step * 0.15f));
        for (int i = 0; i < PIXEL_COUNT; i++) {
            pixel.setPixelColor(i, pixel.Color(bright, 0, 0));
        }
        step++;

    } else {
        // Idle: slow breathing rainbow from Colors.h palette
        uint8_t  bright     = (uint8_t)(30 + 20 * sin(step * 0.08f));
        int      colorIndex = (hue / 1000) % sizeArray;
        uint32_t base       = rainbow[colorIndex];
        uint8_t  r = ((base >> 16) & 0xFF) * bright / 255;
        uint8_t  g = ((base >>  8) & 0xFF) * bright / 255;
        uint8_t  b = ((base      ) & 0xFF) * bright / 255;
        for (int i = 0; i < PIXEL_COUNT; i++) {
            pixel.setPixelColor(i, pixel.Color(r, g, b));
        }
        hue  += 150;
        step++;
    }
    pixel.show();
}

// ── End of plantMonitor.cpp ──────────────────
