// ============================================================
//  Project :   Plant Habitat Monitor
//  Author  :   Nicole 
//           
//  Date    :   3/26/2026
//
//  Hardware confirmed working:
//    Device OS  : 5.8.0  (best for NeoPixels on Photon 2)
//    NeoPixel   : D2, WS2812B, 24 LEDs (2x12 rings)
//    BME280     : I2C 0x76
//    OLED       : SSD1306 128x64, I2C 0x3C
//    Soil sensor: A0 (capacitive)
//    Air quality: A1 (Grove MP-503)
//    Pump relay : D4
//    Mist relay : D5
//    Mode button: D9  (short=next OLED page, long=AUTO/MANUAL toggle)
//    Mist button: D7  (short=mist burst, long=pump cycle)
//
//  Adafruit IO feeds:
//    PUBLISH  -> temperature, humidity, plant-status,
//               airreading, mode
//    SUBSCRIBE<- pumponoff, misteronoff, modetoggle
//
//  Future improvements: resevoir level sensor, sms alert if system needs attention, OLED graphics
//
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

//#gfxDisplay.h //future home for OLED bitmaps!

// SYSTEM_THREAD(ENABLED) moves WiFi/cloud to a background thread
// so loop() never freezes waiting for network events.
SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

// ─────────────────────────────────────────────
//  PIN ASSIGNMENTS  (function first, then "Pin")
// ─────────────────────────────────────────────
const int SOIL_MOISTURE_PIN = A0;
const int AIR_QUALITY_PIN   = A1;
const int PUMP_PIN          = D4;
const int MIST_PIN          = D5;
const int MODE_BUTTON_PIN   = D9;
const int MIST_BUTTON_PIN   = D7;
// NeoPixel data -> D2 (confirmed working, Device OS 5.8.0)

// ─────────────────────────────────────────────
//  THRESHOLDS
// ─────────────────────────────────────────────
const int   SOIL_DRY_VALUE    = 3500; //from 3200
const int   SOIL_WET_VALUE    = 1500;
// Calibration: air=3392, wet soil=3335, dry soil=3360, water=1460
// Air plants in gravel sit near the dry end -- these values are solid.
const int   SOIL_WATER_THRESH = 30;    // % below -> auto-water

const float MIST_TEMP_THRESH  = 30.0;  // deg C above -> consider misting
const float MIST_HUM_THRESH   = 25.0;  // %RH below  -> auto-mist fires

const unsigned long PUMP_RUN_MS      = 5000;   // 5 s per water cycle
const unsigned long PUMP_COOLDOWN_MS = 60000;  // 60 s min between pump cycles
const unsigned long MIST_RUN_MS      = 3000;   // 3 s per mist burst
const unsigned long MODE_LOCK_MS     = 5000;   // dashboard can't override button for 5 s

// ─────────────────────────────────────────────
//  TIMING  (millis-based -- no blocking delays)
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
unsigned long pumpStopMs               = 0;   // cooldown reference
unsigned long mistStartMs              = 0;
unsigned long lastPhysicalModeChangeMs = 0;   // mode-lock reference

// ─────────────────────────────────────────────
//  NEOPIXEL
// ─────────────────────────────────────────────
const int PIXEL_COUNT = 24;   // 2 x 12-LED rings daisy-chained
Adafruit_NeoPixel pixel(PIXEL_COUNT, SPI1); //format to appease neoPixel.h constructor
int sizeArray = sizeof(rainbow) / sizeof(rainbow[0]);

// ─────────────────────────────────────────────
//  HARDWARE OBJECTS
// ─────────────────────────────────────────────
const int OLED_RESET = -1;   // no hardware reset pin
Adafruit_SSD1306 display(OLED_RESET);
Adafruit_BME280  bme;
AirQualitySensor airSensor(AIR_QUALITY_PIN);

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
int   soilRaw = 0, soilPct = 0;
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
//    PAGE_PLANT   -- plant happy/dry + soil % + humidity
//    PAGE_CLIMATE -- temp F + air quality + pressure
//    PAGE_DEVICES -- mister + pump status
//    PAGE_MODE    -- AUTO / MANUAL watering mode
//
//  Interrupt pages (override normal cycle while active):
//    pumpRunning   -> "WATERING!" screen
//    misterRunning -> "MISTING!"  screen
//    pump takes priority if both are somehow active
//
//  GRAPHICS UPGRADE PATH:
//  Text layout is live. Bitmap blocks sit commented out directly
//  below each case -- swap text block for bitmap block when ready.
//  Confirmed bitmaps in gfxDisplay.h:
//    BITMAP_TEMP_F, BITMAP_BAROMETER, BITMAP_RH_PERCENT
//  Still to create:
//    BITMAP_PLANT_HAPPY, BITMAP_PLANT_DRY
//    BITMAP_MIST, BITMAP_PUMP, BITMAP_AUTO, BITMAP_MANUAL
// ─────────────────────────────────────────────
enum OledPage {
    PAGE_PLANT   = 0,
    PAGE_CLIMATE = 1,
    PAGE_DEVICES = 2,
    PAGE_MODE    = 3,
    PAGE_COUNT   = 4
};
int currentPage = PAGE_PLANT;

// ─────────────────────────────────────────────
//  BUTTON DEBOUNCE STATE
// ─────────────────────────────────────────────
bool          lastModeButtonState = HIGH;
unsigned long modeButtonPressMs   = 0;
bool          lastMistButtonState = HIGH;
unsigned long mistButtonPressMs   = 0;
const int     DEBOUNCE_MS         = 50;

// ─────────────────────────────────────────────
//  FORWARD DECLARATIONS
// ─────────────────────────────────────────────
void MQTT_connect();
bool MQTT_ping();
void readSensors();
void drawOledPage();
void advancePage();
void handleModeButton();
void handleMistButton();
void handleSubscriptions();
void publishToAdafruitIO();
void autoWateringLogic();
void autoMistingLogic();
void startPump();
void stopPump();
void startMister();
void stopMister();
void updateNeoPixel();
int  mapSoilToPercent(int raw);
void drawDisplay(String timeDateLabel); // Function to draw the OLED display with moisture level and time-stamp
String getAirQualityLabel(int val);
String timeStamp(); // Function to get the current time and date as a string for the time-stamp
String timeOnly; // Variable to store the current date and time


// ─────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────
void setup() {
    Serial.begin(9600);
    waitFor(Serial.isConnected, 10000);
    Serial.println("Plant Monitor booting...");

    // -- GPIO --------------------------------------
    pinMode(PUMP_PIN,        OUTPUT); digitalWrite(PUMP_PIN, LOW);
    pinMode(MIST_PIN,        OUTPUT); digitalWrite(MIST_PIN, LOW);
    pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
    pinMode(MIST_BUTTON_PIN, INPUT_PULLUP);

    // Suppress onboard RGB -- prevents interference with NeoPixel
    RGB.control(true);
    RGB.color(0, 0, 0);

    // -- NeoPixel ----------------------------------
    pixel.begin();
    pixel.setBrightness(50);
    pixel.show();
    Serial.println("NeoPixel OK");

    // -- I2C -- must come before BME280 and OLED --
    Wire.begin();

    // -- BME280 ------------------------------------
    bool status = bme.begin(0x76);
    if (!status) {
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
    timeOnly = timeStamp(); // Get the current time and date for the initial time-stamp
    Time.zone(-6); // Set the time zone to Mountain Time (UTC-7) for accurate time-stamping (adjust for daylight savings time)
    Particle.syncTime();
    drawDisplay(timeOnly); // Draw the initial display with label and time-stamp
    display.display();

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

    // -- WiFi + Particle cloud ---------------------
    // waitFor() gives each step a time limit then continues regardless.
    // Device keeps working offline if WiFi is unavailable.
    WiFi.on();
    WiFi.connect();
    waitFor(WiFi.ready, 15000);
    if (WiFi.ready()) {
        Serial.println("WiFi connected");
        Particle.connect();
        waitFor(Particle.connected, 10000);
        if (Particle.connected()) {
            Particle.syncTime();
            Serial.println("Particle cloud connected");
        }
    } else {
        Serial.println("WiFi timeout -- continuing offline");
    }

    // Seed timers so first sensor read fires immediately on boot
    lastSensorMs = millis() - SENSOR_INTERVAL_MS + 3000;
    lastMqttMs   = millis();
    lastNeoMs    = millis();
    lastOledMs   = millis();
    pumpStopMs   = millis() - PUMP_COOLDOWN_MS;  // pump ready immediately

    Serial.println("Plant Monitor ready!");
/* 
    delay(5000);        //for easy testing
    digitalWrite(PUMP_PIN, HIGH);
    delay(5000);
    digitalWrite(PUMP_PIN, LOW);
    delay(5000); */
}

// ─────────────────────────────────────────────
//  LOOP
//  Traffic controller -- checks everything quickly, delegates.
//  Nothing here blocks (no delay, no while loops).
// ─────────────────────────────────────────────
void loop() {
    unsigned long now = millis();
    // -- Sensor read every 10 s --------------------
    if (now - lastSensorMs >= SENSOR_INTERVAL_MS) {
        lastSensorMs = now;
        readSensors();
    //    if (modeAuto) {
     //       autoWateringLogic();
      //      autoMistingLogic();
        //}
    }
     if (modeAuto) {
          autoWateringLogic();
          autoMistingLogic();
     }

    // -- Auto-off: pump and mister -----------------
    if (pumpRunning   && (now - pumpStartMs >= PUMP_RUN_MS)) stopPump();
    if (misterRunning && (now - mistStartMs >= MIST_RUN_MS)) stopMister();

    // -- Buttons -----------------------------------
    handleModeButton();
    handleMistButton();

    // -- MQTT keep-alive ---------------------------
    MQTT_connect();
    MQTT_ping();
    mqtt.processPackets(10);  // listen for incoming subscriptions, 10ms window
    handleSubscriptions();

    // -- Publish every 30 min ----------------------
    if (now - lastMqttMs >= MQTT_INTERVAL_MS) {
        lastMqttMs = now;
        publishToAdafruitIO();
    }

    // -- OLED: auto-advance page every 5 s ---------
    if (now - lastOledMs >= OLED_PAGE_MS) {
        lastOledMs = now;
        if (!pumpRunning && !misterRunning) advancePage();
    }
    drawOledPage();

    // -- NeoPixel animation tick -------------------
    if (now - lastNeoMs >= NEO_INTERVAL_MS) {
        lastNeoMs = now;
        updateNeoPixel();
    }
}

// ─────────────────────────────────────────────
//  SENSOR READS
//
// ─────────────────────────────────────────────
void readSensors() {
    tempC     = bme.readTemperature();
    humidRH   = bme.readHumidity();
    tempF     = tempC * 1.8 + 32;
    pressPA   = bme.readPressure();
    pressInHg = 0.0002953 * pressPA;

    soilRaw = analogRead(SOIL_MOISTURE_PIN);
    if (soilRaw >= 4090) {
        // 4095 = floating pin (sensor unplugged or water-damaged)
        // Return early -- don't update soilPct, don't trigger auto-logic
        Serial.println("[SOIL] sensor disconnected -- skipping");
        return;
    }
    soilPct = mapSoilToPercent(soilRaw);

    airReading = airSensor.getValue();
    airQuality = airSensor.slope();

    Serial.printf("[BME]  %.1fF  %.1f%%RH  %.2f inHg\n", tempF, humidRH, pressInHg);
    Serial.printf("[SOIL] raw=%d  %d%%\n", soilRaw, soilPct);
    Serial.printf("[AIR]  %.1f  %s\n", airReading, getAirQualityLabel(airQuality).c_str());
}

// mapSoilToPercent: constrain() clamps to known range,
// map() stretches that range to 0-100%.
int mapSoilToPercent(int raw) {
    raw = constrain(raw, SOIL_WET_VALUE, SOIL_DRY_VALUE);
    return map(raw, SOIL_DRY_VALUE, SOIL_WET_VALUE, 0, 100);
}

// :: is the scope resolution operator -- "look inside AirQualitySensor
// for the constant named FRESH_AIR". Like opening a specific drawer
// in a specific cabinet.
String getAirQualityLabel(int val) {
    if (val == AirQualitySensor::FRESH_AIR)     return "Good";
    if (val == AirQualitySensor::LOW_POLLUTION)  return "Fair";
    if (val == AirQualitySensor::HIGH_POLLUTION) return "Poor";
    if (val == AirQualitySensor::FORCE_SIGNAL)   return "Alert";
    return "---";
}

// ─────────────────────────────────────────────
//  OLED STATE MACHINE
// ─────────────────────────────────────────────

// % wraps currentPage like a clock: 0,1,2,3,0,1,2,3...
void advancePage() {
    currentPage = (currentPage + 1) % PAGE_COUNT;
}

void drawOledPage() {
    display.clearDisplay();

    // Interrupt pages override normal cycle.
    // Pump takes priority over mist if both somehow active.
    if (pumpRunning) {
        display.setTextSize(2);
        display.setCursor(10, 8);  display.print("WATERING!");
        display.setTextSize(1);
        display.setCursor(10, 40); display.print("Auto-pump running");
        display.setCursor(10, 52); display.printf("Soil: %d%%", soilPct);
        // -- BITMAP VERSION --
        // display.drawBitmap(44, 4, BITMAP_PUMP, 40, 40, WHITE);
        // display.setTextSize(1);
        // display.setCursor(10, 50); display.printf("Soil: %d%%", soilPct);
        display.display();
        return;
    }

    if (misterRunning) {
        display.setTextSize(2);
        display.setCursor(10, 8);  display.print("MISTING!");
        display.setTextSize(1);
        display.setCursor(10, 40); display.print("Mist cycle active");
        display.setCursor(10, 52); display.printf("Humid: %.0f%%", humidRH);
        // -- BITMAP VERSION --
        // display.drawBitmap(44, 4, BITMAP_MIST, 40, 40, WHITE);
        // display.setTextSize(1);
        // display.setCursor(10, 50); display.printf("Humid: %.0f%%", humidRH);
        display.display();
        return;
    }

    switch (currentPage) {

        case PAGE_PLANT: {
            bool happy = (soilPct >= SOIL_WATER_THRESH);
            display.setTextSize(2);
            display.setCursor(8, 4);
            display.print(happy ? ":) Happy" : ":( Dry!");
            display.setTextSize(1);
            display.setCursor(8, 36); display.printf("Soil:  %d%%", soilPct);
            display.setCursor(8, 50); display.printf("Humid: %.0f%%", humidRH);
            // -- BITMAP VERSION --
            // const unsigned char* bmp = happy ? BITMAP_PLANT_HAPPY : BITMAP_PLANT_DRY;
            // display.drawBitmap(4, 12, bmp, 40, 40, WHITE);
            // display.setTextSize(1);
            // display.setCursor(50, 16); display.printf("Soil  %d%%", soilPct);
            // display.setCursor(50, 32); display.printf("Hum   %.0f%%", humidRH);
            // display.setCursor(50, 48); display.print(happy ? "Happy!" : "Needs water!");
            break;
        }

        case PAGE_CLIMATE: {
            display.setTextSize(2);
            display.setCursor(8, 4);  display.printf("%.1fF", tempF);
            display.setTextSize(1);
            display.setCursor(8, 36); display.printf("Air:   %s", getAirQualityLabel(airQuality).c_str());
            display.setCursor(8, 50); display.printf("Press: %.2f inHg", pressInHg);
            // -- BITMAP VERSION --
            // display.drawBitmap(4,  12, BITMAP_TEMP_F,     40, 40, WHITE);
            // display.drawBitmap(88, 12, BITMAP_RH_PERCENT, 40, 40, WHITE);
            // display.setTextSize(1);
            // display.setCursor(50, 16); display.printf("%.1fF", tempF);
            // display.setCursor(50, 32); display.printf("Air: %s", getAirQualityLabel(airQuality).c_str());
            // display.setCursor(50, 48); display.printf("%.2f inHg", pressInHg);
            break;
        }

        case PAGE_DEVICES: {
            display.setTextSize(1);
            display.setCursor(8, 8);  display.print("-- Devices --");
            display.setCursor(8, 26); display.printf("Mister: %s", misterRunning ? "ON " : "off");
            display.setCursor(8, 42); display.printf("Pump:   %s", pumpRunning   ? "ON " : "off");
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
            display.setCursor(8, 36); display.print(modeAuto ? "Auto-watering ON" : "Manual mode");
            display.setCursor(8, 50); display.print("Hold D9 to toggle");
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
//  AUTO WATERING WITH HYSTERESIS + MIN ON TIME
// ─────────────────────────────────────────────
const int SOIL_ON_THRESH  = 30;  // turn pump ON below this
const int SOIL_OFF_THRESH = 40;  // turn pump OFF above this
const unsigned long MIN_PUMP_ON_MS = 3000; // minimum ON time

void autoWateringLogic() {

    unsigned long now = millis();

    // If pump is currently running, enforce minimum ON time
    if (pumpRunning) {
        if (now - pumpStartMs < MIN_PUMP_ON_MS) {
            return; // ignore OFF logic until minimum ON time passes
        }

        // Only turn off if soil is comfortably above the OFF threshold
        if (soilPct > SOIL_OFF_THRESH) {
            Serial.printf("[AUTO] Soil recovered (%d%%) -> stopping pump\n", soilPct);
            stopPump();
        }
        return;
    }

    // Pump is OFF — check cooldown
    if (now - pumpStopMs < PUMP_COOLDOWN_MS) {
        return;
    }

    // Turn pump ON only if soil is clearly dry
    if (soilPct < SOIL_ON_THRESH) {
        Serial.printf("[AUTO] Soil dry (%d%%) -> starting pump\n", soilPct);
        startPump();

        if (!alertSentThisCycle) {
            Particle.publish("soil_dry");
            alertSentThisCycle = true;
        }
    } else {
        // Soil is fine — reset alert cycle
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
    digitalWrite(PUMP_PIN, HIGH);
    pumpRunning = true;
    pumpStartMs = millis();
    feedPlantStatus.publish("Pump ON");
    Serial.println("[PUMP] ON");
}

void stopPump() {
    digitalWrite(PUMP_PIN, LOW);
    pumpRunning = false;
    pumpStopMs  = millis();  // cooldown starts here
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
//    Short press < 1 s  -> advance OLED to next page
//    Long press >= 1 s  -> toggle AUTO / MANUAL mode
//
//  MIST button D7:
//    Short press < 1 s  -> manual mist burst
//    Long press >= 1 s  -> manual pump cycle
//
//  Edge detection: watch HIGH->LOW (press) to record time,
//  then LOW->HIGH (release) to measure hold duration.
//  Non-blocking -- loop() keeps running while button is held.
//
// ─────────────────────────────────────────────
void handleModeButton() {
    bool state = digitalRead(MODE_BUTTON_PIN);

    if (lastModeButtonState == HIGH && state == LOW) {
        modeButtonPressMs = millis();   // falling edge: record press time
    }

    if (lastModeButtonState == LOW && state == HIGH) {
        unsigned long held = millis() - modeButtonPressMs;
        if (held > DEBOUNCE_MS) {
            if (held >= 1000) {
                // Long press -> toggle AUTO / MANUAL
                modeAuto = !modeAuto;   // ! flips bool: true->false, false->true
                feedMode.publish(modeAuto ? "AUTO" : "MANUAL");
                currentPage = PAGE_MODE; // Force the OLED to the Mode page
                lastOledMs = millis();   // Reset the timer so it stays here for 5 seconds
                lastPhysicalModeChangeMs = millis();  // start mode-lock window
                Serial.printf("[BTN MODE] -> %s\n", modeAuto ? "AUTO" : "MANUAL");
            } else {
                // Short press -> next OLED page
                advancePage();
                lastOledMs = millis();  // reset auto-advance timer
                Serial.println("[BTN MODE] Next page");
            }
        }
    }
    lastModeButtonState = state;
}

void handleMistButton() {
    bool state = digitalRead(MIST_BUTTON_PIN);

    if (lastMistButtonState == HIGH && state == LOW) {
        mistButtonPressMs = millis();   // falling edge: record press time
    }

    if (lastMistButtonState == LOW && state == HIGH) {
        unsigned long held = millis() - mistButtonPressMs;
        if (held > DEBOUNCE_MS) {
            if (held >= 1000) {
                // Long press -> manual pump cycle
                startPump();
                Serial.println("[BTN MIST] Long press -> pump cycle");
            } else {
                // Short press -> manual mist burst
                startMister();
                Serial.println("[BTN MIST] Short press -> mist burst");
            }
        }
    }
    lastMistButtonState = state;
}

// ─────────────────────────────────────────────
//  MQTT SUBSCRIPTIONS
//
//  readSubscription(10) listens 10ms for incoming messages.
//  Returns a pointer to whichever feed sent a message, or
//  nullptr if nothing arrived. Compared against known feeds.
//
//  Mode-lock: dashboard can't override a recent physical button
//  press for MODE_LOCK_MS milliseconds. Prevents Adafruit IO's
//  retained last-value from fighting the physical button.
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
        // BUG FIX: mode-lock guard now correctly placed here only.
        // Dashboard can't override a recent physical button press.
        if (millis() - lastPhysicalModeChangeMs < MODE_LOCK_MS) return;
        String val = String((char*)subModeToggle.lastread);
        modeAuto = (val == "1" || val.equalsIgnoreCase("AUTO")|| val.equalsIgnoreCase("ON"));
        feedMode.publish(modeAuto ? "AUTO" : "MANUAL");
        Serial.printf("[SUB] modetoggle -> %s\n", val.c_str());
    }
}

// ─────────────────────────────────────────────
//  PUBLISH TO ADAFRUIT IO  (every 30 min)
//
//  feedPlantStatus carries a rich human-readable summary string --
//  one feed tells the whole story on the dashboard.
//  snprintf writes formatted text into a char array (buf),
//  like Serial.printf but into memory instead of the serial port.
//  delay(150) respects Adafruit IO free-tier rate limits.
// ─────────────────────────────────────────────
void publishToAdafruitIO() {
    if (!mqtt.connected()) return;

    feedTemperature.publish(tempF);                       delay(150);
    feedHumidity.publish(humidRH);                        delay(150);
    feedAirReading.publish(airReading);                   delay(150);
    feedMode.publish(modeAuto ? "AUTO" : "MANUAL");       delay(150);

    // Rich summary string: one feed, full picture
    char buf[80];
    snprintf(buf, sizeof(buf),
             "Soil:%s %d%%  Temp:%.1fF  Hum:%.0f%%  Air:%s",
             (soilPct < SOIL_WATER_THRESH) ? "DRY" : "OK",
             soilPct, tempF, humidRH,
             getAirQualityLabel(airQuality).c_str());
    feedPlantStatus.publish(buf);

    Serial.println("[MQTT] Published all feeds.");
}

// ─────────────────────────────────────────────
//  MQTT CONNECT / PING
//
//  MQTT_connect(): 3-try limit -- more robust than "try once"
//  but doesn't hang forever if the router is down.
//  After 3 failures it moves on; next loop() call tries again.
//  This is the sweet spot between your "if" and a blocking "while".
//
//  MQTT_ping(): keepalive every 2 min. 'static' means lastPing
//  remembers its value between calls -- not reset each time.
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
        Serial.println("Pinging MQTT...");
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
//  NEOPIXEL ANIMATION
//
//  Priority: mist sparkle > pump chase > manual gold
//            > dry red pulse > idle rainbow breathe
//
//  Sine wave brightness creates the "breathing" effect:
//    sin() returns -1.0 to +1.0
//    x 20 + 30 gives range 10 to 50 (gentle idle breathe)
//    x 60 + 80 gives range 20 to 140 (stronger dry pulse)
//  Same concept as: brightness = 127.5 * sin(2*PI*f*t) + 127.5
//  just scaled to a smaller range.
//
//  Idle rainbow extracts R,G,B from packed uint32_t color:
//    >> 16 shifts red byte into position, & 0xFF isolates it
//    >> 8  shifts green, & 0xFF isolates it
//    & 0xFF isolates blue in place
//  Then each channel is scaled by brightness/255.
//  No ColorHSV needed -- compatible with all library versions.
// ─────────────────────────────────────────────
void updateNeoPixel() {
    static uint8_t  step = 0;
    static uint16_t hue  = 0;
    pixel.clear();

    if (misterRunning) {
        // Cyan sparkle -- different pixels light each tick
        for (int i = 0; i < PIXEL_COUNT; i++) {
            pixel.setPixelColor((i + step) % PIXEL_COUNT,
                                pixel.Color(0, 160 + (step * 3 % 60), 180));
        }
        step = (step + 3) % PIXEL_COUNT;

    } else if (pumpRunning) {
        // Green 4-pixel chase around the ring
        for (int i = 0; i < 4; i++) {
            pixel.setPixelColor((step + i) % PIXEL_COUNT,
                                pixel.Color(0, 200, 60));
        }
        step = (step + 1) % PIXEL_COUNT;

    } else if (!modeAuto) {
        // Solid warm gold -- manual mode indicator
        for (int i = 0; i < PIXEL_COUNT; i++) {
            pixel.setPixelColor(i, pixel.Color(180, 140, 0));
        }

    } else if (soilPct < SOIL_WATER_THRESH) {
        // Red slow pulse -- dry alert
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

void drawDisplay(String timeOnly) {
  display.clearDisplay(); // Clear the display buffer
  display.setTextSize(1); // Set text size to 1 (6px high)
  display.setTextColor(WHITE); // Set text color to white
  display.setCursor(0,0); // Set cursor to top-left corner
  display.println("Moisture Level:"); // Print label for moisture level
  display.setCursor(0,10); // Move cursor down to next line
  display.print(soilPct); // Print moisture level to OLED display
  display.setCursor(0,20); // Move cursor down to next line
  display.print(timeOnly); // Print time and date label
  display.display(); // Update the display with the new moisture level  
}

String timeStamp() {
  //Particle.syncTime(); //Sync time with Particle Cloud
  String dateTime = Time.timeStr(); // Get current date and time as a string
  String timeOnly = dateTime.substring(11,19); // Extract the time from the
  dateTime = Time.timeStr(); //Current Date and Time from Particle Time class
  timeOnly = dateTime.substring(11,19); //Extract the Time from the DateTime String
  return timeOnly; // Return the time as a string for the time-stamp; 
}

// -- End of plantMonitor.cpp --------------------