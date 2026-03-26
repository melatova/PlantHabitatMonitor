# 🌿 PlantHabitatMonitor
> **The Electro-Natural Plant Tech Sensor Menagerie**

[![Platform](https://img.shields.io)](https://docs.particle.io)
[![Project](https://img.shields.io)](https://www.hackster.io)

---

### 🍃 Smart Tillandsia Care 
**A Multi‑Tiered IoT Air Plant Habitat**

```text
       [  plant  ]
            │
            │
    [sensors + display]
            │
            │
    [ water reservoir ]
    └─────────────────┘



A compact, slightly over‑engineered IoT habitat designed top  
Tillandsia (air plants) alive — despite my historically poor track record.

Built in five and a half days as part of a CNM IoT assignment,  
this project combines environmental sensing, automatic misting,  
and cloud monitoring in a sculptural glass enclosure.

* * * * * * * * * * * * * * * *


Features

  * Environmental monitoring**
    * Temperature & humidity (BME280)
    * Air quality sensing (analog gas sensor)
    * Capacitive moisture detection
    * Water reservoir level sensing

  * Automated plant care**
    * Piezo atomizing mist system
    * Water pump with automatic and manual control

  * Visual feedback**
    * NeoPixel status lighting
    * OLED system display with multiple pages

  * Cloud integration**
    * Live data publishing
    * Remote manual control via Adafruit IO dashboard

  * Modular physical design**
    * Multi‑tier vertical architecture
    * Custom laser‑cut and 3D‑printed structural components
    * Designed to live inside a glass hurricane vase

······························

System Architecture

Main loop structure:

loop()
├── readSensors()
├── updateButtons()
├── evaluateTriggers()
├── controlPumpAndMister()
├── updateNeoPixels()
├── updateOLED()
└── publishCloudData()

The system supports both automatic and manual operation modes,  
with local controls mirrored in the cloud dashboard.

······························

Hardware

  * Particle Photon 2**
  * BME280 temperature / humidity sensor**
  * Analog air quality sensor**
  * Capacitive moisture sensor**
  * DIY silver‑wire water level probes**
  * Piezo atomizer (TD‑002) + disc**
  * Water pump**
  * NeoPixel LED rings**
  * SSD1306 OLED display (I²C)**
  * MOSFET‑based power switching (post‑redesign)**

······························

Pinout (Core Signals)

| Component                | Pin |
|--------------------------|-----|
| OLED SDA / I²C           | D0  |
| OLED SCL / I²C           | D1  |
| NeoPixel Data            | D2  |
| Pump MOSFET Gate         | D4  |
| Mist MOSFET Gate         | D5  |
| Mist Button              | D7  |
| Mode Button              | D9  |
| Soil / Moisture Sensor   | A0  |
| Air Quality Sensor       | A1  |
| Reservoir Probe (ADC)    | A2  |

······························

Design Challenges & Lessons

  * Hardware that works on a home desk doesn’t always work at your away desk
  * Current draw issues revealed limitations of relay‑based switching
  * Redesigning the power stage with MOSFETs improved reliability
  * Wire routing and noise isolation matter in mixed‑signal systems
  * Iteration is unavoidable — and educational

······························

Piezo Atomizer Notes

The misting system uses a TD‑002 piezo atomizer module  
driven via a MOSFET rather than a relay due to current draw spikes.

Key considerations:
  * High‑frequency operation (~110 kHz)
  * 5V supply with adequate current headroom
  * Physical isolation from I²C wiring to reduce interference

Close‑up imagery and test videos are included in the project gallery.

······························

Gallery

https://youtu.be/lMnhnqs0L-Q


System Planning  
(images/planning_circuit_layer_arrangment.png)

Breadboard Prototyping  
(images/breadboard_testing.png)

MOSFET Redesign  
(images/changing_from_relays_to_MOSFETS.png)

Piezo Atomizer Detail  
(images/piezo_atomizer_plate.png)

Cloud Dashboard  
(images/adafruit_Dashboard.png)

Final Habitat  (to be updated)
(images/Finished_gravel_dish_for_air_plants.png)

······························

Files

/src        — firmware  
/images     — photos + macros  
/docs       — diagrams + notes  
/mechanical — laser‑cut + printed parts  

······························

Why This Exists

This project exists because air plants are beautiful, fragile,  
and completely unwilling to tolerate guesswork.

Rather than give up, I decided to give them sensors, firmware,  
and a cloud dashboard — and see what happened.

So far: the plants are alive.

······························

License

MIT License — enjoy, remix, and keep plants alive thoughtfully 🌿


