/* 
 * Project L04_01_neoPixel
 * Author: Nicole
 * Date: 02/18/2026
 * Description:  Using FOR loop anad individual R/G/B, light up 46 pixels (4 individual neoPixels, 1 ring of 12 pixels,
 * and 1 strip of 30 pixels), with a small delaay between.
 */

#include "Particle.h"
#include "neopixel.h" //include neoPixel library

const int PIXELCOUNT =1; //Total number of neoPixels

Adafruit_NeoPixel pixel(PIXELCOUNT, SPI1, WS2812B);


// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(SEMI_AUTOMATIC);

void setup() {
  pixel.begin();
  pixel.setBrightness(100);
  pixel.show();
  }

void loop() {

for (int i=0; i<(PIXELCOUNT); i++) {
  pixel.setPixelColor(i, 0, 200, 100); 
  pixel.show(); 
  delay(200); //set a delay between each pixel lighting up  
    }
pixel.clear();
pixel.show();

/* //Then, Have pixels light up faster
for (int i=0; i<(PIXELCOUNT); i++) {
  pixel.setPixelColor(i, 0, 200, 100); 
  pixel.show(); 
  delay(100); //set a delay between each pixel lighting up  
    }
pixel.clear();
pixel.show();

//Then, Have pixels light up with no delay
for (int i=0; i<(PIXELCOUNT); i++) {
  pixel.setPixelColor(i, 0, 200, 100); 
  pixel.show(); 
    }
delay(2000); //set a delay to hold how the pixels light up before clearing them     
pixel.clear();
pixel.show();
 */
//for (int i=45; i<PIXELCOUNT; i--) {   //How to have pixels light up in reverse order?
  //pixel.setPixelColor(i, 0, 200, 100); 
  //pixel.show(); 
  //delay(350); //set a delay between each pixel lighting up  
    }


