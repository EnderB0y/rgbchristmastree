/* Copyright (c) 2021, François Legrand, flegrand80(at)gmail(dot)com.
 *
 * All Rights Reserved,
 *
 * This file is licensed to you under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

 /* Dependencies:
  * Dans l'arduino IDE, il faut ajouter le gestionnaire de platforme (board manager) suivant:
  * https://dl.espressif.com/dl/package_esp32_index.json
  * Ensuite vous serez en mesure d'ajouter le module ESP32
  * https://github.com/me-no-dev/AsyncTCP/archive/master.zip
  *
  * https://github.com/me-no-dev/ESPAsyncWebServer/archive/master.zip
  * https://arduinojson.org/
  * https://github.com/FastLED/FastLED/archive/master.zip
  * https://github.com/samguyer/FastLED
  */


#include <ESPmDNS.h>
#define FASTLED_ESP32_I2S true
#include <FastLED.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include <Update.h>
#include "AsyncJson.h"
#include <ArduinoJson.h>
#include "time.h"
#include "esp_log.h"
#include "FS.h"


#define BUTTON_DEBOUNCE_MS 250
#define COLOR_CORRECTION TypicalSMD5050  //mini
  //#define COLOR_CORRECTION TypicalLEDStrip  //plywood
  //#define COLOR_CORRECTION CRGB(193,225,255)  //WS2811/T8 RGB LED from BTF Lighting

#define VERSION 0.51 //software version
#define MAX_NUM_LEDS 255

#define FIRE_EFFECT_COOLING  55 //used by fire effect
#define FIRE_EFFECT_SPARKING 120 //used by fire effect
#define FIRE_EFFECT_REVERSE_DIRECTION false //used by fire effect to send from which end of the pixel string the fire is starting.

#define _LINEARBLEND 0
#define _NOBLEND 1
#define SOURCE 2
#define SCALED 3
#define RANDOM 4


#define NONE 0
#define ROTATION 1
#define BLINKING 2
#define CONFETTI 3
#define BOUNCE 4
#define RAY 5


const char* filename = "/config.jsn"; // SD library uses 8.3 filenames
struct Config {
   char wifi_hostname[64];
   char wifi_ssid[64];
   char wifi_password[64];
   char ntp_server1address[64];
   char ntp_server2address[64];
   char ntp_server3address[64];
   char time_tzInfoString[64];
   char hw_ledType[8];
   uint16_t hw_ledPin;
   uint8_t hw_numLeds;
	uint8_t hw_numStrings;
   uint8_t hw_programButtonPin;
   uint8_t hw_brightnessButtonPin;
   uint8_t hw_initialBrightness;
   uint16_t hw_maxAmp;
   uint8_t hw_colCorrect_red;
   uint8_t hw_colCorrect_green;
   uint8_t hw_colCorrect_blue;
   uint16_t currentBrightness;
   uint8_t currentProgram;

	char pp1_name[32];
	uint8_t pp1_animType; 
	uint8_t pp1_animSpeed;
	uint8_t pp1_fillMode;
	bool pp1_glitter;
	uint8_t pp1_chanceOfGlitter;
	uint8_t pp1_fadeAmount;
	uint8_t pp1_pallette;
	uint32_t pp1_color0;
	uint32_t pp1_color1;
	uint32_t pp1_color2;
	uint32_t pp1_color3;
	uint32_t pp1_color4;
	uint32_t pp1_color5;
	uint32_t pp1_color6;
	uint32_t pp1_color7;

	bool pp1_reverseDirecton;
	
};
Config config;


//size_t content_len; //Size of firmware/spiff file being uploaded
time_t rawtime;
time_t lastNTPUpdate;
time_t firstNTPUpdate = 0;
long ntpTimeDiff = 0;
struct tm* timeinfo;

#define TRY_WIFI_STATION_MODE true //Usefull during debug, will try to connect to wifi is set to true (station mode)

uint8_t gStartIndex = 0; //global variable used to allow rotation effects between multiple pixels.
uint8_t gColorIndex = 0; //global variable used to allow sequential colors from pallette
uint8_t gHue = 0;
uint8_t gCompleteCycle = 0;
uint8_t gFramesPerSecond = 60;

char hexstring[7];
bool updateNTPRequired = false;
bool rebootRequired = false;
bool saveConfigRequired = false;
StaticJsonDocument<1024> requestBody;
DeserializationError deserializationError;

CRGBPalette16 gCurrentPalette; //Palette de couleur présentement utilisée
TBlendType    gCurrentBlending; //Type d'interpolation présentement en usage

CRGB leds[MAX_NUM_LEDS]; //Pixel string color data
bool disableLoop = false;

struct Button {
   bool pressed;
   unsigned long lastDebounceTime;  // the last time the output pin was toggled
   unsigned long debounceDelay;
};

Button programButton = { false, 0, BUTTON_DEBOUNCE_MS };
Button brightnessButton = { false, 0, BUTTON_DEBOUNCE_MS };
//char *hexstring = "abcdef0";

//Prototypes
String processor(const String& var);
unsigned long xx_time_get_time();
void updateNTP();
void loadConfiguration(const char* filename, Config& config);
void loadConfiguration();
void saveConfiguration(const char* filename, const Config& config);
void printFile(const char* filename);
String processor(const String& var);
void handleDoUpdate(AsyncWebServerRequest* request, const String& filename, size_t index, uint8_t* data, size_t len, bool final);
void printProgress(size_t prg, size_t sz);
void printLocalTime();
void spiffsInit();
void wifiInit();
void webServerInit();




void setupArduinoOTA();

void IRAM_ATTR brightnessIsr();
void IRAM_ATTR programIsr();

DNSServer dnsServer;
AsyncWebServer server(80);
//AsyncWebSocket ws("/ws");
//AsyncEventSource events("/events");




/* setup function */
void setup(void) {
   // Serial port for debugging purposes
   Serial.begin(115200);
   Serial.print("Booting ");
   Serial.println(__FILE__);
   Serial.println(__DATE__ " " __TIME__);

   loadConfiguration();
   wifiInit();
   webServerInit();
   arduinoOTAInit();
  

	if(config.hw_programButtonPin != 0){
	   pinMode(config.hw_programButtonPin, INPUT_PULLUP);
	   attachInterrupt(config.hw_programButtonPin, programIsr, FALLING);	
	}

	if(config.hw_brightnessButtonPin != 0){
	   pinMode(config.hw_brightnessButtonPin, INPUT_PULLUP);
	   attachInterrupt(config.hw_brightnessButtonPin, brightnessIsr, FALLING);
   }

   if (strcmp(config.hw_ledType, "APA106") == 0) {
      if (config.hw_ledPin == 23)
         FastLED.addLeds<APA106, 23, RGB>(leds, config.hw_numLeds).setCorrection(TypicalSMD5050);
      else if (config.hw_ledPin == 25)
         FastLED.addLeds<APA106, 25, RGB>(leds, config.hw_numLeds).setCorrection(TypicalSMD5050);
      else if (config.hw_ledPin == 26)
         FastLED.addLeds<APA106, 26, RGB>(leds, config.hw_numLeds).setCorrection(TypicalSMD5050);
      else if (config.hw_ledPin == 2526){
         FastLED.addLeds<APA106, 25, RGB>(leds, config.hw_numLeds).setCorrection(TypicalSMD5050);
         FastLED.addLeds<APA106, 26, RGB>(leds, config.hw_numLeds).setCorrection(TypicalSMD5050);     
      }
   }
   else if (strcmp(config.hw_ledType, "WS2811") == 0) {

      if (config.hw_ledPin == 23)
         FastLED.addLeds<WS2811, 23, RGB>(leds, config.hw_numLeds).setCorrection(TypicalSMD5050);
      else if (config.hw_ledPin == 25)
         FastLED.addLeds<WS2811, 25, RGB>(leds, config.hw_numLeds).setCorrection(TypicalSMD5050);
      else if (config.hw_ledPin == 26)
         FastLED.addLeds<WS2811, 26, RGB>(leds, config.hw_numLeds).setCorrection(TypicalSMD5050);
   	else if (config.hw_ledPin == 2526){
         FastLED.addLeds<WS2811, 25, RGB>(leds, config.hw_numLeds).setCorrection(TypicalSMD5050);
         FastLED.addLeds<WS2811, 26, RGB>(leds, config.hw_numLeds).setCorrection(TypicalSMD5050);
   	}   
   }
   else if (strcmp(config.hw_ledType, "WS2812B") == 0) {
      if (config.hw_ledPin == 23)
         FastLED.addLeds<WS2812B, 23, GRB>(leds, config.hw_numLeds).setCorrection(TypicalSMD5050);
      else if (config.hw_ledPin == 25)
         FastLED.addLeds<WS2812B, 25, GRB>(leds, config.hw_numLeds).setCorrection(TypicalSMD5050);
      else if (config.hw_ledPin == 26)
         FastLED.addLeds<WS2812B, 26, GRB>(leds, config.hw_numLeds).setCorrection(TypicalSMD5050);
   	else if (config.hw_ledPin == 2526){
         FastLED.addLeds<WS2812B, 25, RGB>(leds, config.hw_numLeds).setCorrection(TypicalSMD5050);
         FastLED.addLeds<WS2812B, 26, RGB>(leds, config.hw_numLeds).setCorrection(TypicalSMD5050);
   	}
   }

   FastLED.setBrightness(config.hw_initialBrightness);
   FastLED.setCorrection(CRGB(config.hw_colCorrect_red, config.hw_colCorrect_green, config.hw_colCorrect_blue));
   set_max_power_in_volts_and_milliamps(5, (uint16_t)config.hw_maxAmp/config.hw_numStrings);

   programButton.pressed = true; //required to start first sequence
}




void loop(void) {
   ArduinoOTA.handle();
   //ws.cleanupClients();
   dnsServer.processNextRequest();


   if (!disableLoop) {
      random16_add_entropy(esp_random());
      gHue++; //used for rainbow
      gStartIndex = gStartIndex + 1; // motion speed


      if (brightnessButton.pressed) {
         Serial.printf("brightnessButton pressed, value = %u\n", config.currentBrightness);
         FastLED.setBrightness(config.currentBrightness);
         brightnessButton.pressed = false;
      }

      switch (config.currentProgram) {
      case 0:
         if (programButton.pressed) {
            Serial.printf("programButton pressed, value = %u\n", config.currentProgram);
            programButton.pressed = false;
            fadeLedsToBlack();
            pattern0(true);
         }
         else
            pattern0(false);
         break;
      case 1:
         if (programButton.pressed) {
            Serial.printf("programButton pressed, value = %u\n", config.currentProgram);
            programButton.pressed = false;
            fadeLedsToBlack();
            pattern1(true);
         }
         else
            pattern1(false);
         break;
      case 2:
         if (programButton.pressed) {
            Serial.printf("programButton pressed, value = %u\n", config.currentProgram);
            programButton.pressed = false;
            fadeLedsToBlack();
            pattern2(true);
         }
         else
            pattern2(false);
         break;
      case 3:
         if (programButton.pressed) {
            Serial.printf("programButton pressed, value = %u\n", config.currentProgram);
            programButton.pressed = false;
            fadeLedsToBlack();
            pattern3(true);
         }
         else
            pattern3(false);
         break;
      case 4:
         if (programButton.pressed) {
            Serial.printf("programButton pressed, value = %u\n", config.currentProgram);
            programButton.pressed = false;
            fadeLedsToBlack();
            pattern4(true);
         }
         else
            pattern4(false);
         break;
      case 5:
         if (programButton.pressed) {
            Serial.printf("programButton pressed, value = %u\n", config.currentProgram);
            programButton.pressed = false;
            fadeLedsToBlack();
            pattern5(true);
         }
         else
            pattern5(false);
         break;
      case 6:
         if (programButton.pressed) {
            Serial.printf("programButton pressed, value = %u\n", config.currentProgram);
            programButton.pressed = false;
            fadeLedsToBlack();
            pattern6(true);
         }
         else
            pattern6(false);
         break;
      case 7:
         if (programButton.pressed) {
            Serial.printf("programButton pressed, value = %u\n", config.currentProgram);
            programButton.pressed = false;
            fadeLedsToBlack();
            pattern7(true);
         }
         else
            pattern7(false);
         break;
      case 8:
         if (programButton.pressed) {
            Serial.printf("programButton pressed, value = %u\n", config.currentProgram);
            programButton.pressed = false;
            fadeLedsToBlack();
            pattern8(true);
         }
         else
            pattern8(false);
         break;
      case 9:
         if (programButton.pressed) {
            Serial.printf("programButton pressed, value = %u\n", config.currentProgram);
            programButton.pressed = false;
            fadeLedsToBlack();
            pattern9(true);
         }
         else
            pattern9(false);
         break;
      case 10:
         if (programButton.pressed) {
            Serial.printf("programButton pressed, value = %u\n", config.currentProgram);
            programButton.pressed = false;
            fadeLedsToBlack();
            pattern10(true);
         }
            pattern10(false);
         break;
      case 11:
         if (programButton.pressed) {
            Serial.printf("programButton pressed, value = %u\n", config.currentProgram);
            programButton.pressed = false;
            fadeLedsToBlack();
            pattern11(true);
         }
         else
            pattern11(false);
         break;
      case 12:
         if (programButton.pressed) {
            Serial.printf("programButton pressed, value = %u\n", config.currentProgram);
            programButton.pressed = false;
            fadeLedsToBlack();
            pattern12(true);
         }
         else
            pattern12(false);
         break;
      default:
         config.currentProgram = 0;
      }

   	FastLED.delay(1000 / gFramesPerSecond);
   
   }else{
      delay(100);
   }
   



}


void pattern0(bool firstRun) {
	//stop
   if (firstRun) {
      fill_solid(gCurrentPalette, 16, CRGB::Black);
      gStartIndex = 0;
      gFramesPerSecond = 1;
      FillLEDsFromPaletteColors(gStartIndex);
      FastLED.show();
   }
}


void pattern1(bool firstRun) {
   if (firstRun) {
      
      if(config.pp1_pallette==0){
	      gCurrentPalette = CRGBPalette16(
	      	config.pp1_color0, config.pp1_color1, config.pp1_color2, config.pp1_color3,
				config.pp1_color4, config.pp1_color5, config.pp1_color6, config.pp1_color7,
				config.pp1_color0, config.pp1_color1, config.pp1_color2, config.pp1_color3,
	      	config.pp1_color4, config.pp1_color5, config.pp1_color6, config.pp1_color7);      	
      }else if(config.pp1_pallette==1){
      	gCurrentPalette = CloudColors_p;
      }else if(config.pp1_pallette==2){
      	gCurrentPalette = LavaColors_p;
      }else if(config.pp1_pallette==3){
      	gCurrentPalette = OceanColors_p;      
      }else if(config.pp1_pallette==4){
      	gCurrentPalette = ForestColors_p;      
      }else if(config.pp1_pallette==5){
      	gCurrentPalette = RainbowColors_p;
      }else if(config.pp1_pallette==6){
      	gCurrentPalette = RainbowStripeColors_p;
      }else if(config.pp1_pallette==7){
      	gCurrentPalette = PartyColors_p;      
      }else if(config.pp1_pallette==8){
      	gCurrentPalette = HeatColors_p;              
      }else if(config.pp1_pallette==9){
	      gCurrentPalette = CRGBPalette16(
	      	CRGB::Blue, CRGB::White, CRGB::Red, CRGB::White,
	      	CRGB::Blue, CRGB::White, CRGB::Red, CRGB::White,
	      	CRGB::Blue, CRGB::White, CRGB::Red, CRGB::White,
	      	CRGB::Blue, CRGB::White, CRGB::Red, CRGB::White); 
      }else if(config.pp1_pallette==10){
	      gCurrentPalette = CRGBPalette16(
	      	CRGB::Red, CRGB::White, CRGB::Green, CRGB::White,
	      	CRGB::Red, CRGB::White, CRGB::Green, CRGB::White,
	      	CRGB::Red, CRGB::White, CRGB::Green, CRGB::White,
	      	CRGB::Red, CRGB::White, CRGB::Green, CRGB::White);       
      }
      	
      gStartIndex = 0;
      gFramesPerSecond = config.pp1_animSpeed;

		if(config.pp1_animType == NONE){
		   if (config.pp1_fillMode == _LINEARBLEND)
		   	FillLEDsFromPaletteColors(gStartIndex, _LINEARBLEND);
			else if (config.pp1_fillMode == _NOBLEND)
		   	FillLEDsFromPaletteColors(gStartIndex, _NOBLEND);	
			else if (config.pp1_fillMode == SOURCE)
		   	FillLEDsFromPaletteColors(gStartIndex, SOURCE);		
			else if (config.pp1_fillMode == SCALED)
		   	FillLEDsFromPaletteColors(gStartIndex, SCALED);				
			else if (config.pp1_fillMode == RANDOM)
		   	FillLEDsFromPaletteColors(gStartIndex, RANDOM); 						
		}else if (config.pp1_animType == RAY){
			fillLedsWithBlack();
		}

		
   }else{
		if(config.pp1_animType == ROTATION){
		   if (config.pp1_fillMode == _LINEARBLEND)
		   	FillLEDsFromPaletteColors(gStartIndex, _LINEARBLEND);
			else if (config.pp1_fillMode == _NOBLEND)
		   	FillLEDsFromPaletteColors(gStartIndex, _NOBLEND);	
			else if (config.pp1_fillMode == SOURCE)
		   	FillLEDsFromPaletteColors(gStartIndex, SOURCE);		
			else if (config.pp1_fillMode == SCALED)
		   	FillLEDsFromPaletteColors(gStartIndex, SCALED);				
			else if (config.pp1_fillMode == RANDOM)
		   	FillLEDsFromPaletteColors(gStartIndex, RANDOM); 		 
		}else if(config.pp1_animType == BLINKING){   
		   if(gStartIndex<=127){ 
			   if (config.pp1_fillMode == _LINEARBLEND)
			   	FillLEDsFromPaletteColors(0, _LINEARBLEND);
				else if (config.pp1_fillMode == _NOBLEND)
			   	FillLEDsFromPaletteColors(0, _NOBLEND);	
				else if (config.pp1_fillMode == SOURCE)
			   	FillLEDsFromPaletteColors(0, SOURCE);		
				else if (config.pp1_fillMode == SCALED)
			   	FillLEDsFromPaletteColors(0, SCALED);				
				else if (config.pp1_fillMode == RANDOM)
			   	FillLEDsFromPaletteColors(0, RANDOM); 	   
			}else{
				fillLedsWithBlack();
			}
		}else if (config.pp1_animType == CONFETTI){	
		   fadeToBlackBy(leds, config.hw_numLeds, config.pp1_fadeAmount);
		   int pos = random16(config.hw_numLeds);
		   //to do add confetti multiplier
		   leds[pos] = solidRGBColorFromPalette16(random8(15));	
		}else if (config.pp1_animType == BOUNCE){





			
		}else if (config.pp1_animType == RAY){	
		   fadeToBlackBy(leds, config.hw_numLeds, config.pp1_fadeAmount);	   
		   leds[gStartIndex] = solidRGBColorFromPalette16(gColorIndex);	
			if(gStartIndex >= config.hw_numLeds){
				gStartIndex = 0;		
				if(gColorIndex >= 15)
					gColorIndex = 0;
				else
					gColorIndex ++;
			}

		}   	
   	
   	
   if(config.pp1_glitter)
   	addGlitter(config.pp1_chanceOfGlitter);
   }
	FastLED.show();
}


void pattern2(bool firstRun) {
	//fire
   if (firstRun) {
      gCurrentPalette = HeatColors_p;

      gCurrentBlending = LINEARBLEND;
      gStartIndex = 0;
      gFramesPerSecond = 120;
   }

   Fire2012WithPalette();
   FastLED.show();
}


void pattern3(bool firstRun) {
	//Jungle
   if (firstRun) {
      gFramesPerSecond = 60;
   }

   juggle();

   FastLED.show();
}


void pattern4(bool firstRun) {
	//tempo
   if (firstRun) {
      gFramesPerSecond = 60;
   }

   bpm();

   FastLED.show();
}

/*
void pattern5(bool firstRun) {
	//serpent
   if (firstRun) {
      gFramesPerSecond = 60;
   }

   sinelon();

   FastLED.show();
}

*/



void pattern5(bool firstRun) {
  if (firstRun) {
      
	      gCurrentPalette = CRGBPalette16(
	      	CRGB::Purple, CRGB::White, CRGB::Blue, CRGB::White,
	      	CRGB::Pink, CRGB::White, CRGB::Cyan, CRGB::White,
	      	CRGB::Purple, CRGB::White, CRGB::Blue, CRGB::White,
	      	CRGB::Pink, CRGB::White, CRGB::Cyan, CRGB::White); 
     
      	
      gStartIndex = 0;
      gFramesPerSecond = 120;
		fillLedsWithBlack();

		
   }else{
		   fadeToBlackBy(leds, config.hw_numLeds, 2);	   
		   leds[gStartIndex] = solidRGBColorFromPalette16(gColorIndex);	
			if(gStartIndex >= config.hw_numLeds){
				gStartIndex = 0;		
				if(gColorIndex >= 15)
					gColorIndex = 0;
				else
					gColorIndex ++;
			}
   }   	
	FastLED.show();
}



















void pattern6(bool firstRun) {
	//confetti
   if (firstRun) {
      gCurrentBlending = LINEARBLEND;
      gStartIndex = 0;
      gFramesPerSecond = 120;
   }

   confetti();
   FastLED.show();
}


void pattern7(bool firstRun) {
	//smooth rainbow
   if (firstRun) {
      gCurrentPalette = RainbowColors_p;

      gCurrentBlending = LINEARBLEND;
      gStartIndex = 0;
      gFramesPerSecond = 120;
   }

   FillLEDsFromPaletteColors(gStartIndex);
   FastLED.show();
}


void pattern8(bool firstRun) {
	//stripped rainbow
   if (firstRun) {
      gCurrentPalette = RainbowStripeColors_p;
      gCurrentBlending = LINEARBLEND;
      gStartIndex = 0;
      gFramesPerSecond = 45;
   }

   FillLEDsFromPaletteColors(gStartIndex);
   FastLED.show();
}


void pattern9(bool firstRun) {
	   if (firstRun) {
      gCurrentPalette = CRGBPalette16(CRGB::Blue, CRGB::Blue, CRGB::Blue, CRGB::Blue,
         CRGB::White, CRGB::White, CRGB::White, CRGB::White,
         CRGB::Blue, CRGB::Blue, CRGB::Blue, CRGB::Blue,
         CRGB::White, CRGB::White, CRGB::White, CRGB::White);

      //CRGB::White or 0xFFFFFF or CRGB(255,255,255) or CHSV(0,0,255) are all equivalent

      gCurrentBlending = LINEARBLEND;
      gStartIndex = 0;
      gFramesPerSecond = 120;
   }

   FillLEDsFromPaletteColors(gStartIndex);
   FastLED.show();
	
}


void pattern10(bool firstRun) {
   if (firstRun) {
      // FastLED provides several 'preset' palettes: RainbowColors_p, RainbowStripeColors_p,
      // OceanColors_p, CloudColors_p, LavaColors_p, ForestColors_p, and PartyColors_p.
      gCurrentPalette = ForestColors_p;

      gCurrentBlending = LINEARBLEND;
      gStartIndex = 0;
      gFramesPerSecond = 120;
   }

   FillLEDsFromPaletteColors(gStartIndex);
   FastLED.show();
}


void pattern11(bool firstRun) {
   if (firstRun) {
         // 'black out' all 16 palette entries...
   fill_solid(gCurrentPalette, 16, CRGB::Black);
   // and set every fourth one to white.
   gCurrentPalette[0] = CRGB::White;
   gCurrentPalette[4] = CRGB::White;
   gCurrentPalette[8] = CRGB::White;
   gCurrentPalette[12] = CRGB::White;

      gCurrentBlending = LINEARBLEND;
      gStartIndex = 0;
      gFramesPerSecond = 120;
   }

   FillLEDsFromPaletteColors(gStartIndex);
   FastLED.show();
}


void pattern12(bool firstRun) {
   if (firstRun) {
      fill_solid(gCurrentPalette, 16, CRGB::White);
      gStartIndex = 0;
      gFramesPerSecond = 1;
      FillLEDsFromPaletteColors(gStartIndex);
      FastLED.show();
   }
}


CRGB solidRGBColorFromPalette16(uint8_t index){
	return ColorFromPalette(gCurrentPalette, index*16, 255, NOBLEND);
}


void FillLEDsFromPaletteColors(uint8_t colorIndex) {
	   for (int i = 0; i < config.hw_numLeds; i++) {
	      leds[i] = ColorFromPalette(gCurrentPalette, colorIndex, 255, gCurrentBlending);
	      colorIndex += 1;
	   }  
}


void FillLEDsFromPaletteColors(uint8_t colorIndex, uint8_t fillType) {
	if (fillType==_LINEARBLEND){
	   for (int i = 0; i < config.hw_numLeds; i++) {
	      leds[i] = ColorFromPalette(gCurrentPalette, colorIndex, 255, LINEARBLEND);
	      colorIndex += 1;
	   }
	}else if (fillType==_NOBLEND){
		for (int i = 0; i < config.hw_numLeds; i++) {
	      leds[i] = ColorFromPalette(gCurrentPalette, colorIndex, 255, NOBLEND);
	      colorIndex += 1;
		}
	}else if (fillType==SCALED){
	   for (int i = 0; i < config.hw_numLeds; i++) {
	      leds[i] = ColorFromPalette(gCurrentPalette, colorIndex, 255, gCurrentBlending);
	      colorIndex += (uint8_t) 255/config.hw_numLeds;
	   }
	}else if (fillType==SOURCE){
	   for (int i = 0; i < config.hw_numLeds; i++) {
	      leds[i] = solidRGBColorFromPalette16(colorIndex);
	      colorIndex += 1;
	   }
	}else if (fillType==RANDOM){
		for (int i = 0; i < config.hw_numLeds; i++) {
	      leds[i] = ColorFromPalette(gCurrentPalette, random8(), 255, NOBLEND);
	      colorIndex += 1;
		}
	}
}


void addGlitter(fract8 chanceOfGlitter) {
   if (random8() < chanceOfGlitter) {
      leds[random16(config.hw_numLeds)] += CRGB::White;
   }
}


void confetti() {
   // random colored speckles that blink in and fade smoothly
   fadeToBlackBy(leds, config.hw_numLeds, 10);
   int pos = random16(config.hw_numLeds);
   //to do add confetti multiplier
   leds[pos] += CHSV(gHue + random8(64), 200, 255);
}


void juggle() {
   // eight colored dots, weaving in and out of sync with each other
   fadeToBlackBy(leds, config.hw_numLeds, 20);
   byte dothue = 0;
   for (int i = 0; i < 8; i++) {
      leds[beatsin16(i + 7, 0, config.hw_numLeds - 1)] |= CHSV(dothue, 200, 255);
      dothue += 32;
   }
}


void bpm() {
   // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
   uint8_t BeatsPerMinute = 62;
   CRGBPalette16 palette = PartyColors_p;
   uint8_t beat = beatsin8(BeatsPerMinute, 64, 255);
   for (int i = 0; i < config.hw_numLeds; i++) { //9948
      leds[i] = ColorFromPalette(palette, gHue + (i * 2), beat - gHue + (i * 10));
   }
}


void sinelon() {
   // a colored dot sweeping back and forth, with fading trails
   fadeToBlackBy(leds, config.hw_numLeds, 20);
   int pos = beatsin16(13, 0, config.hw_numLeds - 1);
   leds[pos] += CHSV(gHue, 255, 192);
}


void Fire2012WithPalette() {
   // Array of temperature readings at each simulation cell
   static byte heat[MAX_NUM_LEDS];

   // Step 1.  Cool down every cell a little
   for (int i = 0; i < config.hw_numLeds; i++) {
      heat[i] = qsub8(heat[i], random8(0, ((FIRE_EFFECT_COOLING * 10) / config.hw_numLeds) + 2));
   }

   // Step 2.  Heat from each cell drifts 'up' and diffuses a little
   for (int k = config.hw_numLeds - 1; k >= 2; k--) {
      heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
   }

   // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
   if (random8() < FIRE_EFFECT_SPARKING) {
      int y = random8(7);
      heat[y] = qadd8(heat[y], random8(160, 255));
   }

   // Step 4.  Map from heat cells to LED colors
   for (int j = 0; j < config.hw_numLeds; j++) {
      // Scale the heat value from 0-255 down to 0-240
      // for best results with color palettes.
      byte colorindex = scale8(heat[j], 240);
      CRGB color = ColorFromPalette(gCurrentPalette, colorindex);
      int pixelnumber;
      if (FIRE_EFFECT_REVERSE_DIRECTION) {
         pixelnumber = (config.hw_numLeds - 1) - j;
      }
      else {
         pixelnumber = j;
      }
      leds[pixelnumber] = color;
   }
}


unsigned long xx_time_get_time() {
   struct timeval tv;
   gettimeofday(&tv, NULL);
   return (tv.tv_sec * 1000UL + (tv.tv_usec / 1000UL));
}


void updateNTP() {
   unsigned long ntpTimeBeforeUpdate = xx_time_get_time();
   unsigned long runTimeBeforeUpdate = millis();
   unsigned long ntpTimeAfterUpdate = 0;
   unsigned long runTimeAfterUpdate = 0;

   configTzTime(config.time_tzInfoString, config.ntp_server1address, config.ntp_server2address, config.ntp_server3address);
   delay(1000);

   time(&lastNTPUpdate);

   if (firstNTPUpdate == 0) {
      firstNTPUpdate = lastNTPUpdate;
   }

   ntpTimeAfterUpdate = xx_time_get_time();
   runTimeAfterUpdate = millis();
   ntpTimeDiff = ntpTimeAfterUpdate - ntpTimeBeforeUpdate - runTimeAfterUpdate + runTimeBeforeUpdate;

   Serial.printf("NTP update, ESP32 clock drift since last update is estimated to be %d milliseconds\n", ntpTimeDiff);

}


// Loads the configuration from a file
void loadConfiguration(const char* filename, Config& config) {
   // Open file for reading
   File file = SPIFFS.open(filename);

   // Allocate a temporary JsonDocument
   // Don't forget to change the capacity to match your requirements.
   // Use arduinojson.org/v6/assistant to compute the capacity.
   StaticJsonDocument<4096> doc;

   // Deserialize the JSON document
   DeserializationError error = deserializeJson(doc, file);
   if (error)
      Serial.println(F("Failed to read file, using default configuration"));

   // Copy values from the JsonDocument to the Config

   strlcpy(config.wifi_hostname,              // <- destination
      doc["wifi_hostname"] | "sapin", // <- source or default if nothing
      sizeof(config.wifi_hostname));     // <- destination's capacity

   strlcpy(config.wifi_ssid,
      doc["wifi_ssid"] | "default_ssid",
      sizeof(config.wifi_ssid));

   strlcpy(config.wifi_password,
      doc["wifi_password"] | "default_password",
      sizeof(config.wifi_password));

   strlcpy(config.ntp_server1address,
      doc["ntp_server1address"] | "1.ca.pool.ntp.org",
      sizeof(config.ntp_server1address));

   strlcpy(config.ntp_server2address,
      doc["ntp_server2address"] | "2.ca.pool.ntp.org",
      sizeof(config.ntp_server2address));

   strlcpy(config.ntp_server3address,
      doc["ntp_server3address"] | "3.ca.pool.ntp.org",
      sizeof(config.ntp_server3address));

   strlcpy(config.time_tzInfoString,
      doc["time_tzInfoString"] | "EST5EDT4,M3.2.0/02:00:00,M11.1.0/02:00:00",
      sizeof(config.time_tzInfoString));

   strlcpy(config.hw_ledType,
      doc["hw_ledType"] | "WS2812B",
      sizeof(config.hw_ledType));

   config.hw_ledPin = doc["hw_ledPin"] | 23;
   config.hw_numLeds = doc["hw_numLeds"] | 178;
   config.hw_numStrings = doc["hw_numStrings"] | 1; 
   config.hw_programButtonPin = doc["hw_programButtonPin"] | 26;
   config.hw_brightnessButtonPin = doc["hw_brightnessButtonPin"] | 25;
   config.hw_initialBrightness = doc["hw_initialBrightness"] | 15;
   config.hw_maxAmp = doc["hw_maxAmp"] | 500;
   config.hw_brightnessButtonPin = doc["hw_brightnessButtonPin"] | 25;
   config.hw_colCorrect_red = doc["hw_colCorrect_red"] | 255;
   config.hw_colCorrect_green = doc["hw_colCorrect_green"] | 176;
   config.hw_colCorrect_blue = doc["hw_colCorrect_blue"] | 240;
   config.currentBrightness = doc["hw_initialBrightness"] | 15;
   config.currentProgram = doc["currentProgram"] | 1;

	config.pp1_animType = doc["pp1_animType"] | 1;
	config.pp1_animSpeed = doc["pp1_animSpeed"] | 60;
	config.pp1_pallette = doc["pp1_pallette"] | 1;
	config.pp1_fillMode = doc["pp1_fillMode"] | 0;	
	config.pp1_color0 = doc["pp1_color0"] | 0xFF0000;
	config.pp1_color1 = doc["pp1_color1"] | 0xFF0000;
	config.pp1_color2 = doc["pp1_color2"] | 0xFF0000;
	config.pp1_color3 = doc["pp1_color3"] | 0xFF0000;
	config.pp1_color4 = doc["pp1_color4"] | 0xFF0000;
	config.pp1_color5 = doc["pp1_color5"] | 0xFF0000;
	config.pp1_color6 = doc["pp1_color6"] | 0xFF0000;
	config.pp1_color7 = doc["pp1_color7"] | 0xFF0000;
   config.pp1_glitter = doc["pp1_glitter"] | false;
	config.pp1_chanceOfGlitter = doc["pp1_chanceOfGlitter"] | 80;
	config.pp1_fadeAmount = doc["pp1_fadeAmount"]  | 10;
   
   strlcpy(config.pp1_name,
      doc["pp1_name"] | "Animation 1",
      sizeof(config.pp1_name));

  	file.close();
}




void loadConfiguration() {
   spiffsInit();
   //SPIFFS.remove("/config.jsn");
   File root = SPIFFS.open("/");
   File file = root.openNextFile();
   while (file) {
      Serial.print("FILE: ");
      Serial.println(file.name());
      file = root.openNextFile();
   }

   Serial.println(F("Loading configuration..."));
   loadConfiguration(filename, config);
   Serial.println(F("Saving configuration..."));
   saveConfiguration(filename, config);

   Serial.println(F("Print config file..."));
   printFile(filename);
}


// Saves the configuration to a file
void saveConfiguration(const char* filename, const Config& config)
{
   // Delete existing file
   SPIFFS.remove(filename);

   // Open file for writing
   File file = SPIFFS.open(filename, FILE_WRITE);
   if (!file) {
      Serial.println(F("Failed to create file"));
      return;
   }

   // Allocate a temporary JsonDocument
   // Don't forget to change the capacity to match your requirements.
   // Use arduinojson.org/assistant to compute the capacity.
   StaticJsonDocument<2048> doc;

   // Set the values in the document
   doc["wifi_hostname"] = config.wifi_hostname;
   doc["wifi_ssid"] = config.wifi_ssid;
   doc["wifi_password"] = config.wifi_password;
   doc["ntp_server1address"] = config.ntp_server1address;
   doc["ntp_server2address"] = config.ntp_server2address;
   doc["ntp_server3address"] = config.ntp_server3address;
   doc["time_tzInfoString"] = config.time_tzInfoString;
   doc["hw_ledType"] = config.hw_ledType;
   doc["hw_ledPin"] = config.hw_ledPin;
   doc["hw_numLeds"] = config.hw_numLeds;
   doc["hw_numStrings"] = config.hw_numStrings;
   doc["hw_programButtonPin"] = config.hw_programButtonPin;
   doc["hw_brightnessButtonPin"] = config.hw_brightnessButtonPin;
   doc["hw_initialBrightness"] = config.hw_initialBrightness;
   doc["hw_maxAmp"] = config.hw_maxAmp;
   doc["hw_colCorrect_red"] = config.hw_colCorrect_red;
   doc["hw_colCorrect_green"] = config.hw_colCorrect_green;
   doc["hw_colCorrect_blue"] = config.hw_colCorrect_blue;
   doc["currentProgram"] = config.currentProgram;
	doc["pp1_animSpeed"] = config.pp1_animSpeed;
	doc["pp1_pallette"] = config.pp1_pallette;
	doc["pp1_color0"] = config.pp1_color0;
	doc["pp1_color1"] = config.pp1_color1;
	doc["pp1_color2"] = config.pp1_color2;
	doc["pp1_color3"] = config.pp1_color3;
	doc["pp1_color4"] = config.pp1_color4;
	doc["pp1_color5"] = config.pp1_color5;
	doc["pp1_color6"] = config.pp1_color6;
	doc["pp1_color7"] = config.pp1_color7;
   doc["pp1_glitter"] = config.pp1_glitter;
   doc["pp1_name"] = config.pp1_name;
	doc["pp1_animType"] = config.pp1_animType;
	doc["pp1_fillMode"] = config.pp1_fillMode;
	doc["pp1_chanceOfGlitter"] = config.pp1_chanceOfGlitter;
	doc["pp1_fadeAmount"] = config.pp1_fadeAmount;
	
   // Serialize JSON to file
   if (serializeJson(doc, file) == 0) {
      Serial.println(F("Failed to write to file"));
   }

   // Close the file
   file.close();
}


// Prints the content of a file to the Serial
void printFile(const char* filename) {
   // Open file for reading
   File file = SPIFFS.open(filename);
   if (!file)
   {
      Serial.println(F("Failed to read file"));
      return;
   }

   // Extract each characters by one by one
   while (file.available())
   {
      Serial.print((char)file.read());
   }
   Serial.println();

   // Close the file
   file.close();
}


String processor(const String& var) {
   time(&rawtime);
   //Serial.println(var);

   if (var == "currentTimeUTC")
      return String(asctime(gmtime(&rawtime)));
   else if (var == "currentTimeLocal")
      return String(asctime(localtime(&rawtime)));
   else if (var == "lastNTPUpdate")
      return String(asctime(gmtime(&lastNTPUpdate)));
   else if (var == "firstNTPUpdate")
      return String(asctime(gmtime(&firstNTPUpdate)));
   else if (var == "currentIP")
      return WiFi.localIP().toString();
   else if (var == "ntpTimeDiff") {
      if (ntpTimeDiff > 10000)
         return String("unknown");
      else if (ntpTimeDiff == 1 || ntpTimeDiff == -1)
         return String(ntpTimeDiff) + String(" millisecond");
      else
         return String(ntpTimeDiff) + String(" milliseconds");
   }
   else if (var == "firmwareVersion")
      return String(VERSION);
   else if (var == "firmwareCompileTime")
      return String(__DATE__) + String(" ") + String(__TIME__);
   else if (var == "wifi_hostname")
      return config.wifi_hostname;
   else if (var == "wifi_ssid")
      return config.wifi_ssid;
   else if (var == "wifi_password")
      return config.wifi_password;
   else if (var == "ntp_server1address")
      return config.ntp_server1address;
   else if (var == "ntp_server2address")
      return config.ntp_server2address;
   else if (var == "ntp_server3address")
      return config.ntp_server3address;
   else if (var == "time_tzInfoString")
      return String(config.time_tzInfoString);
   else if (var == "hw_ledType_APA106") {
      if (strcmp(config.hw_ledType, "APA106") == 0)
         return "selected";
   }
   else if (var == "hw_ledType_WS2811") {
      if (strcmp(config.hw_ledType, "WS2811") == 0)
         return "selected";
   }
   else if (var == "hw_ledType_WS2812B") {
      if (strcmp(config.hw_ledType, "WS2812B") == 0)
         return "selected";
   }
  	else if (var == "hw_numLeds")
   	return String(config.hw_numLeds);
   	
  	else if (var == "hw_numStrings")
   	return String(config.hw_numStrings);
   	
   else if (var == "currentProgram_00_sel") {
      if (config.currentProgram == 0) return "selected";
   }else if (var == "currentProgram_01_sel") {
      if (config.currentProgram == 1) return "selected";
   }else if (var == "currentProgram_02_sel") {
      if (config.currentProgram == 2) return "selected";
   }else if (var == "currentProgram_03_sel") {
      if (config.currentProgram == 3) return "selected";
   }else if (var == "currentProgram_04_sel") {
      if (config.currentProgram == 4) return "selected";
   }else if (var == "currentProgram_05_sel") {
      if (config.currentProgram == 5) return "selected";
   }else if (var == "currentProgram_06_sel") {
      if (config.currentProgram == 6) return "selected";
   }else if (var == "currentProgram_07_sel") {
      if (config.currentProgram == 7) return "selected";
   }else if (var == "currentProgram_08_sel") {
      if (config.currentProgram == 8) return "selected";
   }else if (var == "currentProgram_09_sel") {
      if (config.currentProgram == 9) return "selected";
   }else if (var == "currentProgram_10_sel") {
      if (config.currentProgram == 10) return "selected";
   }else if (var == "currentProgram_11_sel") {
      if (config.currentProgram == 11) return "selected";
   }else if (var == "currentProgram_12_sel") {
      if (config.currentProgram == 12) return "selected";
   }
      
   else if (var == "hw_maxAmp_500") {
      if (config.hw_maxAmp == 500)
         return "selected";
   }
   else if (var == "hw_maxAmp_1000") {
      if (config.hw_maxAmp == 1000)
         return "selected";
   }
   else if (var == "hw_maxAmp_1500") {
      if (config.hw_maxAmp == 1500)
         return "selected";
   }
   else if (var == "hw_maxAmp_2000") {
      if (config.hw_maxAmp == 2000)
         return "selected";
   }
   else if (var == "hw_maxAmp_2500") {
      if (config.hw_maxAmp == 2500)
         return "selected";
   }
   else if (var == "hw_maxAmp_3000") {
      if (config.hw_maxAmp == 3000)
         return "selected";
   }
   else if (var == "hw_maxAmp_3500") {
      if (config.hw_maxAmp == 3500)
         return "selected";
   }
   else if (var == "hw_maxAmp_4000") {
      if (config.hw_maxAmp == 4000)
         return "selected";
   }
   else if (var == "hw_maxAmp_4500") {
      if (config.hw_maxAmp == 4500)
         return "selected";
   }
   else if (var == "hw_maxAmp_5000") {
      if (config.hw_maxAmp == 5000)
         return "selected";
   } 
   else if (var == "hw_programButtonPin_none"){
		if(config.hw_programButtonPin == 0)
			return "selected";
   }
   else if (var == "hw_programButtonPin_26"){
		if(config.hw_programButtonPin == 26)
			return "selected";
   } 
   else if (var == "hw_brightnessButtonPin_none"){
		if(config.hw_brightnessButtonPin == 0)
			return "selected";
   }
   else if (var == "hw_brightnessButtonPin_25"){
		if(config.hw_brightnessButtonPin == 25)
			return "selected";
   }
   else if (var == "hw_ledPin_23"){
		if(config.hw_ledPin == 23)
			return "selected";
   }
   else if (var == "hw_ledPin_25"){
		if(config.hw_ledPin == 25)
			return "selected";
   }
   else if (var == "hw_ledPin_26"){
		if(config.hw_ledPin == 26)
			return "selected";
   }
   else if (var == "hw_ledPin_2526"){
		if(config.hw_ledPin == 2526)
			return "selected";
   }   
   else if (var == "hw_initialBrightness")
      return String(config.hw_initialBrightness);
   else if (var == "hw_maxAmp")
      return String(config.hw_maxAmp);
   else if (var == "currentBrightness")
      return String(config.currentBrightness);
   else if (var == "hw_colCorrect_red")
      return String(config.hw_colCorrect_red);
   else if (var == "hw_colCorrect_green")
      return String(config.hw_colCorrect_green);
   else if (var == "hw_colCorrect_blue")
      return String(config.hw_colCorrect_blue);
   else if (var == "softwareVersion")
      return " V" + String(VERSION);

   else if (var == "pp1_name")
      return config.pp1_name;
      
   else if (var == "pp1_animType_00_sel"){
		if(config.pp1_animType == NONE) return "selected";
   }else if (var == "pp1_animType_01_sel"){
		if(config.pp1_animType == ROTATION) return "selected";
	}else if (var == "pp1_animType_02_sel"){
		if(config.pp1_animType == BLINKING) return "selected";
   }else if (var == "pp1_animType_03_sel"){
		if(config.pp1_animType == CONFETTI) return "selected";
   }else if (var == "pp1_animType_04_sel"){
		if(config.pp1_animType == BOUNCE) return "selected";
   }else if (var == "pp1_animType_05_sel"){
		if(config.pp1_animType == RAY) return "selected";
   }

   else if (var == "pp1_fillMode_00_sel"){
		if(config.pp1_fillMode == _LINEARBLEND) return "selected";
   }else if (var == "pp1_fillMode_01_sel"){
		if(config.pp1_fillMode == _NOBLEND) return "selected";
	}else if (var == "pp1_fillMode_02_sel"){
		if(config.pp1_fillMode == SOURCE) return "selected";
   }else if (var == "pp1_fillMode_03_sel"){
		if(config.pp1_fillMode == SCALED) return "selected";
   }else if (var == "pp1_fillMode_04_sel"){
		if(config.pp1_fillMode == RANDOM) return "selected";
   }

   else if (var == "pp1_pallette_00_sel"){
		if(config.pp1_pallette == 0) return "selected";
   }else if (var == "pp1_pallette_01_sel"){
		if(config.pp1_pallette == 1) return "selected";
	}else if (var == "pp1_pallette_02_sel"){
		if(config.pp1_pallette == 2) return "selected";
   }else if (var == "pp1_pallette_03_sel"){
		if(config.pp1_pallette == 3) return "selected";
   }else if (var == "pp1_pallette_04_sel"){
		if(config.pp1_pallette == 4) return "selected";
   }else if (var == "pp1_pallette_05_sel"){
		if(config.pp1_pallette == 5) return "selected";
   }else if (var == "pp1_pallette_06_sel"){
		if(config.pp1_pallette == 6) return "selected";
	}else if (var == "pp1_pallette_07_sel"){
		if(config.pp1_pallette == 7) return "selected";
   }else if (var == "pp1_pallette_08_sel"){
		if(config.pp1_pallette == 8) return "selected";
   }else if (var == "pp1_pallette_09_sel"){
		if(config.pp1_pallette == 9) return "selected";
   }else if (var == "pp1_pallette_10_sel"){
		if(config.pp1_pallette == 10) return "selected";
   }

   else if (var == "pp1_animSpeed")
      return String(config.pp1_animSpeed);

   else if (var == "pp1_fadeAmount")
      return String(config.pp1_fadeAmount);
      
   else if (var == "pp1_chanceOfGlitter")
      return String(config.pp1_chanceOfGlitter);

   else if (var == "pp1_color0"){
   	sprintf(hexstring, "#%06X", config.pp1_color0);
      return String(hexstring);
   }
   else if (var == "pp1_color1"){
   	sprintf(hexstring, "#%06X", config.pp1_color1);
      return String(hexstring);
   }   
   else if (var == "pp1_color2"){
   	sprintf(hexstring, "#%06X", config.pp1_color2);
      return String(hexstring);
   }   
   else if (var == "pp1_color3"){
   	sprintf(hexstring, "#%06X", config.pp1_color3);
      return String(hexstring);
   }
   else if (var == "pp1_color4"){
   	sprintf(hexstring, "#%06X", config.pp1_color4);
      return String(hexstring);
   }   
   else if (var == "pp1_color5"){
   	sprintf(hexstring, "#%06X", config.pp1_color5);
      return String(hexstring);
   }      
   else if (var == "pp1_color6"){
   	sprintf(hexstring, "#%06X", config.pp1_color6);
      return String(hexstring);
   }
   else if (var == "pp1_color7"){
   	sprintf(hexstring, "#%06X", config.pp1_color7);
      return String(hexstring);
   }
   else if (var == "pp1_glitter_chk"){
		if(config.pp1_glitter)
			return "checked";
   }	
	

   return String();
}


void handleDoUpdate(AsyncWebServerRequest* request, const String& filename, size_t index, uint8_t* data, size_t len, bool final) {
   if (!index)
   {
      Serial.println("Update");
      //content_len = request->contentLength();
      // if filename includes spiffs, update the spiffs partition
      int cmd = (filename.indexOf("spiffs") > -1) ? U_SPIFFS : U_FLASH;
      if (!Update.begin(UPDATE_SIZE_UNKNOWN, cmd))
      {
         Update.printError(Serial);
      }
   }

   if (Update.write(data, len) != len)
   {
      Update.printError(Serial);
   }

   if (final)
   {
      AsyncWebServerResponse* response = request->beginResponse(302, "text/plain", "Please wait while the device reboots");
      response->addHeader("Refresh", "10");
      response->addHeader("Location", "/");
      request->send(response);
      if (!Update.end(true))
      {
         flashLed(CRGB(255,0,0), 5, 300);
         Update.printError(Serial);
      }
      else
      {
      	flashLed(CRGB(0,255,0), 2, 300);
         Serial.println("Update complete");
         Serial.flush();
         ESP.restart();
      }
   }
}


void printProgress(size_t prg, size_t sz) {
   disableLoop = true;
	
	//Serial.printf("Progress: %d\n", prg);
	//Serial.printf("Size: %d\n", sz);
	
   if(sz >= 1){
	   Serial.printf("Progress: %d%%\n", (prg * 100) / sz);

		fillLedsWithBlack(); 
	  	for (int i = 0; i < int((prg * config.hw_numLeds) / sz); i++) {
	   	leds[i] = CRGB(127,127,127);
	   }
      FastLED.show();
    }

}


void flashLed(CRGB color, uint8_t repeats, uint16_t wait_ms){
	disableLoop = true;

   for (uint8_t i = 0; i < repeats; i++){
      for (int i = 0; i < config.hw_numLeds; i++) {
         leds[i] = color;
      }
      FastLED.show();
		delay(wait_ms);
		fillLedsWithBlack();   
      FastLED.show();
		delay(wait_ms);
   }
}

void fadeLedsToBlack(){
	disableLoop = true;
	for(uint8_t i ; i < 15 ; i++){ 
		fadeToBlackBy(leds, config.hw_numLeds, 20);
	   FastLED.show();
		FastLED.delay(10);
   }
   disableLoop = false;	
}


void fillLedsWithBlack(){
	for (int i = 0; i < config.hw_numLeds; i++) {
   	leds[i] = CRGB(0,0,0);
   }	
}


void printLocalTime() {
   struct tm timeinfo;
   if (!getLocalTime(&timeinfo))
   {
      Serial.println("Failed to obtain time");
      return;
   }
   Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
   Serial.print("Day of week: ");
   Serial.println(&timeinfo, "%A");
   Serial.print("Month: ");
   Serial.println(&timeinfo, "%B");
   Serial.print("Day of Month: ");
   Serial.println(&timeinfo, "%d");
   Serial.print("Year: ");
   Serial.println(&timeinfo, "%Y");
   Serial.print("Hour: ");
   Serial.println(&timeinfo, "%H");
   Serial.print("Hour (12 hour format): ");
   Serial.println(&timeinfo, "%I");
   Serial.print("Minute: ");
   Serial.println(&timeinfo, "%M");
   Serial.print("Second: ");
   Serial.println(&timeinfo, "%S");

   Serial.println("Time variables");
   char timeHour[3];
   strftime(timeHour, 3, "%H", &timeinfo);
   Serial.println(timeHour);
   char timeWeekDay[10];
   strftime(timeWeekDay, 10, "%A", &timeinfo);
   Serial.println(timeWeekDay);
   Serial.println();
}


void spiffsInit() {
   // Initialize SPIFFS
   if (!SPIFFS.begin(true)) {
      Serial.println("An Error has occurred while mounting SPIFFS");
   }
}


void wifiInit() {
   // Connect to Wi-Fi
   // Trying to connect as a station to Wi-Fi access point
   if (TRY_WIFI_STATION_MODE) {
      WiFi.mode(WIFI_STA);
      WiFi.setHostname(config.wifi_hostname);
      WiFi.begin(config.wifi_ssid, config.wifi_password);
      for (int i = 0; i <= 4; i++) {
         delay(1000);
         if (WiFi.status() != WL_CONNECTED)
            Serial.println("Connecting to WiFi...");
         else {
            Serial.println("Connected to WiFi...");
            i = 5;
         }
      }

   }

   if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Unable to connect to WiFi, starting AP mode...");
      WiFi.mode(WIFI_AP);
      WiFi.setHostname(config.wifi_hostname);
      WiFi.softAP(config.wifi_hostname);
   }
   else {
      updateNTP();
      time(&rawtime);
      timeinfo = localtime(&rawtime);
      printf("Current local time and date: %s", asctime(timeinfo));
      timeinfo = gmtime(&rawtime);
      printf("Current UTC time and date: %s", asctime(timeinfo));
      server.onNotFound([](AsyncWebServerRequest* request) { request->send(404); });
   }

   if (!MDNS.begin(config.wifi_hostname)) {
      Serial.println("Error setting up MDNS responder!");
   }
   Serial.println("mDNS responder started");

   Serial.print("IP address: ");
   Serial.println(WiFi.localIP());
   Serial.print("hostname: ");
   Serial.println(config.wifi_hostname);

}


void arduinoOTAInit() {

   // Port defaults to 3232
   // ArduinoOTA.setPort(3232);

   //No authentication by default
   ArduinoOTA.setPassword("admin");

   // Password can be set with it's md5 value as well
   // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
   // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
  	Serial.println("Starting OTA");
  	disableLoop=true;
  	SPIFFS.end();
  	});
  
  ArduinoOTA.onEnd([]() {
	flashLed(CRGB(0,255,0), 2, 300);
   Serial.println("Update complete");
  	});
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {});
  
  ArduinoOTA.onError([](ota_error_t error) {
  	flashLed(CRGB(255,0,0), 5, 300);
   Serial.printf("Error[%u]: ", error);
  });
  
  ArduinoOTA.setHostname(config.wifi_hostname);
  
  Update.onProgress(printProgress);
  
  ArduinoOTA.begin();

}


void webServerInit() {
   server.serveStatic("/", SPIFFS, "/").setCacheControl("max-age=600");

   server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
      request->send(SPIFFS, "/index.html", String(), false, processor);
      });

   server.on("/", HTTP_POST, [](AsyncWebServerRequest* request) {
      int params = request->params();
      Serial.printf("got http_post with %d params\n", params);

      for (int i = 0; i < params; i++) {
         AsyncWebParameter* p = request->getParam(i);
         if (p->isFile()) {
            Serial.printf("FILE[%s]: %s, size: %u\n", p->name().c_str(), p->value().c_str(), p->size());
         }
         else if (p->isPost()) {
            Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
         }
         else {
            Serial.printf("GET[%s]: %s\n", p->name().c_str(), p->value().c_str());
         }
      }

      if (request->hasParam("wifi_hostname", true)) {
         request->getParam("wifi_hostname", true)->value().toCharArray(config.wifi_hostname, sizeof(config.wifi_hostname));
         saveConfigRequired = true;
      }

      if (request->hasParam("wifi_ssid", true)) {
         request->getParam("wifi_ssid", true)->value().toCharArray(config.wifi_ssid, sizeof(config.wifi_ssid));
         saveConfigRequired = true;
      }

      if (request->hasParam("wifi_password", true)) {
         request->getParam("wifi_password", true)->value().toCharArray(config.wifi_password, sizeof(config.wifi_password));
         saveConfigRequired = true;
      }

      if (request->hasParam("ntp_server1address", true)) {
         request->getParam("ntp_server1address", true)->value().toCharArray(config.ntp_server1address, sizeof(config.ntp_server1address));
         saveConfigRequired = true;
         updateNTPRequired = true;
      }

      if (request->hasParam("ntp_server2address", true)) {
         request->getParam("ntp_server2address", true)->value().toCharArray(config.ntp_server2address, sizeof(config.ntp_server2address));
         saveConfigRequired = true;
         updateNTPRequired = true;
      }

      if (request->hasParam("ntp_server3address", true)) {
         request->getParam("ntp_server3address", true)->value().toCharArray(config.ntp_server3address, sizeof(config.ntp_server3address));
         saveConfigRequired = true;
         updateNTPRequired = true;
      }

      if (request->hasParam("time_tzInfoString", true)) {
         request->getParam("time_tzInfoString", true)->value().toCharArray(config.time_tzInfoString, sizeof(config.time_tzInfoString));
         saveConfigRequired = true;
         updateNTPRequired = true;
      }

      if (request->hasParam("hw_ledType", true)) {
         request->getParam("hw_ledType", true)->value().toCharArray(config.hw_ledType, sizeof(config.hw_ledType));
         saveConfigRequired = true;
      }

      if (request->hasParam("hw_maxAmp", true)) {
         if (request->getParam("hw_maxAmp", true)->value() == "0,5A")
            config.hw_maxAmp = 500;
         else if (request->getParam("hw_maxAmp", true)->value() == "1A")
            config.hw_maxAmp = 1000;
         else if (request->getParam("hw_maxAmp", true)->value() == "1,5A")
            config.hw_maxAmp = 1500;
         else if (request->getParam("hw_maxAmp", true)->value() == "2A")
            config.hw_maxAmp = 2000;
         else if (request->getParam("hw_maxAmp", true)->value() == "2,5A")
            config.hw_maxAmp = 2500;
         else if (request->getParam("hw_maxAmp", true)->value() == "3A")
            config.hw_maxAmp = 3000;
         else if (request->getParam("hw_maxAmp", true)->value() == "3,5A")
            config.hw_maxAmp = 3500;
         else if (request->getParam("hw_maxAmp", true)->value() == "4A")
            config.hw_maxAmp = 4000;
         else if (request->getParam("hw_maxAmp", true)->value() == "4,5A")
            config.hw_maxAmp = 4500;
         else if (request->getParam("hw_maxAmp", true)->value() == "5A")
            config.hw_maxAmp = 5000;
         saveConfigRequired = true;
      }

      if (request->hasParam("currentBrightness", true)) {
         config.currentBrightness = request->getParam("currentBrightness", true)->value().toInt();
         Serial.printf("brightness changed through web page, value = %u\n", config.currentBrightness);
         FastLED.setBrightness(config.currentBrightness);
      }

      if (request->hasParam("hw_initialBrightness", true)) {
         config.hw_initialBrightness = request->getParam("hw_initialBrightness", true)->value().toInt();
         saveConfigRequired = true;
      }

      if (request->hasParam("hw_numLeds", true)) {
         config.hw_numLeds = request->getParam("hw_numLeds", true)->value().toInt();
         saveConfigRequired = true;
      }

      if (request->hasParam("hw_numStrings", true)) {
         config.hw_numStrings = request->getParam("hw_numStrings", true)->value().toInt();
         saveConfigRequired = true;
      }

      if (request->hasParam("hw_ledPin", true)) {
         config.hw_ledPin = request->getParam("hw_ledPin", true)->value().toInt();
         saveConfigRequired = true;
      }

      if (request->hasParam("hw_programButtonPin", true)) {
         config.hw_programButtonPin = request->getParam("hw_programButtonPin", true)->value().toInt();
         saveConfigRequired = true;
      }

      if (request->hasParam("hw_brightnessButtonPin", true)) {
         config.hw_brightnessButtonPin = request->getParam("hw_brightnessButtonPin", true)->value().toInt();
         saveConfigRequired = true;
      }

      if (request->hasParam("hw_colCorrect_red", true)) {
         config.hw_colCorrect_red = request->getParam("hw_colCorrect_red", true)->value().toInt();
         FastLED.setCorrection(CRGB(config.hw_colCorrect_red, config.hw_colCorrect_green, config.hw_colCorrect_blue));
         FastLED.show();
         saveConfigRequired = true;
      }

      if (request->hasParam("hw_colCorrect_green", true)) {
         config.hw_colCorrect_green = request->getParam("hw_colCorrect_green", true)->value().toInt();
         FastLED.setCorrection(CRGB(config.hw_colCorrect_red, config.hw_colCorrect_green, config.hw_colCorrect_blue));
         FastLED.show();
         saveConfigRequired = true;
      }

      if (request->hasParam("hw_colCorrect_blue", true)) {
         config.hw_colCorrect_blue = request->getParam("hw_colCorrect_blue", true)->value().toInt();
         FastLED.setCorrection(CRGB(config.hw_colCorrect_red, config.hw_colCorrect_green, config.hw_colCorrect_blue));
         FastLED.show();
         saveConfigRequired = true;
      }

      if (request->hasParam("currentProgram", true)) {
         config.currentProgram = request->getParam("currentProgram", true)->value().toInt();
         programButton.pressed = true;
         saveConfigRequired = true;
      }

      if (request->hasParam("pp1_animSpeed", true)) {
         config.pp1_animSpeed = request->getParam("pp1_animSpeed", true)->value().toInt();
         saveConfigRequired = true;
      }

      if (request->hasParam("pp1_name", true)) {
         request->getParam("pp1_name", true)->value().toCharArray(config.pp1_name, sizeof(config.pp1_name));
         saveConfigRequired = true;
      }

      if (request->hasParam("pp1_animType", true)) {
         config.pp1_animType = request->getParam("pp1_animType", true)->value().toInt();
         saveConfigRequired = true;
      }

      if (request->hasParam("pp1_fillMode", true)) {
         config.pp1_fillMode = request->getParam("pp1_fillMode", true)->value().toInt();
         saveConfigRequired = true;
      }

      if (request->hasParam("pp1_fadeAmount", true)) {
         config.pp1_fadeAmount = request->getParam("pp1_fadeAmount", true)->value().toInt();
         saveConfigRequired = true;
      }

      if (request->hasParam("pp1_chanceOfGlitter", true)) {
         config.pp1_chanceOfGlitter = request->getParam("pp1_chanceOfGlitter", true)->value().toInt();
         if (request->hasParam("pp1_glitter", true))
        		config.pp1_glitter = true;
         else
         	config.pp1_glitter = false;
         saveConfigRequired = true;
      }

      if (request->hasParam("pp1_pallette", true)) {
         config.pp1_pallette = request->getParam("pp1_pallette", true)->value().toInt();
         saveConfigRequired = true;
      }
      if (request->hasParam("pp1_color0", true)) {
         request->getParam("pp1_color0", true)->value().substring(1, 7).toCharArray(hexstring, sizeof(hexstring));
         config.pp1_color0 = (uint32_t)strtol(hexstring, NULL, 16);
         saveConfigRequired = true;
      }
      if (request->hasParam("pp1_color1", true)) {
         request->getParam("pp1_color1", true)->value().substring(1, 7).toCharArray(hexstring, sizeof(hexstring));
         config.pp1_color1 = (uint32_t)strtol(hexstring, NULL, 16);
         saveConfigRequired = true;
      }
      if (request->hasParam("pp1_color2", true)) {
         request->getParam("pp1_color2", true)->value().substring(1, 7).toCharArray(hexstring, sizeof(hexstring));
         config.pp1_color2 = (uint32_t)strtol(hexstring, NULL, 16);
         saveConfigRequired = true;
      }
      if (request->hasParam("pp1_color3", true)) {
         request->getParam("pp1_color3", true)->value().substring(1, 7).toCharArray(hexstring, sizeof(hexstring));
         config.pp1_color3 = (uint32_t)strtol(hexstring, NULL, 16);
         saveConfigRequired = true;
      }
      if (request->hasParam("pp1_color4", true)) {
         request->getParam("pp1_color4", true)->value().substring(1, 7).toCharArray(hexstring, sizeof(hexstring));
         config.pp1_color4 = (uint32_t)strtol(hexstring, NULL, 16);
         saveConfigRequired = true;
      }
      if (request->hasParam("pp1_color5", true)) {
         request->getParam("pp1_color5", true)->value().substring(1, 7).toCharArray(hexstring, sizeof(hexstring));
         config.pp1_color5 = (uint32_t)strtol(hexstring, NULL, 16);
         saveConfigRequired = true;
      }
      if (request->hasParam("pp1_color6", true)) {
         request->getParam("pp1_color6", true)->value().substring(1, 7).toCharArray(hexstring, sizeof(hexstring));
         config.pp1_color6 = (uint32_t)strtol(hexstring, NULL, 16);
         saveConfigRequired = true;
      }
      if (request->hasParam("pp1_color7", true)) {
         request->getParam("pp1_color7", true)->value().substring(1, 7).toCharArray(hexstring, sizeof(hexstring));
         config.pp1_color7 = (uint32_t)strtol(hexstring, NULL, 16);
         saveConfigRequired = true;
         programButton.pressed = true;
      }


		if(saveConfigRequired){
			saveConfigRequired = false;
			saveConfiguration(filename, config);
		}

		if(updateNTPRequired){
			updateNTPRequired = false;
			updateNTP();
		}



		Serial.println("POST processing complete");
      request->send(SPIFFS, "/index.html", String(), false, processor);


      });


   server.on("/reboot", HTTP_GET, [](AsyncWebServerRequest* request) {
      AsyncWebServerResponse* response = request->beginResponse(302, "text/plain", "Please wait while the device reboots");
      response->addHeader("Refresh", "10");
      response->addHeader("Location", "/");
      request->send(response);
      Serial.println("Rebooting...");
      Serial.flush();
      ESP.restart();
      });


   server.on("/synctime", HTTP_GET, [](AsyncWebServerRequest* request) {
      updateNTP();
      request->redirect("/");
      });


   server.on(
      "/doUpdate", HTTP_POST,
      [](AsyncWebServerRequest* request) {},
      [](AsyncWebServerRequest* request, const String& filename, size_t index, uint8_t* data,
         size_t len, bool final) { handleDoUpdate(request, filename, index, data, len, final); });

	server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
		request->send(200, "text/plain", String(ESP.getFreeHeap()));
  	});

	server.on("/api/programs/0/state", HTTP_GET, [](AsyncWebServerRequest *request){	
		if(config.currentProgram == 0)
			request->send(200, "application/json", "{\"is_active\": \"true\"}");
		else
			request->send(200, "application/json", "{\"is_active\": \"false\"}");
	});
	

  server.on("/api/programs/0/state", HTTP_POST, [](AsyncWebServerRequest * request){}, NULL,
    [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {

   if (deserializeJson(requestBody, data))
      Serial.println("Failed to read body as json data");
	else{
		if(requestBody["active"] == "true"){
			config.currentProgram = 0;
			programButton.pressed = true;
			saveConfiguration(filename, config);
		}else{
			config.currentProgram = 0;
			programButton.pressed = true;
			saveConfiguration(filename, config);      
		}
      request->send(200);
	}
  });
	

	server.on("/api/programs/1/state", HTTP_GET, [](AsyncWebServerRequest *request){	
		if(config.currentProgram == 1)
			request->send(200, "application/json", "{\"is_active\": \"true\"}");
		else
			request->send(200, "application/json", "{\"is_active\": \"false\"}");
	});


  server.on("/api/programs/1/state", HTTP_POST, [](AsyncWebServerRequest * request){}, NULL,
    [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {

   if (deserializeJson(requestBody, data))
      Serial.println("Failed to read body as json data");
	else{
		if(requestBody["active"] == "true"){
			config.currentProgram = 1;
			programButton.pressed = true;
			saveConfiguration(filename, config);
		}else{
			config.currentProgram = 0;
			programButton.pressed = true;
			saveConfiguration(filename, config);      
		}
      request->send(200);
	}
  });

	server.on("/api/programs/2/state", HTTP_GET, [](AsyncWebServerRequest *request){	
		if(config.currentProgram == 2)
			request->send(200, "application/json", "{\"is_active\": \"true\"}");
		else
			request->send(200, "application/json", "{\"is_active\": \"false\"}");
	});


  server.on("/api/programs/2/state", HTTP_POST, [](AsyncWebServerRequest * request){}, NULL,
    [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {

   if (deserializeJson(requestBody, data))
      Serial.println("Failed to read body as json data");
	else{
		if(requestBody["active"] == "true"){
			config.currentProgram = 2;
			programButton.pressed = true;
			saveConfiguration(filename, config);
		}else{
			config.currentProgram = 0;
			programButton.pressed = true;
			saveConfiguration(filename, config);      
		}
      request->send(200);
	}
  });

	server.on("/api/programs/3/state", HTTP_GET, [](AsyncWebServerRequest *request){	
		if(config.currentProgram == 3)
			request->send(200, "application/json", "{\"is_active\": \"true\"}");
		else
			request->send(200, "application/json", "{\"is_active\": \"false\"}");
	});

  server.on("/api/programs/3/state", HTTP_POST, [](AsyncWebServerRequest * request){}, NULL,
    [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {

   if (deserializeJson(requestBody, data))
      Serial.println("Failed to read body as json data");
	else{
		if(requestBody["active"] == "true"){
			config.currentProgram = 3;
			programButton.pressed = true;
			saveConfiguration(filename, config);
		}else{
			config.currentProgram = 0;
			programButton.pressed = true;
			saveConfiguration(filename, config);      
		}
      request->send(200);
	}
  });

	server.on("/api/programs/4/state", HTTP_GET, [](AsyncWebServerRequest *request){	
		if(config.currentProgram == 4)
			request->send(200, "application/json", "{\"is_active\": \"true\"}");
		else
			request->send(200, "application/json", "{\"is_active\": \"false\"}");
	});

  server.on("/api/programs/4/state", HTTP_POST, [](AsyncWebServerRequest * request){}, NULL,
    [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {

   if (deserializeJson(requestBody, data))
      Serial.println("Failed to read body as json data");
	else{
		if(requestBody["active"] == "true"){
			config.currentProgram = 4;
			programButton.pressed = true;
			saveConfiguration(filename, config);
		}else{
			config.currentProgram = 0;
			programButton.pressed = true;
			saveConfiguration(filename, config);      
		}
      request->send(200);
	}
  });

	server.on("/api/programs/5/state", HTTP_GET, [](AsyncWebServerRequest *request){	
		if(config.currentProgram == 5)
			request->send(200, "application/json", "{\"is_active\": \"true\"}");
		else
			request->send(200, "application/json", "{\"is_active\": \"false\"}");
	});

  server.on("/api/programs/5/state", HTTP_POST, [](AsyncWebServerRequest * request){}, NULL,
    [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {

   if (deserializeJson(requestBody, data))
      Serial.println("Failed to read body as json data");
	else{
		if(requestBody["active"] == "true"){
			config.currentProgram = 5;
			programButton.pressed = true;
			saveConfiguration(filename, config);
		}else{
			config.currentProgram = 0;
			programButton.pressed = true;
			saveConfiguration(filename, config);      
		}
      request->send(200);
	}
  });

	server.on("/api/programs/6/state", HTTP_GET, [](AsyncWebServerRequest *request){	
		if(config.currentProgram == 6)
			request->send(200, "application/json", "{\"is_active\": \"true\"}");
		else
			request->send(200, "application/json", "{\"is_active\": \"false\"}");
	});

  server.on("/api/programs/6/state", HTTP_POST, [](AsyncWebServerRequest * request){}, NULL,
    [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {

   if (deserializeJson(requestBody, data))
      Serial.println("Failed to read body as json data");
	else{
		if(requestBody["active"] == "true"){
			config.currentProgram = 6;
			programButton.pressed = true;
			saveConfiguration(filename, config);
		}else{
			config.currentProgram = 0;
			programButton.pressed = true;
			saveConfiguration(filename, config);      
		}
      request->send(200);
	}
  });

	server.on("/api/programs/7/state", HTTP_GET, [](AsyncWebServerRequest *request){	
		if(config.currentProgram == 7)
			request->send(200, "application/json", "{\"is_active\": \"true\"}");
		else
			request->send(200, "application/json", "{\"is_active\": \"false\"}");
	});

  server.on("/api/programs/7/state", HTTP_POST, [](AsyncWebServerRequest * request){}, NULL,
    [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {

   if (deserializeJson(requestBody, data))
      Serial.println("Failed to read body as json data");
	else{
		if(requestBody["active"] == "true"){
			config.currentProgram = 7;
			programButton.pressed = true;
			saveConfiguration(filename, config);
		}else{
			config.currentProgram = 0;
			programButton.pressed = true;
			saveConfiguration(filename, config);      
		}
      request->send(200);
	}
  });

	server.on("/api/programs/8/state", HTTP_GET, [](AsyncWebServerRequest *request){	
		if(config.currentProgram == 8)
			request->send(200, "application/json", "{\"is_active\": \"true\"}");
		else
			request->send(200, "application/json", "{\"is_active\": \"false\"}");
	});

  server.on("/api/programs/8/state", HTTP_POST, [](AsyncWebServerRequest * request){}, NULL,
    [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {

   if (deserializeJson(requestBody, data))
      Serial.println("Failed to read body as json data");
	else{
		if(requestBody["active"] == "true"){
			config.currentProgram = 8;
			programButton.pressed = true;
			saveConfiguration(filename, config);
		}else{
			config.currentProgram = 0;
			programButton.pressed = true;
			saveConfiguration(filename, config);      
		}
      request->send(200);
	}
  });

	server.on("/api/programs/9/state", HTTP_GET, [](AsyncWebServerRequest *request){	
		if(config.currentProgram == 9)
			request->send(200, "application/json", "{\"is_active\": \"true\"}");
		else
			request->send(200, "application/json", "{\"is_active\": \"false\"}");
	});

  server.on("/api/programs/9/state", HTTP_POST, [](AsyncWebServerRequest * request){}, NULL,
    [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {

   if (deserializeJson(requestBody, data))
      Serial.println("Failed to read body as json data");
	else{
		if(requestBody["active"] == "true"){
			config.currentProgram = 9;
			programButton.pressed = true;
			saveConfiguration(filename, config);
		}else{
			config.currentProgram = 0;
			programButton.pressed = true;
			saveConfiguration(filename, config);      
		}
      request->send(200);
	}
  });

	server.on("/api/programs/10/state", HTTP_GET, [](AsyncWebServerRequest *request){	
		if(config.currentProgram == 10)
			request->send(200, "application/json", "{\"is_active\": \"true\"}");
		else
			request->send(200, "application/json", "{\"is_active\": \"false\"}");
	});

  server.on("/api/programs/10/state", HTTP_POST, [](AsyncWebServerRequest * request){}, NULL,
    [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {

   if (deserializeJson(requestBody, data))
      Serial.println("Failed to read body as json data");
	else{
		if(requestBody["active"] == "true"){
			config.currentProgram = 10;
			programButton.pressed = true;
			saveConfiguration(filename, config);
		}else{
			config.currentProgram = 0;
			programButton.pressed = true;
			saveConfiguration(filename, config);      
		}
      request->send(200);
	}
  });

	server.on("/api/programs/11/state", HTTP_GET, [](AsyncWebServerRequest *request){	
		if(config.currentProgram == 11)
			request->send(200, "application/json", "{\"is_active\": \"true\"}");
		else
			request->send(200, "application/json", "{\"is_active\": \"false\"}");
	});

  server.on("/api/programs/11/state", HTTP_POST, [](AsyncWebServerRequest * request){}, NULL,
    [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {

   if (deserializeJson(requestBody, data))
      Serial.println("Failed to read body as json data");
	else{
		if(requestBody["active"] == "true"){
			config.currentProgram = 11;
			programButton.pressed = true;
			saveConfiguration(filename, config);
		}else{
			config.currentProgram = 0;
			programButton.pressed = true;
			saveConfiguration(filename, config);      
		}
      request->send(200);
	}
  });

	server.on("/api/programs/12/state", HTTP_GET, [](AsyncWebServerRequest *request){	
		if(config.currentProgram == 12)
			request->send(200, "application/json", "{\"is_active\": \"true\"}");
		else
			request->send(200, "application/json", "{\"is_active\": \"false\"}");
	});

  server.on("/api/programs/12/state", HTTP_POST, [](AsyncWebServerRequest * request){}, NULL,
    [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {

   if (deserializeJson(requestBody, data))
      Serial.println("Failed to read body as json data");
	else{
		if(requestBody["active"] == "true"){
			config.currentProgram = 12;
			programButton.pressed = true;
			saveConfiguration(filename, config);
		}else{
			config.currentProgram = 0;
			programButton.pressed = true;
			saveConfiguration(filename, config);      
		}
      request->send(200);
	}
  });

   // Start server
   server.begin();
}


void IRAM_ATTR brightnessIsr() {
   if ((millis() - brightnessButton.lastDebounceTime) > brightnessButton.debounceDelay) {
      if (digitalRead(config.hw_brightnessButtonPin) == 0) {
         //real button changed
         config.currentBrightness *= 1.5;
         //last possible step before full range is 254, 254*1,5 = 381
         if (config.currentBrightness > 381) { config.currentBrightness = 16; }
         if (config.currentBrightness >= 255) { config.currentBrightness = 255; }
         brightnessButton.pressed = true;
         brightnessButton.lastDebounceTime = millis();
      }
   }
}


void IRAM_ATTR programIsr() {
   if ((millis() - programButton.lastDebounceTime) > programButton.debounceDelay) {
      if (digitalRead(config.hw_programButtonPin) == 0) {
         //real button changed
         config.currentProgram += 1;
         programButton.pressed = true;
         programButton.lastDebounceTime = millis();
      }
   }
}
