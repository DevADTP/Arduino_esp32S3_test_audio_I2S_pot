/*
  carte dev
  - esp32 : https://espressif.github.io/arduino-esp32/package_esp32_index.json

  librairies
  - audio I2S : https://github.com/schreibfaul1/ESP32-audioI2S.git
  - neopixel  : https://github.com/FastLED/FastLED.git
  - rotary encoder : https://github.com/igorantolic/ai-esp32-rotary-encoder.git1
  - RTC PCF8563 : https://github.com/adafruit/RTClib.git
  - cryto : https://github.com/rweather/arduinolibs.git

  PSRAM externe : OSPI PSRAM that embed ESP32-S3R8, pins IO35, IO36, and IO37 
  io35 : SD_DAC (I2S audio)
  io36 : SCK_DAC (I2S audio)
  io37 : WS_DAC (I2S audio)
*/

//#if ARDUINO_USB_MODE
//#warning This sketch should be used when USB is in OTG mode
//void setup(){}
//void loop(){}
//#else
//#include "USB.h"
//#include "FirmwareMSC.h"
//
//#if !ARDUINO_USB_MSC_ON_BOOT
//FirmwareMSC MSC_Update;
//#endif
//#if ARDUINO_USB_CDC_ON_BOOT
//#define HWSerial Serial0
//#define USBSerial Serial
//#else
//#define HWSerial Serial
//USBCDC USBSerial;
//#endif

#include <Arduino.h>
#include <WiFi.h>
#include <Audio.h>
#include <FastLED.h>
#include <AiEsp32RotaryEncoder.h>
#include "SD.h"
#include "FS.h"
#include "SPIFFS.h"
#include "SPI.h"
#include "time.h"

#include "RTClib.h"
#include <Wire.h>

// PINOUT SD CARD
#define SD_CS 10
#define SPI_MOSI 11  // SD Card
#define SPI_MISO 13
#define SPI_SCK 12

//PINOUT I2S CARD (AUDIO 3W)
#define I2S_DOUT 35  //35:protoV1  45:protoV2
#define I2S_BCLK 36  //36:protoV1  48:protoV2
#define I2S_LRC 37   //37:protoV1  46:protoV2
#define I2S_GAIN 39
#define PIN_ENABLE_I2S 42  //41:dev board  42:protov1

//PINOUT I2C RTC PCF8563
#define SDA_PIN_RTC_PCF8563 15
#define SCL_PIN_RTC_PCF8563 16
#define IRQ_PIN_RTC_PCF8563 17

//potentiometre volume
#define PIN_VOLUME 4

//button PLAY (used as gain audio)
#define PIN_BUTTON_PLAY 18
#define PIN_BUTTON_NEXT 8    //18:dev board  8:protov1

//use for turn off red light indicator after 10s
#define TIME_PICTURE_END 10000  //milliseconds (10s)

#define ROTARY_ENCODER_A_PIN 5
#define ROTARY_ENCODER_B_PIN 6
#define ROTARY_ENCODER_BUTTON_PIN 7
#define ROTARY_ENCODER_VCC_PIN 17
#define ROTARY_ENCODER_STEPS 4
#define CYCLE_ROT 8  //9-1  //24 ref bourns PEC11R-4015F-S0024 (RS:737-7739)

//instead of changing here, rather change numbers above
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

//neopixel
#define PIN_ENABLE_NEOPIXEL 41  //40:dev board  41:protov1 not exit on dev board only protov1

//ADC boutton switch rotatif
#define PIN_SWITCH_THEME 2
#define PIN_SWITCH_USER 1

//BATTERY VOLTAGE
#define PIN_BAT_MEAS 14    //not use in dev board

//CHARGE BATTERY STATUS
#define PIN_CHARGE_STATUS 9

//JACK CONNECTED
#define PIN_ADC_JACK_DETECT 47  //14: dev board 47:protov1

//audio i2S
#define PERIOD_READ_VOLUME 50    //ms
#define PERIOD_CHANGE_GAIN 5000  //ms updateTimeGain
#define PERIOD_READ_ADC 1000     //ms updateTimeGain
#define PERIOD_JACK_DETECT 500   //ms read stats jack
#define COUNTER_MAX_JACK 20
Audio audio;

//PERIOD DEFINE
#define PERIOD_LOG_UART 1000  //ms updateTimeGain

//PARAMETERS
//--------------------------------------------------------------------------------------------
//WIFI
String ssid = "*****************";
String password = "**********************";

#define WIFI_ACTIVE 0       //default 1  update RTC on NTP server
#define TEST_GAIN_VOLUME 0  //default 0
#define TEST_LED 0          //default 0  1:test led
#define LAMPE_NB_COLOR 1    //default 1  20:mix cobinaison couleur  top/bottom
#define AUDIO_ACTIVE 1      //default 1 (0:PSRAM access)
#define RTC_ACTIVE 0        //default 1

int int_test_volume = 0;
int int_test_led = 50;

//PARAMETERS
//--------------------------------------------------------------------------------------------

long int valVolume = 0;
long int valVolumeold = 0;
int updatevolume = 0;
int jackInserted = 0;              //status jack audio
volatile int jackInsertedCnt = 0;  //counter jack status

unsigned long nowTimeMillis = 0;
unsigned long updateTimeVolume = 0;
unsigned long updateTimeGain = 0;
unsigned long updateTimeHorloge = 0;
unsigned long updateAdcRead = 0;
unsigned long updateLogUart = 0;
unsigned long CheckTimeJackInserted = 0;

int i = 0;  //for

int changeGain = 0;

int readButPlay = 0;
int readButNext = 0;
int buttonStatePlay;                     // the current reading from the input pin
int buttonStateNext;                     // the current reading from the input pin
int buttonStateLongPress;                // the current reading from the input pin
int lastButtonPlay = HIGH;               // the previous reading from the input pin
int lastButtonNext = HIGH;               // the previous reading from the input pin
unsigned long lastDebounceTimePlay = 0;  // the last time the output pin was toggled
unsigned long lastDebounceTimeNext = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;        // the debounce time; increase if the output flickers

unsigned long longPressButton = 1500;  // long time pressure button

int nextSong = 0;

//adc
int analogSwitchTheme = 0;
int analogSwitchuser = 0;
int analogJackInserted = 0;
int analogBatVoltage = 0;  //PIN_BAT_MEAS
int matAnalogBatVoltage[16];
long int sumBatVoltage = 0;
int indiceBatVoltage = 0;

//battery
int chargeStatus = 0;

//fastled
#define NUM_LEDS 9
#define BRIGHTNESS 200
#define DATA_PIN 40
#define DATA_PIN2 38
#define CLOCK_PIN 13  //not use

//fastled pot
#define NUM_LEDS2 4     ////1: dev board 4:protov1

// Define the array of leds
CRGB leds[NUM_LEDS];
CRGB leds2[NUM_LEDS2];

CRGB couleur = CRGB(0, 0, 0);

int numero_led = 0;
int flip_light = 0;
unsigned long ulong_time_now = 0;
unsigned long ulong_time_picture = 0;  //TIME_PICTURE_END

//files
int intNbAudioFileInDir = 0;
int intNumeroDossier = 9;
int intNombreDossier = 9;

char name_directory[100] = "/09";


//time NTP RTC
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600 * 1;
const int daylightOffset_sec = 3600 * 1;
#define DISPLAY_TIME_PERIOD 60000  //60 seconde on uart
RTC_PCF8563 rtc;
DateTime now;
struct tm timeinfo;
char daysOfTheWeek[7][12] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };


//wifi
int intWifiConnectRetry = 0;

// flag to update serial; set in interrupt callback
volatile uint8_t tick_tock = 1;



//fonction declaration
void change_song(void);
void logUart(void);
void readBatLevel(void);


// INT0 interrupt callback; update tick_tock flag
void set_tick_tock(void) {
  tick_tock = 1;
}



void rotary_onButtonClick() {
  static unsigned long lastTimePressed = 0;
  //ignore multiple press in that time milliseconds
  if (millis() - lastTimePressed < 500) {
    return;
  }
  lastTimePressed = millis();
  Serial.print("button pressed ");
  Serial.print(millis());
  Serial.println(" milliseconds after restart");

  flip_light++;
  if (flip_light > LAMPE_NB_COLOR) flip_light = 0;

  ulong_time_picture = millis() + TIME_PICTURE_END;
}



void rotary_loop() {
  //dont print anything unless value changed
  if (rotaryEncoder.encoderChanged()) {
    //Serial.print("Value: ");
    numero_led = rotaryEncoder.readEncoder();
    intNumeroDossier = (numero_led % intNombreDossier) + 1;

    Serial.print("Dossier THEME:");
    Serial.println(intNumeroDossier);

    Serial.print("Led : ");
    Serial.println(numero_led);

    sprintf(name_directory, "/%02d", intNumeroDossier);
    intNbAudioFileInDir = 0;
    //listDir(SD, "/05", 1);
    listDir(SD, name_directory, 1);

    nextSong = 0;

    switch (intNumeroDossier) {
      case 1:
        audio.connecttoSD("/01/000.mp3");
        break;
      case 2:
        audio.connecttoSD("/02/000.mp3");
        break;
      case 3:
        audio.connecttoSD("/03/000.mp3");
        break;
      case 4:
        audio.connecttoSD("/04/000.mp3");
        break;
      case 5:
        audio.connecttoSD("/05/000.mp3");
        break;
      case 6:
        audio.connecttoSD("/06/000.mp3");
        break;
      case 7:
        audio.connecttoSD("/07/000.mp3");
        break;
      case 8:
        audio.connecttoSD("/08/000.mp3");
        break;
      case 9:
        audio.connecttoSD("/09/000.mp3");
        break;
      default:
        audio.connecttoSD("/09/000.mp3");
        break;
    }

    ulong_time_picture = millis() + TIME_PICTURE_END;
  }
  if (rotaryEncoder.isEncoderButtonClicked()) {
    rotary_onButtonClick();
  }
}



void IRAM_ATTR readEncoderISR() {
  rotaryEncoder.readEncoder_ISR();
}



void initSDCard() {
  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}



void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      intNbAudioFileInDir = 0;
      //Serial.print("  DIR : ");
      //Serial.println(file.name());
      if (levels) {
        char addslash[100];
        strcpy(addslash, "/");
        strcat(addslash, file.name());
        //Serial.println(addslash);
        listDir(fs, addslash, levels - 1);
        //listDir(fs,file.name(), levels - 1);
      }
    } else {

      intNbAudioFileInDir++;
      char addfullpath[100];
      Serial.print("  FILE: ");
      strcpy(addfullpath, dirname);
      strcat(addfullpath, "/");
      strcat(addfullpath, file.name());
      //Serial.print(file.name());
      Serial.print(addfullpath);
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }

  Serial.print("File number:");
  Serial.println(intNbAudioFileInDir);
}

void createDir(fs::FS &fs, const char *path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char *path) {
  Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) {
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char *path1, const char *path2) {
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char *path) {
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file) {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len) {
      size_t toRead = len;
      if (toRead > 512) {
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }


  file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for (i = 0; i < 2048; i++) {
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();
}



void printLocalTime() {

  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

  // Serial.print("Wait time ms:");
  // updateTimeHorloge = timeinfo.tm_sec;
  // Serial.print(updateTimeHorloge);
  // Serial.print(" -- ");
  updateTimeHorloge = millis() + (60 - timeinfo.tm_sec) * 1000;

  // Serial.println(updateTimeHorloge);

  // Serial.print("Day of week: ");
  // Serial.println(&timeinfo, "%A");
  // Serial.print("Month: ");
  // Serial.println(&timeinfo, "%B");
  // Serial.print("Day of Month: ");
  // Serial.println(&timeinfo, "%d");
  // Serial.print("Year: ");
  // Serial.println(&timeinfo, "%Y");
  // Serial.print("Hour: ");
  // Serial.println(&timeinfo, "%H");
  // Serial.print("Hour (12 hour format): ");
  // Serial.println(&timeinfo, "%I");
  // Serial.print("Minute: ");
  // Serial.println(&timeinfo, "%M");
  // Serial.print("Second: ");
  // Serial.println(&timeinfo, "%S");

  // Serial.println("Time variables");
  // char timeHour[3];
  // strftime(timeHour, 3, "%H", &timeinfo);
  // Serial.println(timeHour);
  // char timeWeekDay[10];
  // strftime(timeWeekDay, 10, "%A", &timeinfo);
  // Serial.println(timeWeekDay);
  // Serial.println();


  //RTC PCF8563
  if (RTC_ACTIVE == 1) {
    now = rtc.now();
  }

  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
}


//
//static void usbEventCallback(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
//  if (event_base == ARDUINO_USB_EVENTS) {
//    arduino_usb_event_data_t *data = (arduino_usb_event_data_t *)event_data;
//    switch (event_id) {
//      case ARDUINO_USB_STARTED_EVENT:
//        HWSerial.println("USB PLUGGED");
//        break;
//      case ARDUINO_USB_STOPPED_EVENT:
//        HWSerial.println("USB UNPLUGGED");
//        break;
//      case ARDUINO_USB_SUSPEND_EVENT:
//        HWSerial.printf("USB SUSPENDED: remote_wakeup_en: %u\n", data->suspend.remote_wakeup_en);
//        break;
//      case ARDUINO_USB_RESUME_EVENT:
//        HWSerial.println("USB RESUMED");
//        break;
//
//      default:
//        break;
//    }
//  } else if (event_base == ARDUINO_FIRMWARE_MSC_EVENTS) {
//    arduino_firmware_msc_event_data_t *data = (arduino_firmware_msc_event_data_t *)event_data;
//    switch (event_id) {
//      case ARDUINO_FIRMWARE_MSC_START_EVENT:
//        HWSerial.println("MSC Update Start");
//        break;
//      case ARDUINO_FIRMWARE_MSC_WRITE_EVENT:
//        //HWSerial.printf("MSC Update Write %u bytes at offset %u\n", data->write.size, data->write.offset);
//        HWSerial.print(".");
//        break;
//      case ARDUINO_FIRMWARE_MSC_END_EVENT:
//        HWSerial.printf("\nMSC Update End: %u bytes\n", data->end.size);
//        break;
//      case ARDUINO_FIRMWARE_MSC_ERROR_EVENT:
//        HWSerial.printf("MSC Update ERROR! Progress: %u bytes\n", data->error.size);
//        break;
//      case ARDUINO_FIRMWARE_MSC_POWER_EVENT:
//        HWSerial.printf("MSC Update Power: power: %u, start: %u, eject: %u", data->power.power_condition, data->power.start, data->power.load_eject);
//        break;
//
//      default:
//        break;
//    }
//  }
//}


void jackChangeInterrupt() {
  jackInsertedCnt++;
  jackInserted = 0;
  CheckTimeJackInserted = millis() + PERIOD_JACK_DETECT;
  detachInterrupt(digitalPinToInterrupt(PIN_ADC_JACK_DETECT));
}


void setup() {
  //delay(1000);

  //  HWSerial.begin(2000000);
  //  HWSerial.setDebugOutput(true);
  //
  //  USB.onEvent(usbEventCallback);
  //  MSC_Update.onEvent(usbEventCallback);
  //  MSC_Update.begin();
  //  USBSerial.begin();
  //  USB.begin();
  //
  Serial.begin(2000000);  //uart debug:2000000   uart_usb_otg:115200
  delay(20);
  //while (!Serial) {
  // }
  Serial.println("************************************************************");
  Serial.println("START PROGRAM");
  Serial.println("************************************************************");

  //ADC
  //set the resolution to 12 bits (0-4096)
  analogReadResolution(12);

  //I2C RTC
  if (RTC_ACTIVE == 1) {
    Wire.begin(SDA_PIN_RTC_PCF8563, SCL_PIN_RTC_PCF8563);  //SDA SCL
    pinMode(IRQ_PIN_RTC_PCF8563, INPUT);                   // set up interrupt pin
    digitalWrite(IRQ_PIN_RTC_PCF8563, HIGH);               // turn on pullup resistors
    // attach interrupt to set_tick_tock callback on rising edge of INT0
    attachInterrupt(digitalPinToInterrupt(IRQ_PIN_RTC_PCF8563), set_tick_tock, RISING);

    if (!rtc.begin()) {
      Serial.println("Couldn't find RTC");
      Serial.flush();
      while (1) delay(10);
    } else {
      Serial.println("RTC PCF8563 detect");
    }
  }

  //sd card
  if (AUDIO_ACTIVE == 1) {
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, LOW);
    delay(10);
    digitalWrite(SD_CS, HIGH);
    //SPI.begin(SCK, MISO, MOSI, SS);
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS);

    //SD card
    initSDCard();

    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
  }

  //memoire PSRAM
  //Initialisation PSRAM
  if (psramInit()) {
    Serial.println("PSRAM init pass");
  } else {
    Serial.println("PSRAM fail");
  }

  Serial.print("Memoire disponible PSRAM : ");
  Serial.println( ESP.getFreePsram());

  delay(10);

  //wifi
  if (WIFI_ACTIVE == 1) {
    WiFi.disconnect();

    WiFi.mode(WIFI_STA);

    WiFi.begin(ssid.c_str(), password.c_str());

    Serial.print("WIFI connect :");
    while (WiFi.status() != WL_CONNECTED) {
      intWifiConnectRetry++;
      Serial.print(".");
      delay(100);
      if (intWifiConnectRetry >= 50) break;
    }

    if (intWifiConnectRetry >= 20) {
      Serial.println("wifi PROBLEM not connected");
    } else {
      Serial.println("wifi OK");
    }
    delay(1500);

    //  Serial.print("DATETIME avant: ");
    //  Serial.print(__DATE__);
    //  Serial.print(" - ");
    //  Serial.println(__TIME__);

    //get time ntp
    // Init and get the time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    printLocalTime();

    //update time in RTC
    //structure tm (int8_t wday, int16_t year, int8_t mon, int8_t mday, int8_t hour, int8_t min, int8_t sec)
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    Serial.print("Extract time : ");
    Serial.print((timeinfo.tm_year + 1900));
    Serial.print("_");
    Serial.print(timeinfo.tm_mon + 1);
    Serial.print("_");
    Serial.print(timeinfo.tm_mday);
    Serial.print("_");
    Serial.print(timeinfo.tm_hour);
    Serial.print("_");
    Serial.print(timeinfo.tm_min);
    Serial.print("_");
    Serial.println(timeinfo.tm_sec);

    //reglage RTC with wifi NTP
    if (RTC_ACTIVE == 1) {
      rtc.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec));
      rtc.start();
    }

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);

  }  //wifi define

  Serial.println("init neopixel");

  //power supply led neopixel
  pinMode(PIN_ENABLE_NEOPIXEL, OUTPUT);
  digitalWrite(PIN_ENABLE_NEOPIXEL, LOW);  //HIGH:led off LOW:led on

  //9 leds
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed

  //4 leds
  FastLED.addLeds<NEOPIXEL, DATA_PIN2>(leds2, NUM_LEDS2);  // GRB ordering is assumed

  FastLED.setBrightness(BRIGHTNESS);

  //start animation
  for (i = 0; i < NUM_LEDS; i++) {
    leds[i % 3] = CRGB(0, 0, 150);
    leds[i % 3 + 3] = CRGB(70, 70, 70);
    leds[i % 3 + 6] = CRGB(200, 0, 0);
    delay(200);
    FastLED.show();
  }

  delay(1000);

  for (i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();

  Serial.println("init rotary");
  pinMode(ROTARY_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ROTARY_ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(ROTARY_ENCODER_BUTTON_PIN, INPUT_PULLUP);

  //we must initialize rotary encoder
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  //set boundaries and if values should cycle or not
  //in this example we will set possible values between 0 and 1000;
  bool circleValues = true;
  rotaryEncoder.setBoundaries(0, CYCLE_ROT, circleValues);  //minValue, maxValue, circleValues true|false (when max go to min and vice versa)

  /*Rotary acceleration introduced 25.2.2021.
     in case range to select is huge, for example - select a value between 0 and 1000 and we want 785
     without accelerateion you need long time to get to that number
     Using acceleration, faster you turn, faster wil
     l the value raise.
     For fine tuning slow down.
  */
  //rotaryEncoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
  rotaryEncoder.setAcceleration(1);  //250 or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration

  ulong_time_picture = millis() + TIME_PICTURE_END;

  //gain audio on boot
  Serial.println("gain:3dB");  //GAIN_SLOT=Pull-up 3.3V -> 3 dB
  pinMode(I2S_GAIN, INPUT_PULLUP);

  // Serial.println("gain:6dB");  //GAIN_SLOT=VDD=3.3V -> 6dB
  // pinMode(I2S_GAIN, OUTPUT);
  // digitalWrite(I2S_GAIN, HIGH);

  // Serial.println("gain:9dB");  //GAIN_SLOT=floating input -> 9dB
  // pinMode(I2S_GAIN, INPUT);

  // Serial.println("gain:12dB");  //GAIN_SLOT=GND -> 12dB
  // pinMode(I2S_GAIN, OUTPUT);
  // digitalWrite(I2S_GAIN, LOW);

  // Serial.println("gain:15dB");  //GAIN_SLOT=Pull-down GND -> 15 dB
  // pinMode(I2S_GAIN, INPUT_PULLDOWN);

  //audio web radio
  //enable AUDIO
  pinMode(PIN_ENABLE_I2S, INPUT);  // floatting left+right/2

  if (AUDIO_ACTIVE == 1) {
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setBalance(-16);  // mutes the left channel
    audio.setVolume(0);
    audio.forceMono(true);  //mono application

    //stream music
    //audio.connecttohost("http://vis.media-ice.musicradio.com/CapitalMP3");

    //sd musique
    //audio.connecttoFS(SD, "/04/001.mp3");
    audio.connecttoSD("/04/001.mp3");
    audio.pauseResume();
  }

  pinMode(PIN_BUTTON_PLAY, INPUT_PULLUP);
  pinMode(PIN_BUTTON_NEXT, INPUT_PULLUP);

  //pullup jack detect
  pinMode(PIN_ADC_JACK_DETECT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ADC_JACK_DETECT), jackChangeInterrupt, CHANGE);


  //turn off wifi
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  //battery level
  for (i = 0; i < 16; i++) {
    readBatLevel();
  }
}



void loop() {
  nowTimeMillis = millis();
  ulong_time_now = millis();

  //change gain depending jack inserted

  // if (jackInserted == 1) {
  //   //3dB casque
  //   pinMode(I2S_GAIN, OUTPUT);
  //   digitalWrite(I2S_GAIN, HIGH);
  // } else {
  //   //12db haut parleur
  //   pinMode(I2S_GAIN, INPUT);
  //       //3dB casque
  //   pinMode(I2S_GAIN, OUTPUT);
  //   digitalWrite(I2S_GAIN, HIGH);
  // }

  //check jack status digital pin not analog
  if (millis() > CheckTimeJackInserted) {
    jackInserted = 1;
    jackInsertedCnt = 0;
    attachInterrupt(digitalPinToInterrupt(PIN_ADC_JACK_DETECT), jackChangeInterrupt, CHANGE);
  }

  //read ADC value switch rotary 5 positions ADC
  if (millis() > updateAdcRead) {
    updateAdcRead = millis() + PERIOD_READ_ADC;

    // switch theme
    analogSwitchTheme = analogRead(PIN_SWITCH_THEME);
    analogSwitchTheme = analogRead(PIN_SWITCH_THEME);
    analogSwitchTheme = analogRead(PIN_SWITCH_THEME);
    //    Serial.printf("Switch theme = %d\n", analogSwitchTheme);
    //    analogSwitchTheme = analogReadMilliVolts(PIN_SWITCH_THEME);
    //Serial.printf("ADC mV theme = %d\n", analogSwitchTheme);
    //Serial.printf("%d,", analogSwitchTheme);

    // switch user
    analogSwitchuser = analogRead(PIN_SWITCH_USER);
    analogSwitchuser = analogRead(PIN_SWITCH_USER);
    analogSwitchuser = analogRead(PIN_SWITCH_USER);
    //    Serial.printf("Switch user = %d\n", analogSwitchuser);
    //    analogSwitchuser = analogReadMilliVolts(PIN_SWITCH_USER);
    //Serial.printf("ADC mV user = %d\n", analogSwitchuser);
    //Serial.printf("%d,", analogSwitchuser);

    //Jack connected
    //    analogJackInserted = analogRead(PIN_ADC_JACK_DETECT);
    //    Serial.printf("ADC jack = %d\n", analogJackInserted);
    //analogJackInserted = analogReadMilliVolts(PIN_ADC_JACK_DETECT);
    //analogJackInserted = digitalRead(PIN_ADC_JACK_DETECT);
    //Serial.printf("ADC mV jack = %d\n", analogJackInserted);
    //Serial.printf("%d\n", analogJackInserted);
    //Serial.printf("%d,%d\n", jackInsertedCnt, jackInserted);

    //Battery voltage adc@3.1V
    readBatLevel();

    //Charge status
    chargeStatus = digitalRead(PIN_CHARGE_STATUS);
  }


  //log data
  logUart();

  //update dispay time
  if (WIFI_ACTIVE == 1) {
    if (millis() > updateTimeHorloge) {
      printLocalTime();
    }
  }

  //in loop call your custom function which will process rotary encoder values
  rotary_loop();

  //PART button play change power audio
  readButPlay = digitalRead(PIN_BUTTON_PLAY);
  readButNext = digitalRead(PIN_BUTTON_NEXT);


  if (readButPlay != lastButtonPlay) {
    // reset the debouncing timer
    lastDebounceTimePlay = millis();
  }

  if (readButNext != lastButtonNext) {
    // reset the debouncing timer
    lastDebounceTimeNext = millis();
  }


  if ((millis() - lastDebounceTimeNext) > debounceDelay) {
    //test pause/play
    // if the button state has changed:
    if (readButNext != buttonStateNext) {
      buttonStateNext = readButNext;

      // only toggle the LED if the new button state is HIGH
      if (buttonStateNext == HIGH) {
        nextSong++;
        Serial.println("Next Song click");
        if (TEST_LED == 0) {
          if (AUDIO_ACTIVE == 1) {
            change_song();
          }
        } else {
          int_test_led = int_test_led + 10;
          if (int_test_led >= BRIGHTNESS) int_test_led = 0;
          FastLED.setBrightness(int_test_led);
        }
      }
    }
  }


  if ((millis() - lastDebounceTimePlay) > debounceDelay) {
    // whatever the readButPlay is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (readButPlay != buttonStatePlay) {
      buttonStatePlay = readButPlay;

      //detect long press no effect
      // only toggle the LED if the new button state is HIGH
      if (buttonStatePlay == HIGH) {
        Serial.println("Long press PLAY/PAUSE");  //15dB
        if (TEST_GAIN_VOLUME == 0) {
          if (AUDIO_ACTIVE == 1) {
            audio.pauseResume();
          }
        } else {
          int_test_volume++;
          if (int_test_volume >= 5) int_test_volume = 0;

          switch (int_test_volume) {
            case 0:
              Serial.println("gain:3dB");  //GAIN_SLOT=Pull-up 3.3V -> 3 dB
              pinMode(I2S_GAIN, INPUT_PULLUP);
              break;
            case 1:
              Serial.println("gain:6dB");  //GAIN_SLOT=VDD=3.3V -> 6dB
              pinMode(I2S_GAIN, OUTPUT);
              digitalWrite(I2S_GAIN, HIGH);
              break;
            case 2:
              Serial.println("gain:9dB");  //GAIN_SLOT=floating input -> 9dB
              pinMode(I2S_GAIN, INPUT);
              break;
            case 3:
              Serial.println("gain:12dB");  //GAIN_SLOT=GND -> 12dB
              pinMode(I2S_GAIN, OUTPUT);
              digitalWrite(I2S_GAIN, LOW);
              break;
            case 4:
              Serial.println("gain:15dB");  //GAIN_SLOT=Pull-down GND -> 15 dB
              pinMode(I2S_GAIN, INPUT_PULLDOWN);
              break;
            default:
              //none
              Serial.println("gain: no change");
              break;
          }
        }
      }
    }
  }

  lastButtonPlay = readButPlay;
  lastButtonNext = readButNext;
  //END PART button play change power audio


  //refresh volume with led
  if (nowTimeMillis > updateTimeVolume) {
    updateTimeVolume = nowTimeMillis + PERIOD_READ_VOLUME;

    valVolume = 4095 - analogRead(PIN_VOLUME);
    valVolume = (valVolume * 21) / 4095;

    if (valVolume >= 21) valVolume = 21;
    if (valVolume <= 0) valVolume = 0;

    if ((valVolume - valVolumeold) > 1) updatevolume = 1;
    if ((valVolumeold - valVolume) > 1) updatevolume = 1;

    if (updatevolume == 1) {

      Serial.print(valVolume);
      Serial.print(",");
      Serial.println(updatevolume);
      updatevolume = 0;
      valVolumeold = valVolume;
      if (AUDIO_ACTIVE == 1) {
        audio.setVolume(valVolume);  // 0...21
      }

      for (i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB::Black;
      }

      for (i = 0; i < map(valVolume, 0, 21, 0, NUM_LEDS); i++) {
        couleur = CRGB(200, 0, 0);
        leds[i] = couleur;  //rouge
      }
      FastLED.show();

    } else {
      //Serial.println(updatevolume);
      //init all led with OFF or ON
      for (i = 0; i < NUM_LEDS2; i++) {
        if (flip_light >= 1) {
          //4 led lampe
          switch (flip_light) {
            case 1:
              leds2[i] = CRGB(255, 255, 255);  //blanc
              break;
            case 2:
              leds2[i] = CRGB(255, 255, 255);  //blanc
              break;
            case 3:
              leds2[i] = CRGB(255, 255, 255);  //blanc
              break;
            case 4:
              leds2[i] = CRGB(255, 255, 255);  //blanc
              break;
            case 5:
              leds2[i] = CRGB(0, 255, 0);  //vert
              break;
            case 6:
              leds2[i] = CRGB(0, 255, 0);  //vert
              break;
            case 7:
              leds2[i] = CRGB(0, 255, 0);  //vert
              break;
            case 8:
              leds2[i] = CRGB(0, 255, 0);  //vert
              break;
            case 9:
              leds2[i] = CRGB(255, 0, 0);  //rouge
              break;
            case 10:
              leds2[i] = CRGB(255, 0, 0);  //rouge
              break;
            case 11:
              leds2[i] = CRGB(255, 0, 0);  //rouge
              break;
            case 12:
              leds2[i] = CRGB(255, 0, 0);  //rouge
              break;
            case 13:
              leds2[i] = CRGB(0, 0, 255);  //bleu
              break;
            case 14:
              leds2[i] = CRGB(0, 0, 255);  //bleu
              break;
            case 15:
              leds2[i] = CRGB(0, 0, 255);  //bleu
              break;
            case 16:
              leds2[i] = CRGB(0, 0, 255);  //bleu
              break;
            case 17:
              leds2[i] = CRGB(0, 0, 0);  //noir
              break;
            case 18:
              leds2[i] = CRGB(0, 0, 0);  //noir
              break;
            case 19:
              leds2[i] = CRGB(0, 0, 0);  //noir
              break;
            case 20:
              leds2[i] = CRGB(0, 0, 0);  //noir
              break;
            default:
              leds2[i] = CRGB(255, 255, 255);  //blanc
              break;
          }


        } else {
          leds2[i] = CRGB::Black;
        }
      }

      for (i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB::Black;
      }

      if (ulong_time_now >= ulong_time_picture) {
        // if (flip_light == 1) {
        //   leds[numero_led] = CRGB::White;
        // } else {
        //   leds[numero_led] = CRGB::Black;
        // }
      } else {
        //9 leds
        //leds[numero_led] = CRGB::Red;
        switch (flip_light) {
          case 1:
            leds[numero_led] = CRGB(255, 0, 0);  //rouge
            break;
          case 2:
            leds[numero_led] = CRGB(0, 255, 0);  //vert
            break;
          case 3:
            leds[numero_led] = CRGB(0, 0, 255);  //bleu
            break;
          case 4:
            leds[numero_led] = CRGB(255, 255, 255);  //blanc
            break;
          case 5:
            leds[numero_led] = CRGB(255, 0, 0);  //rouge
            break;
          case 6:
            leds[numero_led] = CRGB(0, 255, 0);  //vert
            break;
          case 7:
            leds[numero_led] = CRGB(0, 0, 255);  //bleu
            break;
          case 8:
            leds[numero_led] = CRGB(255, 255, 255);  //blanc
            break;
          case 9:
            leds[numero_led] = CRGB(255, 0, 0);  //rouge
            break;
          case 10:
            leds[numero_led] = CRGB(0, 255, 0);  //vert
            break;
          case 11:
            leds[numero_led] = CRGB(0, 0, 255);  //bleu
            break;
          case 12:
            leds[numero_led] = CRGB(255, 255, 255);  //blanc
            break;
          case 13:
            leds[numero_led] = CRGB(255, 0, 0);  //rouge
            break;
          case 14:
            leds[numero_led] = CRGB(0, 255, 0);  //vert
            break;
          case 15:
            leds[numero_led] = CRGB(0, 0, 255);  //bleu
            break;
          case 16:
            leds[numero_led] = CRGB(255, 255, 255);  //blanc
            break;
          case 17:
            leds[numero_led] = CRGB(255, 0, 0);  //rouge
            break;
          case 18:
            leds[numero_led] = CRGB(0, 255, 0);  //vert
            break;
          case 19:
            leds[numero_led] = CRGB(0, 0, 255);  //bleu
            break;
          case 20:
            leds[numero_led] = CRGB(255, 255, 255);  //blanc
            break;
          default:
            leds[numero_led] = CRGB(255, 0, 0);  //rouge
            break;
        }
      }

      FastLED.show();
    }
  }

  if (AUDIO_ACTIVE == 1) {
    audio.loop();
  }
}


//allow liste automatique quand fin de musique
void audio_eof_mp3(const char *info) {  //end of file
  Serial.print("audio_info: ");
  Serial.println(info);

  nextSong++;
  Serial.println("Next Song autoloop");
  change_song();
}



void change_song(void) {
  switch (intNumeroDossier) {
    case 1:
      if (nextSong > 1) nextSong = 0;
      if (nextSong == 0) audio.connecttoSD("/01/001.mp3");
      if (nextSong == 1) audio.connecttoSD("/01/003.mp3");
      break;
    case 2:
      if (nextSong > 4) nextSong = 0;
      if (nextSong == 0) audio.connecttoSD("/02/001.mp3");
      if (nextSong == 1) audio.connecttoSD("/02/002.mp3");
      if (nextSong == 2) audio.connecttoSD("/02/003.mp3");
      if (nextSong == 3) audio.connecttoSD("/02/004.mp3");
      if (nextSong == 4) audio.connecttoSD("/02/005.mp3");
      break;
    case 3:
      if (nextSong > 3) nextSong = 0;
      if (nextSong == 0) audio.connecttoSD("/03/001.mp3");
      if (nextSong == 1) audio.connecttoSD("/03/002.mp3");
      if (nextSong == 2) audio.connecttoSD("/03/003.mp3");
      if (nextSong == 3) audio.connecttoSD("/03/004.mp3");
      break;
    case 4:
      if (nextSong > 1) nextSong = 0;
      if (nextSong == 0) audio.connecttoSD("/04/001.mp3");
      if (nextSong == 1) audio.connecttoSD("/04/002.mp3");
      break;
    case 5:
      if (nextSong > 0) nextSong = 0;
      if (nextSong == 0) audio.connecttoSD("/05/001.mp3");
      break;
    case 6:
      if (nextSong > 2) nextSong = 0;
      if (nextSong == 0) audio.connecttoSD("/06/001.mp3");
      if (nextSong == 1) audio.connecttoSD("/06/002.mp3");
      if (nextSong == 2) audio.connecttoSD("/06/003.mp3");
      break;
    case 7:
      if (nextSong > 0) nextSong = 0;
      if (nextSong == 0) audio.connecttoSD("/07/001.mp3");
      break;
    case 8:
      if (nextSong > 1) nextSong = 0;
      if (nextSong == 0) audio.connecttoSD("/08/001.mp3");
      if (nextSong == 1) audio.connecttoSD("/08/002.mp3");
      break;
    case 9:
      if (nextSong > 7) nextSong = 0;
      if (nextSong == 0) audio.connecttoSD("/09/001.mp3");
      if (nextSong == 1) audio.connecttoSD("/09/002.mp3");
      if (nextSong == 2) audio.connecttoSD("/09/003.mp3");
      if (nextSong == 3) audio.connecttoSD("/09/004.mp3");
      if (nextSong == 4) audio.connecttoSD("/09/005.mp3");
      if (nextSong == 5) audio.connecttoSD("/09/006.mp3");
      if (nextSong == 6) audio.connecttoSD("/09/007.mp3");
      if (nextSong == 7) audio.connecttoSD("/09/008.mp3");
      break;
    default:
      if (nextSong > 7) nextSong = 0;
      if (nextSong == 0) audio.connecttoSD("/09/001.mp3");
      if (nextSong == 1) audio.connecttoSD("/09/002.mp3");
      if (nextSong == 2) audio.connecttoSD("/09/003.mp3");
      if (nextSong == 3) audio.connecttoSD("/09/004.mp3");
      if (nextSong == 4) audio.connecttoSD("/09/005.mp3");
      if (nextSong == 5) audio.connecttoSD("/09/006.mp3");
      if (nextSong == 6) audio.connecttoSD("/09/007.mp3");
      if (nextSong == 7) audio.connecttoSD("/09/008.mp3");
      break;
  }
}


void logUart(void) {
  if (millis() > updateLogUart) {
    updateLogUart = millis() + PERIOD_LOG_UART;

    Serial.printf("%d,", analogSwitchTheme);
    Serial.printf("%d,", analogSwitchuser);
    Serial.printf("%d,%d,", jackInsertedCnt, jackInserted);
    Serial.printf("%d,", analogBatVoltage);

    Serial.printf("%d,", chargeStatus);

    Serial.print(ESP.getFreePsram());
    Serial.printf("\n");
  }
}


void readBatLevel(void) {
  //Battery voltage adc@3.1V
  analogBatVoltage = analogRead(PIN_BAT_MEAS);  //100k serie 150K
  analogBatVoltage = analogRead(PIN_BAT_MEAS);  //100k serie 150K
  analogBatVoltage = analogRead(PIN_BAT_MEAS);  //100k serie 150K
  analogBatVoltage = analogRead(PIN_BAT_MEAS);  //100k serie 150K
  analogBatVoltage = analogRead(PIN_BAT_MEAS);  //100k serie 150K

  indiceBatVoltage++;
  sumBatVoltage = (sumBatVoltage - matAnalogBatVoltage[indiceBatVoltage % 16]);
  sumBatVoltage = analogBatVoltage + sumBatVoltage;

  matAnalogBatVoltage[indiceBatVoltage % 16] = analogBatVoltage;

  //analogBatVoltage = analogBatVoltage * 100 / 695;  //ratio Vout/Vin=0.6
  analogBatVoltage = (sumBatVoltage >> 4) * 100 / 695;  //ratio Vout/Vin=0.6
}