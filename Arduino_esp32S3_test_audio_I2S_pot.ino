/*
  carte dev
  - esp32 : https://espressif.github.io/arduino-esp32/package_esp32_index.json

  librairies
  - audio I2S : https://github.com/schreibfaul1/ESP32-audioI2S.git
  - neopixel  : https://github.com/FastLED/FastLED.git
  - RTC PCF8563 : https://github.com/adafruit/RTClib.git
  - cryto : https://github.com/rweather/arduinolibs.git

  PSRAM externe : OSPI PSRAM that embed ESP32-S3R8, pins IO35, IO36, and IO37 
  io35 : SD_DAC (I2S audio)
  io36 : SCK_DAC (I2S audio)
  io37 : WS_DAC (I2S audio)
*/

/*
.___ _______  _________ .____     ____ ___________  ___________
|   |\      \ \_   ___ \|    |   |    |   \______ \ \_   _____/
|   |/   |   \/    \  \/|    |   |    |   /|    |  \ |    __)_ 
|   /    |    \     \___|    |___|    |  / |    `   \|        \
|___\____|__  /\______  /_______ \______/ /_______  /_______  /
            \/        \/        \/                \/        \/ 
//INCLUDE
--------------------------------------------------------------------------------------------
*/
#include <Arduino.h>
#include <WiFi.h>
#include <Audio.h>
#include <FastLED.h>
#include "SD.h"
#include "FS.h"
#include "SPIFFS.h"
#include "SPI.h"
#include "time.h"
#include "RTClib.h"
#include <Wire.h>

// #include "SdFat.h"
// #include "Adafruit_TinyUSB.h"

/*
________  ______________________.___ _______  ___________
\______ \ \_   _____/\_   _____/|   |\      \ \_   _____/
 |    |  \ |    __)_  |    __)  |   |/   |   \ |    __)_ 
 |    `   \|        \ |     \   |   /    |    \|        \
/_______  /_______  / \___  /   |___\____|__  /_______  /
        \/        \/      \/                \/        \/ 
//DEFINE
--------------------------------------------------------------------------------------------
*/
// PINOUT SD CARD
#define SD_CS 10
#define SPI_MOSI 11
#define SPI_MISO 13
#define SPI_SCK 12

//PINOUT I2S CARD (AUDIO 3W)
#define I2S_DOUT 45  //35:protoV1  45:protoV2
#define I2S_BCLK 48  //36:protoV1  48:protoV2
#define I2S_LRC 46   //37:protoV1  46:protoV2
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
#define PIN_BUTTON_NEXT 8  //18:dev board  8:protov1

//use for turn off red light indicator after 10s
#define TIME_PICTURE_END 10000  //milliseconds (10s)

//timeout read I2C
#define TIMEOUT_I2C_READ 500  //500ms

//button power and light
#define PIN_POWER_BOARD_SWITCH_LIGHT 7

//TIME long press button light
#define LONG_PRESS 4000  //4 secondes

//potentiometre emotions
#define PIN_INT_SW9 5
#define PIN_RESET_SW9 3         //RESET: LOW  ACTIVE DEVICE:HIGH
#define PI4IOE5V9539_ADDR 0x74  //0x74 0xE8  // Adresse I2C du PI4IOE5V9539
#define PIN_POWER_SW9 40

//neopixel
#define PIN_ENABLE_NEOPIXEL 41  //40:dev board  41:protov1 not exit on dev board only protov1

//boutton switch rotatif (ADC measure)
#define PIN_SWITCH_THEME 2
#define PIN_SWITCH_USER 1

//BATTERY VOLTAGE
#define PIN_BAT_MEAS 14    //not use in dev board
#define PIN_BAT_MEAS_EN 6  //new on versin 2.0

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

//LED NEOPIXEL
#define NUM_LEDS 9
#define NUM_LEDS2 4  ////1: dev board 4:protov1
#define BRIGHTNESS 200
#define DATA_PIN 21
#define DATA_PIN2 21
#define CLOCK_PIN 13  //not use



/*
__________                                     __                       
\______   \_____ ____________    _____   _____/  |_  ___________  ______
 |     ___/\__  \\_  __ \__  \  /     \_/ __ \   __\/ __ \_  __ \/  ___/
 |    |     / __ \|  | \// __ \|  Y Y  \  ___/|  | \  ___/|  | \/\___ \ 
 |____|    (____  /__|  (____  /__|_|  /\___  >__|  \___  >__|  /____  >
                \/           \/      \/     \/          \/           \/ 
//PARAMETERS
--------------------------------------------------------------------------------------------
*/
//WIFI
String ssid = "*****************";
String password = "**********************";

#define WIFI_ACTIVE 0       //default 1  update RTC on NTP server
#define TEST_GAIN_VOLUME 0  //default 0
#define TEST_LED 0          //default 0  1:test led
#define LAMPE_NB_COLOR 1    //default 1  20:mix cobinaison couleur  top/bottom
#define AUDIO_ACTIVE 1      //default 1 (0:PSRAM access)
#define RTC_ACTIVE 0        //default 1
#define DEBUG_UART 1        //default 0


/*
____   _________ __________.___   _____ __________.____     ___________ _________
\   \ /   /  _  \\______   \   | /  _  \\______   \    |    \_   _____//   _____/
 \   Y   /  /_\  \|       _/   |/  /_\  \|    |  _/    |     |    __)_ \_____  \ 
  \     /    |    \    |   \   /    |    \    |   \    |___  |        \/        \
   \___/\____|__  /____|_  /___\____|__  /______  /_______ \/_______  /_______  /
                \/       \/            \/       \/        \/        \/        \/ 
//VARIABLES
--------------------------------------------------------------------------------------------
*/

// // File system on SD Card
// SdFat sd;

// SdFile root;
// SdFile file;

// // USB Mass Storage object
// Adafruit_USBD_MSC usb_msc;

// // Set to true when PC write to flash
// bool fs_changed;

//test variables
int int_test_volume = 0;
int int_test_led = 50;
int i = 0;  //for loop

//audio variables
long int valVolume = 0;
long int valVolumeold = 0;
int updatevolume = 0;
int jackInserted = 0;              //status jack audio
volatile int jackInsertedCnt = 0;  //counter jack status
int changeGain = 0;

//time variables
unsigned long nowTimeMillis = 0;
unsigned long updateTimeVolume = 0;
unsigned long updateTimeGain = 0;
unsigned long updateTimeHorloge = 0;
unsigned long updateAdcRead = 0;
unsigned long updateLogUart = 0;
unsigned long CheckTimeJackInserted = 0;
unsigned long timeoutPressButtonLight = 0;
unsigned long lastDebounceTimePlay = 0;  // the last time the output pin was toggled
unsigned long lastDebounceTimeNext = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;        // the debounce time; increase if the output flickers
unsigned long longPressButton = 1500;    // long time pressure button

//button play/pause next
int readButPlay = 0;
int readButNext = 0;
int buttonStatePlay;        // the current reading from the input pin
int buttonStateNext;        // the current reading from the input pin
int buttonStateLongPress;   // the current reading from the input pin
int lastButtonPlay = HIGH;  // the previous reading from the input pin
int lastButtonNext = HIGH;  // the previous reading from the input pin

int nextSong = 0;

//switch emotion 9 postiions
//switch theme 5 positions
int intDetectExpIoSw9 = 0;
int intCmptRotSw9 = 0;
int intMatSelect[10] = { 0, 8, 16, 32, 64, 128, 256, 512, 1024, 2048 };
int intMatTheme[8] = { 0, 3045, 1939, 3254, 2590, 3406, 2947, 3530 };
int intVarAdc = 10;

//button light
int buttonlightLevel = 0;
int oldButtonlightLevel = 0;


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

// Led neopixel
CRGB leds2[NUM_LEDS2];
CRGB couleur = CRGB(0, 0, 0);

int numero_led = 0;
int flip_light = 0;
int flipLight = 0;
unsigned long ulong_time_now = 0;
unsigned long ulong_time_picture = 0;  //TIME_PICTURE_END

//files
int intNbAudioFileInDir = 0;
int intNumeroDossier = 9;  //emotion
int intthemeChoice = 0;
int intOldNumeroDossier = 0;
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



/*
___________                   __  .__                      
\_   _____/_ __  ____   _____/  |_|__| ____   ____   ______
 |    __)|  |  \/    \_/ ___\   __\  |/  _ \ /    \ /  ___/
 |     \ |  |  /   |  \  \___|  | |  (  <_> )   |  \\___ \ 
 \___  / |____/|___|  /\___  >__| |__|\____/|___|  /____  >
     \/             \/     \/                    \/     \/ 
//functions declaration
--------------------------------------------------------------------------------------------
*/
void change_song(void);
void logUart(void);
int themeSelect(void);
void readBatLevel(void);
void set_tick_tock(void);
void powerOffLed(void);
void powerOnLed(void);
void fadeOutLed(void);
void fadeInLed(void);
void activeLed(int red, int green, int blue, int active);
int readLightButton(void);
int readSwitchEmo(int bypassInt);
void changeDirEmotion(int intDirEmotion);
void changeGainJack(void);

void initSDCard();
void listDir(fs::FS &fs, const char *dirname, uint8_t levels);
void createDir(fs::FS &fs, const char *path);
void removeDir(fs::FS &fs, const char *path);
void readFile(fs::FS &fs, const char *path);
void writeFile(fs::FS &fs, const char *path, const char *message);
void appendFile(fs::FS &fs, const char *path, const char *message);
void renameFile(fs::FS &fs, const char *path1, const char *path2);
void deleteFile(fs::FS &fs, const char *path);
void testFileIO(fs::FS &fs, const char *path);

void printLocalTime();

void msc_flush_cb(void);
int32_t msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize);
int32_t msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize);


//INTERRUPT jack audio inserted
void jackChangeInterrupt() {
  jackInsertedCnt++;
  jackInserted = 0;
  CheckTimeJackInserted = millis() + PERIOD_JACK_DETECT;
  detachInterrupt(digitalPinToInterrupt(PIN_ADC_JACK_DETECT));
}


//INTERRUPT expander io switch 9 positions
void intExpIoSw9() {
  intCmptRotSw9++;
  intDetectExpIoSw9 = 1;
  detachInterrupt(digitalPinToInterrupt(PIN_INT_SW9));
}



/*
  ___________________________________ _____________ 
 /   _____/\_   _____/\__    ___/    |   \______   \
 \_____  \  |    __)_   |    |  |    |   /|     ___/
 /        \ |        \  |    |  |    |  / |    |    
/_______  //_______  /  |____|  |______/  |____|    
        \/         \/                               
//setup init
--------------------------------------------------------------------------------------------
*/
void setup() {
  //auto maintain power of the board when start
  //DO activate when R22 100K remove
  pinMode(PIN_POWER_BOARD_SWITCH_LIGHT, OUTPUT);
  digitalWrite(PIN_POWER_BOARD_SWITCH_LIGHT, HIGH);  //high:power ON switch9 low:power OFF switch9
  delay(1000);

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

  //I2C expanderIO and RTC
  Wire.begin(SDA_PIN_RTC_PCF8563, SCL_PIN_RTC_PCF8563);  //SDA SCL

  //I2C RTC
  if (RTC_ACTIVE == 1) {
    pinMode(IRQ_PIN_RTC_PCF8563, INPUT);      // set up interrupt pin
    digitalWrite(IRQ_PIN_RTC_PCF8563, HIGH);  // turn on pullup resistors
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

  //INIT batterie read voltage pin
  pinMode(PIN_BAT_MEAS_EN, OUTPUT);
  digitalWrite(PIN_BAT_MEAS_EN, HIGH);  //HIGH:read bat off LOW:read bat on

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
  Serial.println(ESP.getFreePsram());

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
  powerOnLed();

  //init 4 leds
  FastLED.addLeds<NEOPIXEL, DATA_PIN2>(leds2, NUM_LEDS2);  // GRB ordering is assumed
  FastLED.setBrightness(BRIGHTNESS);

  //start animation
  fadeInLed();

  //not present on v2.0
  // Serial.println("init rotary");
  // pinMode(ROTARY_ENCODER_A_PIN, INPUT_PULLUP);
  // pinMode(ROTARY_ENCODER_B_PIN, INPUT_PULLUP);
  // pinMode(ROTARY_ENCODER_BUTTON_PIN, INPUT_PULLUP);

  // //we must initialize rotary encoder
  // rotaryEncoder.begin();
  // rotaryEncoder.setup(readEncoderISR);
  // //set boundaries and if values should cycle or not
  // //in this example we will set possible values between 0 and 1000;
  // bool circleValues = true;
  // rotaryEncoder.setBoundaries(0, CYCLE_ROT, circleValues);  //minValue, maxValue, circleValues true|false (when max go to min and vice versa)

  // /*Rotary acceleration introduced 25.2.2021.
  //    in case range to select is huge, for example - select a value between 0 and 1000 and we want 785
  //    without accelerateion you need long time to get to that number
  //    Using acceleration, faster you turn, faster wil
  //    l the value raise.
  //    For fine tuning slow down.
  // */
  // //rotaryEncoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
  // rotaryEncoder.setAcceleration(1);  //250 or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration

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
  if (DEBUG_UART == 1) {
    // delay(2000);
    Serial.println("Enable AUDIO");
  }
  pinMode(PIN_ENABLE_I2S, INPUT);  // floatting left+right/2
  //new board 2.0
  //pinMode(PIN_ENABLE_I2S, OUTPUT);
  //digitalWrite(PIN_ENABLE_I2S, HIGH);

  if (DEBUG_UART == 1) {
    // delay(2000);
    Serial.println("active read AUDIO");
  }
  if (AUDIO_ACTIVE == 1) {
    Serial.println("set pinout");
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    if (DEBUG_UART == 1) {
      //delay(2000);
      Serial.println("set balance");
    }
    audio.setBalance(-16);  // mutes the left channel
    if (DEBUG_UART == 1) {
      //delay(2000);
      Serial.println("set volume 0");
    }
    audio.setVolume(0);
    if (DEBUG_UART == 1) {
      //delay(2000);
      Serial.println("set mono true");
    }
    audio.forceMono(true);  //mono application

    //stream music
    //audio.connecttohost("http://vis.media-ice.musicradio.com/CapitalMP3");

    //sd musique
    //audio.connecttoFS(SD, "/04/001.mp3");
    if (DEBUG_UART == 1) {
      //delay(2000);
      Serial.println("audio connect mp3");
    }
    audio.connecttoSD("/04/001.mp3");

    if (DEBUG_UART == 1) {
      //delay(2000);
      Serial.println("pause");
    }
    audio.pauseResume();
  }

  if (DEBUG_UART == 1) {
    //delay(2000);
    Serial.println("buton play next");
  }
  //int button play next
  pinMode(PIN_BUTTON_PLAY, INPUT_PULLUP);
  pinMode(PIN_BUTTON_NEXT, INPUT_PULLUP);


  if (DEBUG_UART == 1) {
    //delay(2000);
    Serial.println("jack detect");
  }
  //pullup jack detect
  pinMode(PIN_ADC_JACK_DETECT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ADC_JACK_DETECT), jackChangeInterrupt, CHANGE);

  if (DEBUG_UART == 1) {
    //delay(2000);
    Serial.println("switch emotion");
  }
  //init pin switch 9 emotions
  pinMode(PIN_RESET_SW9, OUTPUT);
  digitalWrite(PIN_RESET_SW9, HIGH);  //high:enable expander low:disable expander (reset)
  pinMode(PIN_POWER_SW9, OUTPUT);
  digitalWrite(PIN_POWER_SW9, HIGH);  //high:power ON switch9 low:power OFF switch9
  delay(1);

  //interrupt expander IO SW9
  pinMode(PIN_INT_SW9, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_INT_SW9), intExpIoSw9, FALLING);

  if (DEBUG_UART == 1) {
    //delay(2000);
    Serial.println("wifi disconnect");
  }
  //turn off wifi
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  if (DEBUG_UART == 1) {
    //delay(2000);
    Serial.println("read battery");
  }
  //battery level
  for (i = 0; i < 16; i++) {
    readBatLevel();
  }

  if (DEBUG_UART == 1) {
    //delay(2000);
    Serial.println("exit setup -> loop");
    // while (true)
    //   ;
  }

  //init switch 9 pos emotion
  intNumeroDossier = readSwitchEmo(1);  //1 direct read  0:wait interrupt for reading

  //   // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
  //   usb_msc.setID("Mandalou", "SDCard", "1.0");


  //   // Set read write callback
  //   usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);

  //   // Still initialize MSC but tell usb stack that MSC is not ready to read/write
  //   // If we don't initialize, board will be enumerated as CDC only
  //   usb_msc.setUnitReady(false);
  //   usb_msc.begin();

  //   if (!sd.begin(SD_CS, SD_SCK_MHZ(50))) {
  //     Serial.println("initialization failed. Things to check:");
  //     Serial.println("* is a card inserted?");
  //     Serial.println("* is your wiring correct?");
  //     Serial.println("* did you change the chipSelect pin to match your shield or module?");
  //     while (1) delay(1);
  //   }

  //   // Size in blocks (512 bytes)
  // #if SD_FAT_VERSION >= 20000
  //   uint32_t block_count = sd.card()->sectorCount();
  // #else
  //   uint32_t block_count = sd.card()->cardSize();
  // #endif

  //   Serial.print("Volume size (MB):  ");
  //   Serial.println((block_count / 2) / 1024);

  //   // Set disk size, SD block size is always 512
  //   usb_msc.setCapacity(block_count, 512);

  //   // MSC is ready for read/write
  //   usb_msc.setUnitReady(true);

  //   fs_changed = true;  // to print contents initially
}



/*
.____    ________   ________ __________ 
|    |   \_____  \  \_____  \\______   \
|    |    /   |   \  /   |   \|     ___/
|    |___/    |    \/    |    \    |    
|_______ \_______  /\_______  /____|    
        \/       \/         \/          
//main loop
--------------------------------------------------------------------------------------------
*/
void loop() {
  nowTimeMillis = millis();
  ulong_time_now = millis();

  //change gain depending jack inserted
  //changeGainJack();

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
    //Serial.printf("Switch theme = %d\n", analogSwitchTheme);
    intthemeChoice = themeSelect(analogSwitchTheme);

    // switch user
    analogSwitchuser = analogRead(PIN_SWITCH_USER);
    analogSwitchuser = analogRead(PIN_SWITCH_USER);
    analogSwitchuser = analogRead(PIN_SWITCH_USER);
    //Serial.printf("Switch user = %d\n", analogSwitchuser);

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

  //read switch 9 pos emotion
  intNumeroDossier = readSwitchEmo(0);  //1 direct read  0:wait interrupt for reading
  if (intNumeroDossier != intOldNumeroDossier) {
    intOldNumeroDossier = intNumeroDossier;
    changeDirEmotion(intNumeroDossier);
  }

  //read button light on pwer ohh if long press
  buttonlightLevel = readLightButton();

  if (buttonlightLevel != oldButtonlightLevel) {

    if (oldButtonlightLevel == 0 && buttonlightLevel == 1) {
      flipLight = flipLight ^ 1;
      if (flipLight == 1)
        activeLed(0, 150, 0, 1);  // red,  green,  blue,  active
      else
        activeLed(0, 0, 0, 0);  // red,  green,  blue,  active
      //Serial.println(flipLight);
    } else {
      //Serial.println(buttonlightLevel);
    }

    oldButtonlightLevel = buttonlightLevel;

    if (oldButtonlightLevel == 2) {
      //power off
      //mandalou off audio
      //audio.connecttoSD("/ADVERT/0004.mp3");  //bug audio with delay bloc

      //led off
      fadeOutLed();

      Serial.println("POWER OFF");
      for (int ii = 10; ii >= 0; ii--) {
        Serial.println(ii);
        delay(1000);
      }
      Serial.println("OFF");
      pinMode(PIN_POWER_BOARD_SWITCH_LIGHT, OUTPUT);
      //digitalWrite(PIN_POWER_BOARD_SWITCH_LIGHT, LOW);  //high:power ON switch9 low:power OFF switch9
      delay(20);
    }
  }


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
    //valVolume = (valVolume * 21) / 4095;

    valVolume = map(valVolume, 0, 4095, 0, 21);

    if (valVolume >= 21) valVolume = 21;
    if (valVolume <= 0) valVolume = 0;

    if ((valVolume - valVolumeold) > 1) updatevolume = 1;
    if ((valVolumeold - valVolume) > 1) updatevolume = 1;

    if (updatevolume == 1) {

      // Serial.print(valVolume);
      // Serial.print(",");
      // Serial.println(updatevolume);
      updatevolume = 0;
      valVolumeold = valVolume;
      if (AUDIO_ACTIVE == 1) {
        audio.setVolume(valVolume);  // 0...21
      }

    } else {
      //no update volume
    }
  }

  if (AUDIO_ACTIVE == 1) {
    audio.loop();
  }
}  //loop



void initSDCard() {
  if (!SD.begin(SD_CS)) {
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

  updateTimeHorloge = millis() + (60 - timeinfo.tm_sec) * 1000;

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



int themeSelect(int adcValue) {
  static int choiceDetect = 0;

  for (int kk = 0; kk < 8; kk++) {
    if (((intMatTheme[kk] - intVarAdc) < adcValue) && adcValue < (intMatTheme[kk] + intVarAdc)) {
      choiceDetect = kk;
      break;
    }
  }
  return choiceDetect;
}



void logUart(void) {
  //adc theme  | theme1-5 |  ADC user | led_status | emotion |volume0-21 | jack cnt  | status jack insert | batterie voltage | charge status | PSRAM
  if (millis() > updateLogUart) {
    updateLogUart = millis() + PERIOD_LOG_UART;

    Serial.printf("%d,", analogSwitchTheme);
    //Méditation Yoga Musique Histoire Bruit Blanc
    Serial.printf("%d,", intthemeChoice);

    Serial.printf("%d,", analogSwitchuser);

    Serial.printf("%d,", flipLight);  //led status

    Serial.printf("%d,", intNumeroDossier);  //emotion



    Serial.printf("%d,", valVolume);
    Serial.printf("%d,%d,", jackInsertedCnt, jackInserted);

    Serial.printf("%d,", analogBatVoltage);
    Serial.printf("%d,", chargeStatus);

    Serial.print(ESP.getFreePsram());
    Serial.printf("\n");
  }
}


void readBatLevel(void) {
  //enable read battery
  digitalWrite(PIN_BAT_MEAS_EN, HIGH);  //HIGH:read bat off LOW:read bat on
  //delay(1);

  //Battery voltage adc@3.1V
  analogBatVoltage = analogRead(PIN_BAT_MEAS);  //100k serie 150K
  analogBatVoltage = analogRead(PIN_BAT_MEAS);  //100k serie 150K
  analogBatVoltage = analogRead(PIN_BAT_MEAS);  //100k serie 150K
  analogBatVoltage = analogRead(PIN_BAT_MEAS);  //100k serie 150K
  analogBatVoltage = analogRead(PIN_BAT_MEAS);  //100k serie 150K

  //disable read bat
  //digitalWrite(PIN_BAT_MEAS_EN, LOW);  //HIGH:read bat off LOW:read bat on

  indiceBatVoltage++;
  sumBatVoltage = (sumBatVoltage - matAnalogBatVoltage[indiceBatVoltage % 16]);
  sumBatVoltage = analogBatVoltage + sumBatVoltage;

  matAnalogBatVoltage[indiceBatVoltage % 16] = analogBatVoltage;

  //analogBatVoltage = analogBatVoltage * 100 / 695;  //ratio Vout/Vin=0.6
  analogBatVoltage = (sumBatVoltage >> 4) * 100 / 695;  //ratio Vout/Vin=0.6
}



void fadeInLed(void) {
  powerOnLed();
  for (int ii = 10; ii < 150; ii = ii + 10) {
    for (i = 0; i < NUM_LEDS2; i++) {
      leds2[i] = CRGB(0, 0, ii);
    }
    FastLED.show();
    delay(100);
  }
  delay(1000);
  for (i = 0; i < NUM_LEDS2; i++) {
    leds2[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
  delay(10);
  powerOffLed();
}



void fadeOutLed(void) {
  powerOnLed();
  for (i = 0; i < NUM_LEDS2; i++) {
    leds2[i] = CRGB(0, 0, 150);
  }
  FastLED.show();
  delay(1000);
  for (int ii = 140; ii >= 0; ii = ii - 10) {

    for (i = 0; i < NUM_LEDS2; i++) {
      leds2[i] = CRGB(0, 0, ii);
    }
    FastLED.show();
    delay(100);
  }
  powerOffLed();
}


void activeLed(int red, int green, int blue, int active) {
  for (i = 0; i < NUM_LEDS2; i++) {
    if (active == 1) {
      powerOnLed();
      leds2[i] = CRGB(red, green, blue);
    } else {
      powerOffLed();
    }
  }
  delay(10);
  FastLED.show();
}


void powerOnLed(void) {
  //power supply led neopixel
  pinMode(PIN_ENABLE_NEOPIXEL, OUTPUT);
  digitalWrite(PIN_ENABLE_NEOPIXEL, LOW);  //HIGH:led off LOW:led on
  delay(2);
}


void powerOffLed(void) {
  //power supply led neopixel
  pinMode(PIN_ENABLE_NEOPIXEL, OUTPUT);
  digitalWrite(PIN_ENABLE_NEOPIXEL, HIGH);  //HIGH:led off LOW:led on
}


// INT0 interrupt callback; update tick_tock flag
void set_tick_tock(void) {
  tick_tock = 1;
}



int readLightButton(void) {
  static int firstPress = 1;
  int lightLevel = 0;
  //activate detection button
  pinMode(PIN_POWER_BOARD_SWITCH_LIGHT, INPUT);
  lightLevel = digitalRead(PIN_POWER_BOARD_SWITCH_LIGHT);

  //allow power
  pinMode(PIN_POWER_BOARD_SWITCH_LIGHT, OUTPUT);
  digitalWrite(PIN_POWER_BOARD_SWITCH_LIGHT, HIGH);  //high:power ON switch9 low:power OFF switch9

  if (lightLevel == 0) {
    //button realize
    firstPress = 1;
    return 0;
  } else {
    //button press
    if (firstPress == 1) {
      firstPress = 0;
      timeoutPressButtonLight = millis() + LONG_PRESS;
    }

    if (millis() > timeoutPressButtonLight) { return 2; }

    return 1;
  }
}



int readSwitchEmo(int bypassInt) {
  byte error;
  byte inputPort0 = 0;
  byte inputPort1 = 0;
  int highByte = 0;
  int lowByte = 0;
  int value = 0;
  unsigned long timeoutI2c = 0;
  int sw9 = 0;
  int selection_emo = 0;

  //detect if interrupt active
  if (bypassInt == 1) {

    if (intDetectExpIoSw9 == 1) {
      intDetectExpIoSw9 = 0;
      delay(2);
      attachInterrupt(digitalPinToInterrupt(PIN_INT_SW9), intExpIoSw9, FALLING);
    }

    Serial.print("int SW9:");
    Serial.println(intCmptRotSw9);

    // Commencer la communication avec l'esclave à l'adresse 0x74
    Wire.beginTransmission(PI4IOE5V9539_ADDR);
    // Sélectionner le registre 0x00
    Wire.write(0x00);
    // Terminer la transmission
    Wire.endTransmission();

    // Demander 2 octets de données à l'esclave
    Wire.requestFrom(PI4IOE5V9539_ADDR, 2);

    // Attendre que les données soient disponibles
    //0.5ms lecture pot 9 pos
    timeoutI2c = millis() + TIMEOUT_I2C_READ;
    while (Wire.available() < 2) {
      if (millis() > timeoutI2c) {
        break;  //non bloquant
      }
    }

    // Lire les octets et les combiner pour former une valeur de 16 bits
    highByte = Wire.read();
    lowByte = Wire.read();
    value = (highByte << 8) | lowByte;

    selection_emo = 0;
    for (sw9 = 0; sw9 < 10; sw9++) {
      if (value == intMatSelect[sw9]) {
        selection_emo = sw9;
        break;
      }
    }

    if (selection_emo == 0)
      selection_emo = intNumeroDossier;

    // Afficher la valeur lue
    Serial.print("Selection : ");
    Serial.println(selection_emo);
    return selection_emo;


  } else {
    //read if int occur
    if (intDetectExpIoSw9 == 1) {
      intDetectExpIoSw9 = 0;
      delay(2);
      attachInterrupt(digitalPinToInterrupt(PIN_INT_SW9), intExpIoSw9, FALLING);

      Serial.print("int SW9:");
      Serial.println(intCmptRotSw9);

      // Commencer la communication avec l'esclave à l'adresse 0x74
      Wire.beginTransmission(PI4IOE5V9539_ADDR);
      // Sélectionner le registre 0x00
      Wire.write(0x00);
      // Terminer la transmission
      Wire.endTransmission();

      // Demander 2 octets de données à l'esclave
      Wire.requestFrom(PI4IOE5V9539_ADDR, 2);

      // Attendre que les données soient disponibles
      //0.5ms lecture pot 9 pos
      timeoutI2c = millis() + TIMEOUT_I2C_READ;
      while (Wire.available() < 2) {
        if (millis() > timeoutI2c) {
          break;  //non bloquant
        }
      }

      // Lire les octets et les combiner pour former une valeur de 16 bits
      highByte = Wire.read();
      lowByte = Wire.read();
      value = (highByte << 8) | lowByte;

      selection_emo = 0;
      for (sw9 = 0; sw9 < 10; sw9++) {
        if (value == intMatSelect[sw9]) {
          selection_emo = sw9;
          break;
        }
      }

      if (selection_emo == 0)
        selection_emo = intNumeroDossier;

      // Afficher la valeur lue
      Serial.print("Selection : ");
      Serial.println(selection_emo);

      return selection_emo;
    }
    return intNumeroDossier;
  }
}


void changeDirEmotion(int intDirEmotion) {
  Serial.print("Dossier EMOTION:");
  Serial.println(intDirEmotion);

  sprintf(name_directory, "/%02d", intDirEmotion);
  intNbAudioFileInDir = 0;
  //listDir(SD, "/05", 1);
  listDir(SD, name_directory, 1);

  nextSong = 0;

  switch (intDirEmotion) {
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
}



void changeGainJack(void) {
  if (jackInserted == 1) {
    //3dB casque
    pinMode(I2S_GAIN, OUTPUT);
    digitalWrite(I2S_GAIN, HIGH);
  } else {
    //12db haut parleur
    pinMode(I2S_GAIN, INPUT);
    //3dB casque
    pinMode(I2S_GAIN, OUTPUT);
    digitalWrite(I2S_GAIN, HIGH);
  }
}

/*
// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
int32_t msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize) {
  bool rc;

#if SD_FAT_VERSION >= 20000
  rc = sd.card()->readSectors(lba, (uint8_t *)buffer, bufsize / 512);
#else
  rc = sd.card()->readBlocks(lba, (uint8_t *)buffer, bufsize / 512);
#endif

  return rc ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
int32_t msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize) {
  bool rc;

  // digitalWrite(LED_BUILTIN, HIGH);

#if SD_FAT_VERSION >= 20000
  rc = sd.card()->writeSectors(lba, buffer, bufsize / 512);
#else
  rc = sd.card()->writeBlocks(lba, buffer, bufsize / 512);
#endif

  return rc ? bufsize : -1;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
void msc_flush_cb(void) {
#if SD_FAT_VERSION >= 20000
  sd.card()->syncDevice();
#else
  sd.card()->syncBlocks();
#endif

  // clear file system's cache to force refresh
  //sd.cacheClear();

  fs_changed = true;

  // digitalWrite(LED_BUILTIN, LOW);
}
*/
