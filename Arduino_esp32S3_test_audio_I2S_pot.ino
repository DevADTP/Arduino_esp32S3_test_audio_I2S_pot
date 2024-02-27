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

#include "SdFat.h"
#include "Adafruit_TinyUSB.h"

#include <esp_task_wdt.h>
#include <ArduinoJson.h>
#include <Preferences.h>

#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>

#include "mbedtls/aes.h"

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

// PINOUT SD CARD USB stick
#define SD_CS2 10
#define SPI_MOSI2 11
#define SPI_MISO2 13
#define SPI_SCK2 12

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

//TIME medium press button next random
#define MEDIUM_PRESS 2000  //2 secondes

//TIME refresh led usb mode
#define UPDATE_LED 100  //100ms

//TIMEOUT AUTOOFF
#define TIME_AUTO_OFF 3600000  //10min=600000ms  //1H=3600000ms  //8h=28800000ms

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

//time watchdog
#define WDT_TIMEOUT 10  //2 seconds WDT



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

//key AES128bit 16Bytes
#define KEY_AES128 "abcdefghijklmnop"
#define AES_BLOCK_SIZE 16

#define WIFI_ACTIVE 0       //default 1  update RTC on NTP server
#define TEST_GAIN_VOLUME 0  //default 0
#define TEST_LED 0          //default 0  1:test led
#define LAMPE_NB_COLOR 1    //default 1  20:mix cobinaison couleur  top/bottom
#define AUDIO_ACTIVE 1      //default 1 (0:PSRAM access)
#define RTC_ACTIVE 0        //default 1
#define DEBUG_UART 0        //default 0
#define WEBSOCKET 0         //default 0


#define BATT_ULTRA_LOW 320  //batt ultra low level light  322 -> 3,5V  317 -> 3,48V ADC stable  RED STOP
#define BATT_LOW 329        //batt low level light        329 -> 3,56V  8%  RED
#define BATT_MEDIUM 340     //batt low level light        364 -> 3,93V 30%  ORANGE
#define BATT_HIGH 389       //battery high level light    389 -> 4,2V  100% GREEN
#define BATT_HYST 2



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
// File system on SD Card
SdFat sd;
SdFile root;
SdFile file;
// USB Mass Storage object
Adafruit_USBD_MSC usb_msc;
// Set to true when PC write to flash
bool fs_changed;

//Memoire RTC mode low power et reboot
//RTC_DATA_ATTR int bootMode = 0;  //0:normal audio 1:usb key
int localBootMode = 0;

static unsigned int bootMode2 = 0;

//test variables
int int_test_volume = 0;
int int_test_led = 50;
int i = 0;   //for loop
int ii = 0;  //for loop
int jj = 0;  //for loop

//audio variables
long int valVolume = 0;
long int valVolumeold = 0;
int updatevolume = 0;
int jackInserted = 0;     //status jack audio
int oldJackInserted = 1;  //status jack audio

int intJackInserted = 0;           //interrupt active
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
unsigned long oldCheckTimeJackInserted = 0;
unsigned long timeoutPressButtonLight = 0;
unsigned long lastDebounceTimePlay = 0;      // the last time the output pin was toggled
unsigned long lastDebounceTimeNext = 0;      // the last time the output pin was toggled
unsigned long lastDebounceTimePlayNext = 0;  // the last time the output pin was toggled
unsigned long lastLongPressTimeNext = 0;     // the last time the output pin was toggled
int longPressPlayNext = 2000;                // press PLAY NEXT during 2 secondes
unsigned long debounceDelay = 50;            // the debounce time; increase if the output flickers
unsigned long longPressButton = 1500;        // long time pressure button
unsigned long timeoutRefreshLed = 0;         //timeout update led refresh
unsigned long timeoutAutoOff = 0;            //timeout auto off if not touch TIME_AUTO_OFF

unsigned long timeAutoOff = TIME_AUTO_OFF;

unsigned long timeDecodeCrypt = 0;  //time measure decrytpage



//button play/pause next
int readButPlay = 0;
int readButNext = 0;
int buttonStatePlay;              // the current reading from the input pin
int buttonStateNext;              // the current reading from the input pin
int buttonStateNextLongPress;     // the current reading from the input pin
int buttonStateLongPress;         // the current reading from the input pin
int lastButtonPlay = HIGH;        // the previous reading from the input pin
int lastButtonNext = HIGH;        // the previous reading from the input pin
int lastButtonNextRandom = HIGH;  // the previous reading from the input pin

int nextSong = 0;
int modeRandNorm = 1;  //0:normal  1:random
int backupModeRandom_sleep=1;  //sleep only normal not random

//switch emotion 9 positions
//switch theme 5 positions
int intDetectExpIoSw9 = 0;
int intCmptRotSw9 = 0;

int intMatSelect[10] = { 0, 8, 16, 32, 64, 128, 256, 512, 1024, 2048 };

/*
0     1   colere        
8     2   tristesse     
16    3   peur            
32    4   anxiete    -> mon moment a moi    
64    5   jalousie      
128   6   joie        
256   7   fatigue    -> S'endormir  
512   8   reveil     -> Se reveiller  
1024  9   contenue libre 
*/

//seuil ADC theme generique

//carte 01 S45-2023 -> Dev board cirly  manque mos batterie (conso permanante 16.8µA 4.2V@250kohms)
//int intMatTheme[8] = { 0, 3045, 1939, 8000, 9000, 10000, 2950, 3532 };  //0 1 2 . . . 6 7

//carte 02 S45-2023 -> Dev board cirly
//int intMatTheme[8] = { 0, 3045, 1939, 8000, 9000, 10000, 2950, 3532 };  //0 1 2 . . . 6 7


//carte 01 S48-2023 -> livraison 25/01/2024  -> problem boot button   ****************** OK DIODE RESOLVE PROBLEM ****************************
//int intMatTheme[8] = { 0, 3015, 1922, 8000, 9000, 10000, 2920, 3492 };  //0 1 2 . . . 6 7

//carte 02 S48-2023 -> Dev board
//int intMatTheme[8] = { 0, 3021, 1927, 8000, 9000, 10000, 2923, 3495 };  //0 1 2 . . . 6 7

//carte 03 S48-2023 -> livraison
//int intMatTheme[8] = { 0, 3143, 2007, 8000, 9000, 10000, 3041, 3611 };  //0 1 2 . . . 6 7

//carte 04 S48-2023 -> livraison 25/01/2024
//int intMatTheme[8] = { 0, 3109, 1980, 8000, 9000, 10000, 3012, 3570 };  //0 1 2 . . . 6 7

//carte 05 S48-2023 -> livraison
//int intMatTheme[8] = { 0, 3143, 2008, 8000, 9000, 10000, 3035, 3605 };  //0 1 2 . . . 6 7

//carte 06 S48-2023 -> livraison 25/01/2024
//int intMatTheme[8] = { 0, 3123, 1992, 8000, 9000, 10000, 3020, 3606 };  //0 1 2 . . . 6 7

//carte 07 S48-2023 -> livraison 25/01/2024 -> probleme CC            ******************OK BOF CC carte SD  PROBLEM CC ************************
//int intMatTheme[8] = { 0, 3087, 1970, 8000, 9000, 10000, 2989, 3548 };  //0 1 2 . . . 6 7

//carte 08 S48-2023 -> livraison 25/01/2024  -> problem boot button   ****************** OK DIODE RESOLVE PROBLEM ****************************
int intMatTheme[8] = { 0, 3089, 1971, 8000, 9000, 10000, 2987, 3553 };  //0 1 2 . . . 6 7

//carte 09 S48-2023 -> livraison
//int intMatTheme[8] = { 0, 3150, 2013, 8000, 9000, 10000, 3050, 3628 };  //0 1 2 . . . 6 7

//carte 10 S48-2023 -> livraison 25/01/2024  -> problem boot button   ****************** OK DIODE RESOLVE PROBLEM ****************************
//int intMatTheme[8] = { 0, 3191, 2037, 8000, 9000, 10000, 3094, 3652 };  //0 1 2 . . . 6 7

//carte 11 S48-2023 -> livraison 25/01/2024
//int intMatTheme[8] = { 0, 3083, 1966, 8000, 9000, 10000, 2983, 3547 };  //0 1 2 . . . 6 7


//ADC flucutation for thema
int intVarAdc = 50;  //ecart adc 10 -> 50

//info device
long int numeroSerie = 2023539999;
int verSw = 0101;
int verHw = 0201;
int gainHp = 1;
int gainCasq = 1;

//debug uart
int uartdbg = 1;

//button light
int buttonlightLevel = 0;
int oldButtonlightLevel = 0;
int lightLevel = 0;

//adc
int analogSwitchTheme = 0;
int analogSwitchuser = 0;
int analogJackInserted = 0;
long int analogBatVoltage = 0;  //PIN_BAT_MEAS
int matAnalogBatVoltage[16];
long int sumBatVoltage = 0;
int adcBatStable = 0;
int indiceBatVoltage = 0;     //mean calculation
int battLightLevel = 4;       //light batt level 4:fullfull 3:full 2:medium 1:low 0:ultra low
int battLightLevelOld = -10;  //for update light led if batt level change

int battUltraLowLevel = BATT_ULTRA_LOW;
int battLowLevel = BATT_LOW;
int battMedLevel = BATT_MEDIUM;
int battHighLevel = BATT_HIGH;
int batHystLevel = BATT_HYST;

//battery
int chargeStatus = 0;
int oldChargeStatus = 0;
int firstChargeSatatus = 1;

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
int intNumeroDossier = 9;  //emotion 9:Contenu libre
int intthemeChoice = 1;    //1:Méditation
int intOldthemeChoice = 1;
int intOldNumeroDossier = 0;
int intNombreDossier = 9;

char name_directory[200] = "/09";

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

//Json for config
DynamicJsonDocument jsonConfig(1024);
String jsonString;
String receivedUartConfig = "";
int updateConfig = 0;

//preference for boot
Preferences preferences;


//websocket debug
String jsonStringws;
DynamicJsonDocument jsonDoc(1024);
AsyncWebServer server(8080);
AsyncWebSocket wbsoc("/ws");


//encryption
#ifdef _MODIF_SULSULDEV
#else
mbedtls_aes_context aes;
char *keycrypt = KEY_AES128;
#endif

unsigned char decrypredData[16] = { 0 };
unsigned char encrypredData[16] = { 0 };
unsigned char decrypredDataBig[2048] = { 0 };
unsigned char encrypredDataBig[2048] = { 0 };
uint32_t nrOfDecryptedBytes = 16;
char strcrypt[3];

//encryption
// mbedtls_aes_context aes;
// char *keycrypt = KEY_AES128;

char *inputcrypt = "Mandalou - crypt";  //16bytes packet
unsigned char outputcrypt[16] = { 0 };
//char strcrypt[3];


void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("Client connecté");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("Client déconnecté");
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      // Message texte reçu
      data[len] = 0;
      Serial.printf("Données reçues: %s\n", data);
    }
  }
}



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
void change_song_name(void);
void logUart(int modeuart);
void uartConfig(void);
int themeSelect(void);
void readBatLevel(void);
void ledBatteryLevel(void);
void set_tick_tock(void);
void powerOffLed(void);
void powerOnLed(void);
void fadeOutLed(void);
void fadeInLed(void);
void ledlight(void);
void animateledRot(void);
void animateledFlip(void);
void animateLedChargeComplete(void);
void fadeEffect(int percent_red, int percent_green, int percent_blue, int timeWait, int cycle);
void animateLedInCharge(void);
void activeLed(int red, int green, int blue, int brighness, int active);
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

int choisirFichierAleatoire(const char *cheminDossier);

void printLocalTime();

void msc_flush_cb(void);
int32_t msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize);
int32_t msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize);

//setup and loop modes
void setup_usb();
void loop_usb();
void setup_veilleuse();
void loop_veilleuse();

void print_wakeup_reason();

//websocket
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

//encryption
void decryptFile(const char *encryptedFileName, const char *decryptedFileName);

//file classification
bool hasMp3Extension(const char *filename);
int fileClassifiation(void);
bool containsSubstring(const char *filename, const char *substring);



/*
.___        __                                    __          
|   | _____/  |_  __________________ __ _________/  |_  ______
|   |/    \   __\/ __ \_  __ \_  __ \  |  \____ \   __\/  ___/
|   |   |  \  | \  ___/|  | \/|  | \/  |  /  |_> >  |  \___ \ 
|___|___|  /__|  \___  >__|   |__|  |____/|   __/|__| /____  >
         \/          \/                   |__|             \/ 
//interrupt functions
--------------------------------------------------------------------------------------------
*/
//INTERRUPT jack audio inserted
void jackChangeInterrupt() {
  //jackInsertedCnt++;
  intJackInserted = 1;
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
               __                
  ______ _____/  |_ __ ________  
 /  ___// __ \   __\  |  \____ \ 
 \___ \\  ___/|  | |  |  /  |_> >
/____  >\___  >__| |____/|   __/ 
     \/     \/           |__|    
//Init main application
--------------------------------------------------------------------------------------------
*/
void setup() {
  //auto maintain power of the board when start
  //DO activate when R22 100K remove
  // pinMode(PIN_POWER_BOARD_SWITCH_LIGHT, INPUT);
  // delay(200);

  pinMode(PIN_POWER_BOARD_SWITCH_LIGHT, OUTPUT);
  digitalWrite(PIN_POWER_BOARD_SWITCH_LIGHT, HIGH);  //high:power ON switch9 low:power OFF switch9
  delay(200);

  //++bootMode;  //flip
  //++bootMode2;

  //preference on flash data etain after reboot
  preferences.begin("my-app", false);
  // Remove all preferences under the opened namespace
  //preferences.clear();
  // Or remove the counter key only
  //preferences.remove("counter");
  // Get the counter value, if the key does not exist, return a default value of 0
  // Note: Key name is limited to 15 chars.
  bootMode2 = preferences.getUInt("bootmode", 0);
  modeRandNorm = preferences.getUInt("audioread", 1);  //random if not define
  backupModeRandom_sleep=modeRandNorm;

  //new prefs
  intMatTheme[1] = preferences.getUInt("th1", 0);
  intMatTheme[2] = preferences.getUInt("th2", 0);
  intMatTheme[6] = preferences.getUInt("th6", 0);
  intMatTheme[7] = preferences.getUInt("th7", 0);
  numeroSerie = preferences.getLong("sn", 0);

  verSw = preferences.getUInt("sw", 0);
  verHw = preferences.getUInt("hw", 0);
  gainHp = preferences.getUInt("gh", 0);
  gainCasq = preferences.getUInt("gc", 0);

  uartdbg = preferences.getUInt("uartdbg", 1);  //uart debug by default

  timeAutoOff = preferences.getUInt("autooff", 60);  //default 60 min
  timeAutoOff = timeAutoOff * 60000;

  // Increase counter by 1
  //bootMode2++;

  // Store the counter to the Preferences
  //preferences.putUInt("bootmode", bootMode2);

  // Close the Preferences
  preferences.end();

  Serial.begin(2000000);  //uart debug:2000000   uart_usb_otg:115200
  delay(20);

  //while (!Serial) {
  // }
  Serial.println("************************************************************");
  if ((bootMode2 % 2) == 0) {
    //0:normal audio 1:usb key
    Serial.print("START PROGRAM MODE AUDIO -mode usb:");
  } else {
    //0:normal audio 1:usb key
    Serial.print("START PROGRAM USB KEY  -mode normal:");
  }
  Serial.println(bootMode2);
  Serial.println("************************************************************");

  print_wakeup_reason();

  //watchdog
  esp_task_wdt_init(WDT_TIMEOUT, true);  //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);                //add current thread to WDT watch

  //test encryption simple
  Serial.println("Test encryption");

  timeDecodeCrypt = millis();
  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_enc(&aes, (const unsigned char *)keycrypt, strlen(keycrypt) * 8);
  mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, (const unsigned char *)inputcrypt, outputcrypt);
  mbedtls_aes_free(&aes);
  timeDecodeCrypt = millis() - timeDecodeCrypt;

  for (i = 0; i < 16; i++) {
    sprintf(strcrypt, "%02x", (int)outputcrypt[i]);
    Serial.print(strcrypt);
  }
  Serial.print("duree-");
  Serial.println(timeDecodeCrypt);
  Serial.println("fin test encryption");

  //random seed for random music
  randomSeed(analogRead(0));
  // random(300);

  if ((bootMode2 % 2) == 0) {
    setup_veilleuse();
  } else {
    setup_usb();
  }
}



/*
.__                        
|  |   ____   ____ ______  
|  |  /  _ \ /  _ \\____ \ 
|  |_(  <_> |  <_> )  |_> >
|____/\____/ \____/|   __/ 
                   |__|    
//Main loop application
--------------------------------------------------------------------------------------------
*/
void loop() {
  if ((bootMode2 % 2) == 0) {
    loop_veilleuse();
  } else {
    loop_usb();
  }
}



/*
 __      __         __            .__        _____             
/  \    /  \_____  |  | __ ____   |__| _____/ ____\____  ______
\   \/\/   /\__  \ |  |/ // __ \  |  |/    \   __\/  _ \/  ___/
 \        /  / __ \|    <\  ___/  |  |   |  \  | (  <_> )___ \ 
  \__/\  /  (____  /__|_ \\___  > |__|___|  /__|  \____/____  >
       \/        \/     \/    \/          \/                \/ 
//Wakeup informations
--------------------------------------------------------------------------------------------
*/
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t source_reveil;

  source_reveil = esp_sleep_get_wakeup_cause();

  switch (source_reveil) {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Réveil causé par un signal externe avec RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1: Serial.println("Réveil causé par un signal externe avec RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Réveil causé par un timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Réveil causé par un touchpad"); break;
    default: Serial.printf("Réveil pas causé par le Deep Sleep: %d\n", source_reveil); break;
  }
}



/*
               __                                              
  ______ _____/  |_ __ ________     ____   ___________  _____  
 /  ___// __ \   __\  |  \____ \   /    \ /  _ \_  __ \/     \ 
 \___ \\  ___/|  | |  |  /  |_> > |   |  (  <_> )  | \/  Y Y  \
/____  >\___  >__| |____/|   __/  |___|  /\____/|__|  |__|_|  /
     \/     \/           |__|          \/                   \/                            
//setup init normal mode audio
--------------------------------------------------------------------------------------------
*/
void setup_veilleuse() {

  //reset watchdog
  esp_task_wdt_reset();

  // //auto maintain power of the board when start
  // //DO activate when R22 100K remove
  // pinMode(PIN_POWER_BOARD_SWITCH_LIGHT, OUTPUT);
  // digitalWrite(PIN_POWER_BOARD_SWITCH_LIGHT, HIGH);  //high:power ON switch9 low:power OFF switch9
  // delay(1000);

  // Serial.begin(2000000);  //uart debug:2000000   uart_usb_otg:115200
  // delay(20);
  // //while (!Serial) {
  // // }
  // Serial.println("************************************************************");
  // Serial.println("START PROGRAM");
  // Serial.println("************************************************************");

  Serial.println("init neopixel");

  //power supply led neopixel
  powerOffLed();
  delay(10);
  powerOnLed();

  //init 4 leds
  FastLED.addLeds<NEOPIXEL, DATA_PIN2>(leds2, NUM_LEDS2);  // GRB ordering is assumed
  FastLED.setBrightness(BRIGHTNESS);

  //start animation
  //  fadeInLed();

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

  //BATTERY STATUS
  pinMode(PIN_CHARGE_STATUS, INPUT);

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

    //test decrypt file
    //66.248s decript medium_file_not_encrypt.enc
    //decryptFile("/medium_file_not_encrypt.enc", "/medium_file_not_encrypt.mp3");
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
      esp_task_wdt_reset();
      if (intWifiConnectRetry >= 50) break;
    }

    if (intWifiConnectRetry >= 20) {
      Serial.println("wifi PROBLEM not connected");
    } else {
      Serial.println("wifi OK");
    }
    //delay(1500);
    esp_task_wdt_reset();

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

    if (WEBSOCKET == 1) {
      wbsoc.onEvent(onWebSocketEvent);
      server.addHandler(&wbsoc);
      server.begin();
    } else {
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
    }
  }  //wifi define

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

  //enable AUDIO
  if ((DEBUG_UART == 1) || (uartdbg == 1)) {
    // delay(2000);
    Serial.println("Enable AUDIO");
  }
  pinMode(PIN_ENABLE_I2S, INPUT);  // floatting left+right/2
  //new board 2.0
  //pinMode(PIN_ENABLE_I2S, OUTPUT);
  //digitalWrite(PIN_ENABLE_I2S, HIGH);

  if ((DEBUG_UART == 1) || (uartdbg == 1)) {
    // delay(2000);
    Serial.println("active read AUDIO");
  }
  if (AUDIO_ACTIVE == 1) {
    Serial.println("set pinout");
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    if ((DEBUG_UART == 1) || (uartdbg == 1)) {
      //delay(2000);
      Serial.println("set balance");
    }
    audio.setBalance(-16);  // mutes the left channel
    if ((DEBUG_UART == 1) || (uartdbg == 1)) {
      //delay(2000);
      Serial.println("set volume 0");
    }
    audio.setVolume(0);
    if ((DEBUG_UART == 1) || (uartdbg == 1)) {
      //delay(2000);
      Serial.println("set mono true");
    }
    audio.forceMono(true);  //mono application

    //stream music
    //audio.connecttohost("http://vis.media-ice.musicradio.com/CapitalMP3");

    //sd musique
    //audio.connecttoFS(SD, "/04/001.mp3");
    if ((DEBUG_UART == 1) || (uartdbg == 1)) {
      //delay(2000);
      Serial.println("audio connect mp3");
    }
    audio.connecttoSD("/04/001.mp3");

    if ((DEBUG_UART == 1) || (uartdbg == 1)) {
      //delay(2000);
      Serial.println("pause");
    }
    audio.pauseResume();
  }

  if ((DEBUG_UART == 1) || (uartdbg == 1)) {
    //delay(2000);
    Serial.println("button play next");
  }
  //int button play next
  pinMode(PIN_BUTTON_PLAY, INPUT_PULLUP);
  pinMode(PIN_BUTTON_NEXT, INPUT_PULLUP);


  if ((DEBUG_UART == 1) || (uartdbg == 1)) {
    //delay(2000);
    Serial.println("jack detect");
  }
  //pullup jack detect
  pinMode(PIN_ADC_JACK_DETECT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ADC_JACK_DETECT), jackChangeInterrupt, CHANGE);

  if ((DEBUG_UART == 1) || (uartdbg == 1)) {
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

  if ((DEBUG_UART == 1) || (uartdbg == 1)) {
    //delay(2000);
    Serial.println("wifi disconnect");
  }

  if (WEBSOCKET == 0) {
    //turn off wifi
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }

  if ((DEBUG_UART == 1) || (uartdbg == 1)) {
    //delay(2000);
    Serial.println("read battery");
  }
  //battery level
  for (i = 0; i < 32; i++) {
    readBatLevel();
  }

  //Led light on startup depending battery level
  //ledBatteryLevel();

  //led with battery level green >4.1V  yellow >3.8V  red <3.5V
  // while (1) {
  //   ledBatteryLevel();

  //   //enable read battery
  //   digitalWrite(PIN_BAT_MEAS_EN, LOW);  //HIGH:read bat off LOW:read bat on
  //   delay(1);
  //   analogBatVoltage = analogRead(PIN_BAT_MEAS);  //100k serie 150K
  //   analogBatVoltage = analogRead(PIN_BAT_MEAS);  //100k serie 150K
  //   analogBatVoltage = analogRead(PIN_BAT_MEAS);  //100k serie 150K
  //   analogBatVoltage = analogRead(PIN_BAT_MEAS);  //100k serie 150K
  //   analogBatVoltage = analogRead(PIN_BAT_MEAS);  //100k serie 150K
  //   //disable read battery
  //   digitalWrite(PIN_BAT_MEAS_EN, HIGH);  //HIGH:read bat off LOW:read bat on
  //   delay(1);
  //   // Serial.print(",");
  //   // Serial.print(999999999999);
  //   // Serial.print(",");
  //   // Serial.println(analogBatVoltage);
  // }

  if ((DEBUG_UART == 1) || (uartdbg == 1)) {
    //delay(2000);
    Serial.println("exit setup -> loop");
    // while (true)
    //   ;
  }

  //init switch 9 pos emotion
  intNumeroDossier = readSwitchEmo(1);  //1 direct read  0:wait interrupt for reading

  // // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
  // usb_msc.setID("Mandalou", "SDCard", "1.0");

  // // Set read write callback
  // usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);

  // // Still initialize MSC but tell usb stack that MSC is not ready to read/write
  // // If we don't initialize, board will be enumerated as CDC only
  // usb_msc.setUnitReady(false);

  //   usb_msc.begin();

  //   if (!sd.begin(SD_CS2, SD_SCK_MHZ(50))) {
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

  //   fs_changed = false;  // to print contents initially

  //init button play next
  readButPlay = digitalRead(PIN_BUTTON_PLAY);
  readButNext = digitalRead(PIN_BUTTON_NEXT);
  buttonStateNextLongPress = readButNext;

  lastDebounceTimePlayNext = millis() + longPressPlayNext;

  timeoutAutoOff = millis() + timeAutoOff;

  //file classifiation if add in usb mode at root
  fileClassifiation();

}  //setup_veilleuse



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
void loop_veilleuse() {
  nowTimeMillis = millis();
  ulong_time_now = millis();

  //auto off if no action during long time
  if (millis() > timeoutAutoOff) {
    //power auto off
    Serial.println("POWER AUTO OFF");
    //led off
    fadeOutLed();

    Serial.println("OFF");
    pinMode(PIN_POWER_BOARD_SWITCH_LIGHT, INPUT);
    //    digitalWrite(PIN_POWER_BOARD_SWITCH_LIGHT, LOW);  //high:power ON switch9 input:power OFF switch9
    delay(20);
  }


  //check jack status digital pin not analog
  if (millis() > CheckTimeJackInserted) {
    jackInserted = 1;
    //jackInsertedCnt = 0;
    attachInterrupt(digitalPinToInterrupt(PIN_ADC_JACK_DETECT), jackChangeInterrupt, CHANGE);
  }

  //change gain depending jack inserted
  changeGainJack();

  //read ADC value switch rotary 5 positions ADC
  if (millis() > updateAdcRead) {
    updateAdcRead = millis() + PERIOD_READ_ADC;

    // switch theme
    analogSwitchTheme = analogRead(PIN_SWITCH_THEME);
    analogSwitchTheme = analogRead(PIN_SWITCH_THEME);
    analogSwitchTheme = analogRead(PIN_SWITCH_THEME);
    //Serial.printf("Switch theme = %d\n", analogSwitchTheme);
    intthemeChoice = themeSelect(analogSwitchTheme);

    if (intOldthemeChoice != intthemeChoice) {
      //update auto-off if action on any button
      timeoutAutoOff = millis() + timeAutoOff;

      intOldthemeChoice = intthemeChoice;
      //change_song();
      changeDirEmotion(intNumeroDossier);
    }

    // switch user
    analogSwitchuser = analogRead(PIN_SWITCH_USER);
    analogSwitchuser = analogRead(PIN_SWITCH_USER);
    analogSwitchuser = analogRead(PIN_SWITCH_USER);
    //Serial.printf("Switch user = %d\n", analogSwitchuser);

    //Battery voltage adc@3.3V level 4.2V, 3.9V and 3.5V low limit
    readBatLevel();

    //Charge status BMS
    chargeStatus = digitalRead(PIN_CHARGE_STATUS);

    if ((oldChargeStatus != chargeStatus) || (firstChargeSatatus == 1)) {
      oldChargeStatus = chargeStatus;
      firstChargeSatatus = 0;

      battLightLevelOld = -10;

      // if (chargeStatus == 0) {
      //   //blink red in charge
      //   animateLedInCharge();
      // } else {
      //   //blink green charge finish
      //   animateLedChargeComplete();
      // }
    }

    //led animation if batt level change
    if (battLightLevelOld != battLightLevel) {
      battLightLevelOld = battLightLevel;
      animateLedChargeComplete();
    }

    if (WEBSOCKET == 1) {
      // Créer un objet JSON avec la bibliothèque ArduinoJson
      //const size_t capacity = JSON_OBJECT_SIZE(5);
      jsonDoc["bat"] = analogBatVoltage;
      jsonDoc["thm"] = intthemeChoice;
      jsonDoc["emo"] = intNumeroDossier;
      jsonDoc["vol"] = valVolume;
      jsonDoc["chg"] = chargeStatus;

      // Convertir l'objet JSON en chaîne
      serializeJson(jsonDoc, jsonStringws);

      // Envoyer la trame JSON aux clients WebSocket connectés
      Serial.println(jsonStringws);
      esp_task_wdt_reset();
      //wbsoc.textAll(jsonStringws);
    }
  }  //update adc updateAdcRead each 1sec

  //log data
  logUart(1);

  //uart config
  uartConfig();

  //update dispay time
  if (WIFI_ACTIVE == 1) {
    if (millis() > updateTimeHorloge) {
      printLocalTime();
    }
  }

  //read switch 9 pos emotion
  intNumeroDossier = readSwitchEmo(0);  //1 direct read  0:wait interrupt for reading
  if (intNumeroDossier != intOldNumeroDossier) {
    //update auto-off if action on any button
    timeoutAutoOff = millis() + timeAutoOff;

    intOldNumeroDossier = intNumeroDossier;
    changeDirEmotion(intNumeroDossier);
  }

  //read button light on pwer off if long press
  buttonlightLevel = readLightButton();

  if (buttonlightLevel != oldButtonlightLevel) {

    if (oldButtonlightLevel == 0 && buttonlightLevel == 1) {

      //update auto-off if action on any button
      timeoutAutoOff = millis() + timeAutoOff;
      flipLight++;
      //animateLedChargeComplete();  //Bett info after click light
      ledlight();

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
      for (ii = 2; ii >= 0; ii--) {
        Serial.println(ii);
        delay(1000);
        esp_task_wdt_reset();
      }

      Serial.println("OFF");
      pinMode(PIN_POWER_BOARD_SWITCH_LIGHT, INPUT);
      //digitalWrite(PIN_POWER_BOARD_SWITCH_LIGHT, LOW);  //high:power ON switch9 input:power OFF switch9
      delay(20);
    }
  }


  //PART button play change power audio
  readButPlay = digitalRead(PIN_BUTTON_PLAY);
  readButNext = digitalRead(PIN_BUTTON_NEXT);

  //test press PLAY NEXT same time => BOOT MODE
  if (readButPlay == 0 && readButNext == 0) {
    if ((millis()) > lastDebounceTimePlayNext) {
      //reboot and change boot type
      //++bootMode;  //flip
      //Serial.println("Reboot mode :" + String(bootMode));

      ++bootMode2;
      // Ajoutez des données au document JSON
      jsonConfig["mode"] = "usb";  //normal ou usb
      jsonConfig["Nb"] = bootMode2;

      // convert json and print
      serializeJson(jsonConfig, jsonString);
      //Serial.println(jsonString);   //not secure see password uart
      Serial.println("BOOT USB MODE");

      //save on preference flash
      //preference on flash data etain after reboot
      preferences.begin("my-app", false);
      // Remove all preferences under the opened namespace
      //preferences.clear();
      // Or remove the counter key only
      //preferences.remove("counter");
      // Get the counter value, if the key does not exist, return a default value of 0
      // Note: Key name is limited to 15 chars.
      //bootMode2 = preferences.getUInt("bootmode", 0);
      // Increase counter by 1
      //bootMode2++;

      // Store the counter to the Preferences
      preferences.putUInt("bootmode", bootMode2);

      // Close the Preferences
      preferences.end();

      powerOnLed();
      while (true) {
        //for (int ki = 0; ki < 12; ki++) {
        leds2[0] = CRGB(0, 0, 0);
        leds2[1] = CRGB(0, 0, 0);
        leds2[2] = CRGB(0, 0, 0);
        leds2[3] = CRGB(0, 0, 0);
        ii++;
        if (ii >= 4) ii = 0;
        leds2[ii] = CRGB(0, 90, 0);
        FastLED.show();
        delay(100);
      }  //for watchdog

      leds2[0] = CRGB(0, 0, 0);
      leds2[1] = CRGB(0, 0, 0);
      leds2[2] = CRGB(0, 0, 0);
      leds2[3] = CRGB(0, 0, 0);
      FastLED.show();
      delay(100);

      //boot usb
      //setup();

      //esp_task_wdt_reset();
    }
  } else {
    lastDebounceTimePlayNext = millis() + longPressPlayNext;
    esp_task_wdt_reset();
  }

  if (readButPlay != lastButtonPlay) {
    //update auto-off if action on any button
    timeoutAutoOff = millis() + timeAutoOff;

    // reset the debouncing timer
    lastDebounceTimePlay = millis();
  }

  if (readButNext != lastButtonNext) {
    //update auto-off if action on any button
    timeoutAutoOff = millis() + timeAutoOff;

    // reset the debouncing timer
    lastDebounceTimeNext = millis();
  }

  if (readButNext != lastButtonNextRandom) {
    //update auto-off if action on any button
    timeoutAutoOff = millis() + timeAutoOff;

    // reset the debouncing timer and detect long press RANDOM
    lastLongPressTimeNext = millis();
  }

  //mode random/normal
  if ((millis() - lastLongPressTimeNext) > MEDIUM_PRESS) {
    //test pause/play

    // if the button state has changed:
    if (readButNext != buttonStateNextLongPress) {
      buttonStateNextLongPress = readButNext;

      // only toggle the LED if the new button state is HIGH
      if (buttonStateNextLongPress == HIGH) {

        //flip normal/random
        modeRandNorm = modeRandNorm ^ 1;
        backupModeRandom_sleep=modeRandNorm;
        if (modeRandNorm == 0) {
          Serial.println("Normal mode read song");
          //preference on flash data etain after reboot
          preferences.begin("my-app", false);
          // Store the counter to the Preferences
          preferences.putUInt("audioread", modeRandNorm);
          // Close the Preferences
          preferences.end();
          //Animate rotation
          animateledRot();
        } else {
          Serial.println("Random mode read song");
          modeRandNorm = 1;
          backupModeRandom_sleep=modeRandNorm;
          //preference on flash data etain after reboot
          preferences.begin("my-app", false);
          // Store the counter to the Preferences
          preferences.putUInt("audioread", modeRandNorm);
          // Close the Preferences
          preferences.end();
          //Animate flip random
          animateledFlip();
        }
      }
    } else {
    }
  }

  if ((millis() - lastDebounceTimeNext) > debounceDelay) {
    //test pause/play
    // if the button state has changed:
    if (readButNext != buttonStateNext) {
      buttonStateNext = readButNext;

      // only toggle the LED if the new button state is HIGH
      if (buttonStateNext == HIGH) {
        nextSong++;
        if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
          Serial.println("Next Song click");
        }
        if (TEST_LED == 0) {
          if (AUDIO_ACTIVE == 1) {
            change_song_name();
            //change_song();
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
        if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
          Serial.println("Long press PLAY/PAUSE");  //15dB
        }
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
  lastButtonNextRandom = readButNext;
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
      //update auto-off if action on any button
      timeoutAutoOff = millis() + timeAutoOff;

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

  if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
    Serial.printf("Listing directory: %s\n", dirname);
  }

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
        char addslash[200];
        strcpy(addslash, "/");
        strcat(addslash, file.name());
        //Serial.println(addslash);
        listDir(fs, addslash, levels - 1);
        //listDir(fs,file.name(), levels - 1);
      }
    } else {

      intNbAudioFileInDir++;
      char addfullpath[200];

      strcpy(addfullpath, dirname);
      strcat(addfullpath, "/");
      strcat(addfullpath, file.name());
      //Serial.print(file.name());
      if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
        Serial.print("  FILE: ");
        Serial.print(addfullpath);
        Serial.print("  SIZE: ");
        Serial.println(file.size());
      }
    }
    file = root.openNextFile();
  }

  if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
    Serial.print("File number:");
    Serial.println(intNbAudioFileInDir);
  }
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



//auto loop directory
void audio_eof_mp3(const char *info) {  //end of file

  nextSong++;
  if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
    Serial.print("audio_info eof: ");
    Serial.println(info);
    Serial.println("Next Song autoloop");
  }
  change_song_name();
  //change_song();
}

//audio info I2S
//------------------EVENTS----------------------------------------------------------------------------------------------
void audio_info(const char *info) {
  //Serial.printf("info: %s\n", info);
}


void change_song(void) {
  char local_name_directory[200] = "/09";

  //gestion of normal or random
  if (modeRandNorm == 1) {
    nextSong = random((intNbAudioFileInDir - 1));
  }

  //if (nextSong >= (intNbAudioFileInDir - 1)) nextSong = 0;  //intNbAudioFileInDir
  if (nextSong >= intNbAudioFileInDir) nextSong = 0;  //intNbAudioFileInDir

  //theme test
  sprintf(local_name_directory, "/%02d/%02d/%03d.mp3", intNumeroDossier, intthemeChoice, (nextSong + 1));
  audio.connecttoSD(local_name_directory);
  Serial.println(local_name_directory);
}



int themeSelect(int adcValue) {

  static int choiceDetect = 1;

  for (int kk = 0; kk < 8; kk++) {
    if (((intMatTheme[kk] - intVarAdc) < adcValue) && adcValue < (intMatTheme[kk] + intVarAdc)) {
      //Meditation : pos6-dir01
      //histoire audio : pos7-dir04
      //Musique : pos0-dir03
      //Bruit blancs : pos1-dir05
      //yoga : pos2-dir01
      switch (kk) {
        case 6:
          choiceDetect = 1;
          break;
        case 7:
          choiceDetect = 4;
          break;
        case 0:
          choiceDetect = 3;
          break;
        case 1:
          choiceDetect = 5;
          break;
        case 2:
          choiceDetect = 2;
          break;
        default:
          // no change
          break;
      }
    }
  }  //for 8 choices (switch theme 8 positions but used only 5 positions 2-1-0-7-6)
  return choiceDetect;
}



void logUart(int modeuart) {

  //ADC theme|theme1-5|ADC user|led_status|emotion|nb_files|volume0-21|jack cnt|jack insert|bat voltage|charge status|bootMode|modeRandNorm|PSRAM
  if ((millis() > updateLogUart) && ((DEBUG_UART == 1) || (uartdbg == 1))) {
    //init prio dens uart
    updateLogUart = millis() + PERIOD_LOG_UART;

    if (modeuart == 1) {

      Serial.printf("%d,", analogSwitchTheme);

      //Méditation Yoga Musique Histoire Bruit Blanc
      Serial.printf("%d,", intthemeChoice);  //THEMES Sub directory

      //Serial.printf("%d,", analogSwitchuser);

      Serial.printf("%d,", flipLight);  //led status

      Serial.printf("%d,", intNumeroDossier);  //EMOTION Main directory

      Serial.printf("%d,", intNbAudioFileInDir);  //nb file

      Serial.printf("%d,", valVolume);

      Serial.printf("%d,%d,", jackInsertedCnt, jackInserted);

      Serial.printf("%d,", analogBatVoltage);
      Serial.printf("%d,", chargeStatus);

      //Serial.printf("%d,", bootMode);

      Serial.printf("%d,", bootMode2);

      Serial.printf("%d,", modeRandNorm);

      Serial.printf("%d,", verSw);
      Serial.printf("%d,", verHw);
      Serial.printf("%d,", gainHp);
      Serial.printf("%d,", gainCasq);


      if (WEBSOCKET == 1) {
        //print ip
        Serial.print(WiFi.localIP());
        Serial.print(",");
      }

      // //decrypt
      // Serial.print(",");
      // for (i = 0; i < 16; i++) {
      //   sprintf(strcrypt, "%02x", (int)outputcrypt[i]);
      //   Serial.print(strcrypt);
      // }
      //Serial.printf("%d,", timeDecodeCrypt);

      Serial.printf("%d,", numeroSerie);

      Serial.print(ESP.getFreePsram());
      Serial.printf("\n");
    }  //if modeuart=1 NORMAL
    else {
      //modeuart!=1 USB
      Serial.printf("%d,", bootMode2);

      Serial.printf("%d,", lightLevel);

      Serial.printf("%d,", numeroSerie);

      Serial.print(ESP.getFreePsram());
      Serial.printf("\n");
    }
  }
}



void readBatLevel(void) {
  //enable read battery
  digitalWrite(PIN_BAT_MEAS_EN, LOW);  //HIGH:read bat off LOW:read bat on
  delay(1);

  //Battery voltage adc@3.1V
  analogBatVoltage = analogRead(PIN_BAT_MEAS);  //100k serie 150K
  analogBatVoltage = analogRead(PIN_BAT_MEAS);  //100k serie 150K
  analogBatVoltage = analogRead(PIN_BAT_MEAS);  //100k serie 150K
  analogBatVoltage = analogRead(PIN_BAT_MEAS);  //100k serie 150K
  analogBatVoltage = analogRead(PIN_BAT_MEAS);  //100k serie 150K

  //disable read bat
  //digitalWrite(PIN_BAT_MEAS_EN, LOW);  //HIGH:read bat off LOW:read bat on

  //for (int tt = 0; tt < 16; tt++) {
  indiceBatVoltage++;

  adcBatStable++;
  if (adcBatStable >= 32) adcBatStable = 32;

  // analogBatVoltage = analogRead(PIN_BAT_MEAS);  //100k serie 150K
  // delay(1);
  sumBatVoltage = (sumBatVoltage - matAnalogBatVoltage[indiceBatVoltage % 16]);
  sumBatVoltage = analogBatVoltage + sumBatVoltage;

  matAnalogBatVoltage[indiceBatVoltage % 16] = analogBatVoltage;
  // }

  // for (int tt = 0; tt < 16; tt++) {
  //   Serial.print(",");
  //   Serial.print(matAnalogBatVoltage[tt]);
  // }

  // Serial.print(",");
  // Serial.print(sumBatVoltage);

  //analogBatVoltage = analogBatVoltage * 100 / 695;  //ratio Vout/Vin=0.6
  analogBatVoltage = (sumBatVoltage >> 4) * 825 / 6144;  //Vbat=Vadc*1,6*100

  // Serial.print(",");
  //Serial.println(analogBatVoltage);

  digitalWrite(PIN_BAT_MEAS_EN, HIGH);  //HIGH:read bat off LOW:read bat on

  //light level
  if (adcBatStable == 32) {

    // if (analogBatVoltage < battUltraLowLevel) {
    //   battLightLevel = 0;
    // } else if ((analogBatVoltage >= battUltraLowLevel) && (analogBatVoltage < battLowLevel)) {
    //   battLightLevel = 1;
    // } else if ((analogBatVoltage >= battLowLevel) && (analogBatVoltage < battMedLevel)) {
    //   battLightLevel = 2;
    // } else if ((analogBatVoltage >= battMedLevel) && (analogBatVoltage < battHighLevel)) {
    //   battLightLevel = 3;
    // } else {
    //   battLightLevel = 4;
    // }

    // Mise à jour du niveau actuel en fonction de la lecture de l'ADC avec hystérésis
    if (analogBatVoltage < (battUltraLowLevel - batHystLevel)) {
      battLightLevel = 0;
    } else if ((analogBatVoltage >= battUltraLowLevel) && (analogBatVoltage < (battLowLevel - batHystLevel))) {
      battLightLevel = 1;
    } else if ((analogBatVoltage >= battLowLevel) && (analogBatVoltage < (battMedLevel - batHystLevel))) {
      battLightLevel = 2;
    } else if ((analogBatVoltage >= battMedLevel) && (analogBatVoltage < (battHighLevel - batHystLevel))) {
      battLightLevel = 3;
    } else if (analogBatVoltage >= battHighLevel) {
      battLightLevel = 4;
    }

    // if (analogBatVoltage <= battUltraLowLevel) {
    //   //ultra low levl force power off for preserving battery
    //   battLightLevel = 0;  //ultra low
    // }

    // if (((battLowLevel - batHystLevel) <= analogBatVoltage) && (analogBatVoltage <= (battLowLevel + batHystLevel))) {
    //   battLightLevel = 1;  //low
    // }

    // if (((battMedLevel - batHystLevel) <= analogBatVoltage) && (analogBatVoltage <= (battMedLevel + batHystLevel))) {
    //   battLightLevel = 2;  //Medium
    // }

    // if (analogBatVoltage >= battHighLevel - batHystLevel) {
    //   battLightLevel = 3;  //High
    // }
  }  //stable adc batt measure
}



void ledBatteryLevel(void) {
  int colorR = 0;
  int colorG = 0;
  int colorB = 0;

  readBatLevel();

  if (battLightLevel == 1) {  //low batt
    colorR = 1;
    colorG = 0;
    colorB = 0;
  } else {
    if (battLightLevel == 2) {
      battLightLevel = 2;  //Medium batt
      colorR = 0;
      colorG = 1;
      colorB = 0;
    } else {
      battLightLevel = 3;  //Full batt
      colorR = 0;
      colorG = 1;
      colorB = 0;
    }
  }

  powerOnLed();
  for (ii = 10; ii < 150; ii = ii + 10) {
    for (i = 0; i < NUM_LEDS2; i++) {
      leds2[i] = CRGB(colorR * ii, colorG * ii, colorB * ii);
    }
    FastLED.show();
    delay(100);
    esp_task_wdt_reset();
  }
  delay(1000);
  for (i = 0; i < NUM_LEDS2; i++) {
    leds2[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
  delay(10);
  powerOffLed();
}



void fadeInLed(void) {
  powerOnLed();
  for (ii = 10; ii < 250; ii = ii + 10) {
    for (i = 0; i < NUM_LEDS2; i++) {
      leds2[i] = CRGB(0, 0, ii);
    }
    FastLED.setBrightness(200);
    FastLED.show();
    delay(100);
    esp_task_wdt_reset();
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
  FastLED.setBrightness(200);
  FastLED.show();
  delay(1000);
  for (ii = 250; ii >= 0; ii = ii - 10) {

    for (i = 0; i < NUM_LEDS2; i++) {
      leds2[i] = CRGB(0, 0, ii);
    }
    FastLED.show();
    delay(100);
    esp_task_wdt_reset();
  }
  powerOffLed();
}



void ledlight(void) {
  if (flipLight >= 4) flipLight = 0;
  switch (flipLight) {
    case 1:
      // light warm low
      activeLed(255, 200, 0, 10, 1);  // red,  green,  blue,  brighness, active
      break;
    case 2:
      // light warm medium
      activeLed(255, 200, 0, 80, 1);  // red,  green,  blue,  brighness, active
      break;
    case 3:
      // light warm high
      activeLed(255, 200, 0, 200, 1);  // red,  green,  blue,  brighness, active
      break;
    default:
      // light off
      activeLed(0, 0, 0, 200, 0);  // red,  green,  blue,  active
      break;
  }
}



void animateledRot(void) {
  unsigned long localTimeMillis = 0;
  int ki = 0;

  powerOnLed();
  while (true) {
    esp_task_wdt_reset();
    if (millis() > localTimeMillis) {
      localTimeMillis = millis() + 100;  //100ms
      ki++;
      if (ki >= 12) break;

      leds2[0] = CRGB(0, 0, 0);
      leds2[1] = CRGB(0, 0, 0);
      leds2[2] = CRGB(0, 0, 0);
      leds2[3] = CRGB(0, 0, 0);
      ii++;
      if (ii >= 4) ii = 0;
      leds2[ii] = CRGB(0, 250, 0);
      FastLED.show();
      delay(10);
    }
    if (AUDIO_ACTIVE == 1) {
      audio.loop();
    }
  }
  powerOffLed();
  ledlight();
}



void animateledFlip(void) {
  ii = 0;
  unsigned long localTimeMillis = 0;
  int ki = 0;

  powerOnLed();
  while (true) {
    esp_task_wdt_reset();
    if (millis() > localTimeMillis) {
      localTimeMillis = millis() + 100;  //100ms
      ki++;
      if (ki >= 12) break;
      if (ii == 0) {
        ii = 1;
        leds2[0] = CRGB(0, 250, 0);
        leds2[1] = CRGB(0, 0, 0);
        leds2[2] = CRGB(0, 250, 0);
        leds2[3] = CRGB(0, 0, 0);
      } else {
        ii = 0;
        leds2[0] = CRGB(0, 0, 0);
        leds2[1] = CRGB(0, 250, 0);
        leds2[2] = CRGB(0, 0, 0);
        leds2[3] = CRGB(0, 250, 0);
      }
      FastLED.show();
      delay(10);
    }

    if (AUDIO_ACTIVE == 1) {
      audio.loop();
    }
  }
  powerOffLed();
  ledlight();
}



void animateLedInCharge(void) {
  ii = 0;
  unsigned long localTimeMillis = 0;
  int ki = 0;

  powerOnLed();
  while (true) {
    esp_task_wdt_reset();
    if (millis() > localTimeMillis) {
      localTimeMillis = millis() + 100;  //100ms
      ki++;
      if (ki >= 12) break;
      if (ii == 0) {
        ii = 1;
        leds2[0] = CRGB(190, 0, 0);
        leds2[1] = CRGB(190, 0, 0);
        leds2[2] = CRGB(190, 0, 0);
        leds2[3] = CRGB(190, 0, 0);
      } else {
        ii = 0;
        leds2[0] = CRGB(0, 0, 0);
        leds2[1] = CRGB(0, 0, 0);
        leds2[2] = CRGB(0, 0, 0);
        leds2[3] = CRGB(0, 0, 0);
      }
      FastLED.show();
      delay(10);
    }

    if (AUDIO_ACTIVE == 1) {
      audio.loop();
    }
  }
  powerOffLed();
  ledlight();
}


void fadeEffect(int percent_red, int percent_green, int percent_blue, int timeWait, int cycle) {
  int nbCycle = 0;
  // Fade in/out loop
  int brightness = 0;
  int step = 10;
  unsigned long localTimeMillis = 0;

  powerOnLed();
  while (true) {
    esp_task_wdt_reset();
    //set step
    if (millis() > localTimeMillis) {
      localTimeMillis = millis() + timeWait;  //100ms


      if ((brightness >= BRIGHTNESS) && (step == 10)) {
        step = -10;
      }

      if ((brightness <= 0) && (step == -10)) {
        step = +10;
        nbCycle++;
        if ((nbCycle >= cycle)) break;
      }

      brightness = brightness + step;

      // Update the brightness of each LED
      for (int i = 0; i < NUM_LEDS2; i++) {
        int red = map(percent_red, 0, BRIGHTNESS, 0, brightness);
        int green = map(percent_green, 0, BRIGHTNESS, 0, brightness);
        int blue = map(percent_blue, 0, BRIGHTNESS, 0, brightness);
        leds2[i] = CRGB(red, green, blue);
      }

      // Serial.print("brightness: ");
      // Serial.println(brightness);

      FastLED.setBrightness(brightness);
      FastLED.show();
    }

    if (AUDIO_ACTIVE == 1) {
      audio.loop();
    }

  }  //while
  powerOffLed();
  ledlight();
}


void animateLedChargeComplete(void) {
  ii = 0;
  unsigned long localTimeMillis = 0;
  int ki = 0;
  int colorRed = 0;
  int colorGreen = 0;
  int colorBlue = 0;


  switch (battLightLevel) {
    case 1:
      //low red
      fadeEffect(100, 0, 0, 50, 1);
      break;
    case 2:
      //medium orange  255 127
      fadeEffect(100, 50, 0, 50, 1);
      break;
    case 3:
      //high green
      fadeEffect(0, 100, 0, 50, 1);
      break;
    case 4:
      //high green
      fadeEffect(0, 100, 0, 50, 2);
      break;
    default:
      // ultra low battery power off
      fadeEffect(100, 0, 0, 10, 4);
      //POWER OFF BATTERY ULTRA LOW
      pinMode(PIN_POWER_BOARD_SWITCH_LIGHT, INPUT);
      delay(100);
      break;
  }

  // powerOnLed();
  // ki = 0;
  // while (true) {
  //   esp_task_wdt_reset();
  //   if (millis() > localTimeMillis) {
  //     localTimeMillis = millis() + 100;  //100ms
  //     ki++;
  //     if (ki >= 12) break;
  //     if (ii == 0) {
  //       ii = 1;
  //       leds2[0] = CRGB(colorRed, colorGreen, colorBlue);
  //       leds2[1] = CRGB(colorRed, colorGreen, colorBlue);
  //       leds2[2] = CRGB(colorRed, colorGreen, colorBlue);
  //       leds2[3] = CRGB(colorRed, colorGreen, colorBlue);
  //     } else {
  //       ii = 0;
  //       leds2[0] = CRGB(0, 0, 0);
  //       leds2[1] = CRGB(0, 0, 0);
  //       leds2[2] = CRGB(0, 0, 0);
  //       leds2[3] = CRGB(0, 0, 0);
  //     }
  //     FastLED.show();
  //     delay(10);
  //   }

  //   if (AUDIO_ACTIVE == 1) {
  //     audio.loop();
  //   }
  // }
  // powerOffLed();
  // ledlight();
}



void activeLed(int red, int green, int blue, int brighness, int active) {
  for (i = 0; i < NUM_LEDS2; i++) {
    if (active == 1) {
      powerOnLed();
      leds2[i] = CRGB(red, green, blue);
    } else {
      powerOffLed();
    }
  }
  delay(10);
  FastLED.setBrightness(brighness);
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
  digitalWrite(PIN_POWER_BOARD_SWITCH_LIGHT, HIGH);  //high:power ON switch9 INPUT:power OFF switch9

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


    if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
      Serial.print("int SW9:");
      Serial.println(intCmptRotSw9);
    }

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
    if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
      Serial.print("Selection : ");
      Serial.println(selection_emo);
    }
    return selection_emo;


  } else {
    //read if int occur
    if (intDetectExpIoSw9 == 1) {
      intDetectExpIoSw9 = 0;
      delay(2);
      attachInterrupt(digitalPinToInterrupt(PIN_INT_SW9), intExpIoSw9, FALLING);

      if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
        Serial.print("int SW9:");
        Serial.println(intCmptRotSw9);
      }

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
      if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
        Serial.print("Selection : ");
        Serial.println(selection_emo);
      }

      return selection_emo;
    }
    return intNumeroDossier;
  }
}



void changeDirEmotion(int intDirEmotion) {

  char local_name_directory[200] = "";

  if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
    Serial.print("Dossier EMOTION:");
    Serial.println(intDirEmotion);
  }


  //get emotion intDirEmotion, intthemeChoice
  switch (intNumeroDossier) {
    case 1:
      // Joie
      sprintf(local_name_directory, "/Joie");
      break;
    case 2:
      // Jalousie
      sprintf(local_name_directory, "/Jalousie");
      break;
    case 3:
      // Mon moment à moi
      sprintf(local_name_directory, "/Mon moment à moi");
      break;
    case 4:
      // Contenu libre
      sprintf(local_name_directory, "/Contenu libre");
      break;
    case 5:
      // Je m'endors paisiblement
      sprintf(local_name_directory, "/Je m'endors paisiblement");
      break;
    case 6:
      // Je me réveille de bonne humeur
      sprintf(local_name_directory, "/Je me réveille de bonne humeur");
      break;
    case 7:
      // Tristesse
      sprintf(local_name_directory, "/Tristesse");
      break;
    case 8:
      // Colère
      sprintf(local_name_directory, "/Colère");
      break;
    case 9:
      // Peur
      sprintf(local_name_directory, "/Peur");
      break;
    default:
      // Contenu libre
      sprintf(local_name_directory, "/Contenu libre");
      break;
  }

  //get thema
  switch (intthemeChoice) {
    case 1:
      // Méditations
      sprintf(local_name_directory, "%s/Méditations", local_name_directory);
      break;
    case 2:
      // Activités
      sprintf(local_name_directory, "%s/Activités", local_name_directory);
      break;
    case 3:
      // Musique
      sprintf(local_name_directory, "%s/Musique", local_name_directory);
      break;
    case 4:
      // Histoires audio
      sprintf(local_name_directory, "%s/Histoires audio", local_name_directory);
      break;
    case 5:
      // Bruits blancs et sons de la nature
      sprintf(local_name_directory, "%s/Bruits blancs et sons de la nature", local_name_directory);
      break;
    default:
      //Musique
      sprintf(local_name_directory, "%s/Musique", local_name_directory);
      break;
  }


  //sprintf(name_directory, "/%02d/%02d", intDirEmotion, intthemeChoice);

  intNbAudioFileInDir = 0;
  //  listDir(SD, name_directory, 1);
  listDir(SD, local_name_directory, 1);

  nextSong = 0;

  //lecture automatique chantgement emotion
  change_song_name();

  //sprintf(local_name_directory, "/%02d/%02d/000.mp3", intNumeroDossier, intthemeChoice);
  //audio.connecttoSD(local_name_directory);
}



void changeGainJack(void) {

  if (jackInserted != oldJackInserted) {
    oldJackInserted = jackInserted;

    //reload preferences
    preferences.begin("my-app", true);  //true readonly

    gainHp = preferences.getUInt("gh", 0);
    gainCasq = preferences.getUInt("gc", 0);

    // Close the Preferences
    preferences.end();

    if (jackInserted == 1) {

      if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
        Serial.println("Jack inserted");
      }
      //3dB casque
      // pinMode(I2S_GAIN, OUTPUT);
      // digitalWrite(I2S_GAIN, HIGH);
      switch (gainCasq) {
        case 1:
          if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
            Serial.println("gain:3dB");  //GAIN_SLOT=Pull-up 3.3V -> 3 dB
          }
          pinMode(I2S_GAIN, INPUT_PULLUP);
          break;
        case 2:
          if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
            Serial.println("gain:6dB");  //GAIN_SLOT=VDD=3.3V -> 6dB
          }
          pinMode(I2S_GAIN, OUTPUT);
          digitalWrite(I2S_GAIN, HIGH);
          break;
        case 3:
          if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
            Serial.println("gain:9dB");  //GAIN_SLOT=floating input -> 9dB
          }
          pinMode(I2S_GAIN, INPUT);
          break;
        case 4:
          if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
            Serial.println("gain:12dB");  //GAIN_SLOT=GND -> 12dB
          }
          pinMode(I2S_GAIN, OUTPUT);
          digitalWrite(I2S_GAIN, LOW);
          break;
        case 5:
          if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
            Serial.println("gain:15dB");  //GAIN_SLOT=Pull-down GND -> 15 dB
          }
          pinMode(I2S_GAIN, INPUT_PULLDOWN);
          break;
        default:
          //none
          if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
            Serial.println("gain: no change");
          }
          break;
      }

    } else {

      if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
        Serial.println("Jack ejected -> HP");
      }
      //12db haut parleur
      // pinMode(I2S_GAIN, INPUT);
      // //3dB casque
      // pinMode(I2S_GAIN, OUTPUT);
      // digitalWrite(I2S_GAIN, HIGH);
      switch (gainHp) {
        case 1:
          if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
            Serial.println("gain:3dB");  //GAIN_SLOT=Pull-up 3.3V -> 3 dB
          }
          pinMode(I2S_GAIN, INPUT_PULLUP);
          break;
        case 2:
          if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
            Serial.println("gain:6dB");  //GAIN_SLOT=VDD=3.3V -> 6dB
          }
          pinMode(I2S_GAIN, OUTPUT);
          digitalWrite(I2S_GAIN, HIGH);
          break;
        case 3:
          if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
            Serial.println("gain:9dB");  //GAIN_SLOT=floating input -> 9dB
          }
          pinMode(I2S_GAIN, INPUT);
          break;
        case 4:
          if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
            Serial.println("gain:12dB");  //GAIN_SLOT=GND -> 12dB
          }
          pinMode(I2S_GAIN, OUTPUT);
          digitalWrite(I2S_GAIN, LOW);
          break;
        case 5:
          if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
            Serial.println("gain:15dB");  //GAIN_SLOT=Pull-down GND -> 15 dB
          }
          pinMode(I2S_GAIN, INPUT_PULLDOWN);
          break;
        default:
          //none
          if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
            Serial.println("gain: no change");
          }
          break;
      }
    }
  }
}



/*
 ____ ___  ___________________            __  .__        __    
|    |   \/   _____/\______   \   _______/  |_|__| ____ |  | __
|    |   /\_____  \  |    |  _/  /  ___/\   __\  |/ ___\|  |/ /
|    |  / /        \ |    |   \  \___ \  |  | |  \  \___|    < 
|______/ /_______  / |______  / /____  > |__| |__|\___  >__|_ \
                 \/         \/       \/               \/     \/
//USB key function
               __                                ___.    
  ______ _____/  |_ __ ________      __ __  _____\_ |__  
 /  ___// __ \   __\  |  \____ \    |  |  \/  ___/| __ \ 
 \___ \\  ___/|  | |  |  /  |_> >   |  |  /\___ \ | \_\ \
/____  >\___  >__| |____/|   __/____|____//____  >|___  /
     \/     \/           |__| /_____/          \/     \/ 
--------------------------------------------------------------------------------------------
*/
// the setup function runs once when you press reset or power the board
void setup_usb() {

  //reset watchdog
  esp_task_wdt_reset();

  //POWER ON LED
  powerOnLed();

  pinMode(PIN_POWER_BOARD_SWITCH_LIGHT, OUTPUT);
  digitalWrite(PIN_POWER_BOARD_SWITCH_LIGHT, HIGH);  //high:power ON switch9 low:power OFF switch9
  delay(50);

  //int button play next
  pinMode(PIN_BUTTON_PLAY, INPUT_PULLUP);
  pinMode(PIN_BUTTON_NEXT, INPUT_PULLUP);

  Serial.begin(2000000);
  //while (!Serial) delay(10);  // wait for native usb

  //init 4 leds
  FastLED.addLeds<NEOPIXEL, DATA_PIN2>(leds2, NUM_LEDS2);  // GRB ordering is assumed
  FastLED.setBrightness(BRIGHTNESS);

  for (ii = 0; ii < NUM_LEDS2; ii++) {
    leds2[ii] = CRGB(0, 0, 0);
  }
  FastLED.show();

  delay(100);
  Serial.println("Test Mass Storage SD Card Mandalou");

  delay(200);

  //pinMode(LED_BUILTIN, OUTPUT);
  //SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CHIP_SELECT_PIN); //ajouter pour check pin spi
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS);

  // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
  usb_msc.setID("Mandalou", "SDCard", "1.0");

  // Set read write callback
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);

  // Still initialize MSC but tell usb stack that MSC is not ready to read/write
  // If we don't initialize, board will be enumerated as CDC only
  usb_msc.setUnitReady(false);
  usb_msc.begin();


  Serial.print("\nInitializing SD card ... ");
  //Print pins to see where wiring SDs (pin by default)
  Serial.print("SCK:");
  Serial.println(SCK);
  Serial.print("MISO:");
  Serial.println(MISO);
  Serial.print("MOSI:");
  Serial.println(MOSI);
  //Serial.print("SS:"); Serial.println(SS)

  Serial.print("CS = ");
  Serial.println(SD_CS);

  // SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CHIP_SELECT_PIN); //ajouter pour check pin spi

  if (!sd.begin(SD_CS, SD_SCK_MHZ(50))) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    while (1) delay(1);
  }

  // Size in blocks (512 bytes)
#if SD_FAT_VERSION >= 20000
  uint32_t block_count = sd.card()->sectorCount();
#else
  uint32_t block_count = sd.card()->cardSize();
#endif

  Serial.print("Volume size (MB):  ");
  Serial.println((block_count / 2) / 1024);

  // Set disk size, SD block size is always 512
  usb_msc.setCapacity(block_count, 512);

  // MSC is ready for read/write
  usb_msc.setUnitReady(true);

  fs_changed = true;  // to print contents initially

  nowTimeMillis = millis();
  timeoutPressButtonLight = millis() + LONG_PRESS;

  //reset watchdog
  esp_task_wdt_reset();
}


/*
.__                                        ___.    
|  |   ____   ____ ______      __ __  _____\_ |__  
|  |  /  _ \ /  _ \\____ \    |  |  \/  ___/| __ \ 
|  |_(  <_> |  <_> )  |_> >   |  |  /\___ \ | \_\ \
|____/\____/ \____/|   __/____|____//____  >|___  /
                   |__| /_____/          \/     \/ 
*/

void loop_usb() {
  nowTimeMillis = millis();

  //reset watchdog
  esp_task_wdt_reset();

  //uart config
  uartConfig();

  //log data
  logUart(0);

  //activate detection button
  pinMode(PIN_POWER_BOARD_SWITCH_LIGHT, INPUT);
  lightLevel = digitalRead(PIN_POWER_BOARD_SWITCH_LIGHT);

  //allow power
  pinMode(PIN_POWER_BOARD_SWITCH_LIGHT, OUTPUT);
  digitalWrite(PIN_POWER_BOARD_SWITCH_LIGHT, HIGH);  //high:power ON switch9 low:power OFF switch9

  //POWER OFF
  if (lightLevel == 0) {
    timeoutPressButtonLight = millis() + LONG_PRESS;
  }

  if (millis() > timeoutPressButtonLight) {
    Serial.println("OFF");

    leds2[0] = CRGB(0, 0, 0);
    leds2[1] = CRGB(0, 0, 0);
    leds2[2] = CRGB(0, 0, 0);
    leds2[3] = CRGB(0, 0, 0);
    for (jj = 250; jj > 0; jj = jj - 27) {
      {
        for (ii = 0; ii < NUM_LEDS2; ii++) {
          leds2[ii] = CRGB(jj, 0, 0);
        }
        FastLED.show();
        delay(100);
        esp_task_wdt_reset();
      }
    }

    //LED OFF
    leds2[0] = CRGB(0, 0, 0);
    leds2[1] = CRGB(0, 0, 0);
    leds2[2] = CRGB(0, 0, 0);
    leds2[3] = CRGB(0, 0, 0);
    FastLED.show();
    delay(100);
    //POWER ON LED
    powerOffLed();

    for (ii = 0; ii < 4; ii++) {
      delay(1000);  //power off
      esp_task_wdt_reset();
    }

    //POWER OFF
    pinMode(PIN_POWER_BOARD_SWITCH_LIGHT, INPUT);
    //digitalWrite(PIN_POWER_BOARD_SWITCH_LIGHT, LOW);  //high:power ON switch9 low:power OFF switch9
    delay(200);
  }

  //flash rotating led for usb mode
  if (millis() > timeoutRefreshLed) {
    timeoutRefreshLed = millis() + UPDATE_LED;
    leds2[0] = CRGB(0, 0, 0);
    leds2[1] = CRGB(0, 0, 0);
    leds2[2] = CRGB(0, 0, 0);
    leds2[3] = CRGB(0, 0, 0);
    ii++;
    if (ii >= 4) ii = 0;
    if (fs_changed) {
      leds2[ii] = CRGB(200, 0, 0);
    } else {
      leds2[ii] = CRGB(0, 0, 200);
    }
    FastLED.show();
    delay(50);
  }


  //detect change mode pressing PLAY NEXT in same time
  //PART button play change power audio
  readButPlay = digitalRead(PIN_BUTTON_PLAY);
  readButNext = digitalRead(PIN_BUTTON_NEXT);


  //test press PLAY NEXT same time  BOOT MODE
  if (readButPlay == 0 && readButNext == 0) {
    if ((millis()) > lastDebounceTimePlayNext) {
      //reboot and change boot type
      //++bootMode;  //flip
      //Serial.println("Reboot mode :" + String(bootMode));

      ++bootMode2;
      // Ajoutez des données au document JSON
      jsonConfig["mode"] = "normal";  //normal ou usb
      jsonConfig["Nb"] = bootMode2;

      // convert json and print
      serializeJson(jsonConfig, jsonString);
      //Serial.println(jsonString);   //not secure see password uart
      Serial.println("BOOT APPLICATIVE MODE");

      //save on preference flash
      //preference on flash data etain after reboot
      preferences.begin("my-app", false);
      // Remove all preferences under the opened namespace
      //preferences.clear();
      // Or remove the counter key only
      //preferences.remove("counter");
      // Get the counter value, if the key does not exist, return a default value of 0
      // Note: Key name is limited to 15 chars.
      //bootMode2 = preferences.getUInt("bootmode", 0);
      // Increase counter by 1
      //bootMode2++;

      // Store the counter to the Preferences
      preferences.putUInt("bootmode", bootMode2);

      // Close the Preferences
      preferences.end();

      powerOnLed();
      while (true) {
        //for (int ki = 0; ki < 12; ki++) {
        leds2[0] = CRGB(0, 0, 0);
        leds2[1] = CRGB(0, 0, 0);
        leds2[2] = CRGB(0, 0, 0);
        leds2[3] = CRGB(0, 0, 0);
        ii++;
        if (ii >= 4) ii = 0;
        leds2[ii] = CRGB(0, 250, 0);
        FastLED.show();
        delay(100);
      }  //for watchdog

      leds2[0] = CRGB(0, 0, 0);
      leds2[1] = CRGB(0, 0, 0);
      leds2[2] = CRGB(0, 0, 0);
      leds2[3] = CRGB(0, 0, 0);
      FastLED.show();
      delay(100);

      //boot usb
      //setup();

      //esp_task_wdt_reset();
    }
  } else {
    lastDebounceTimePlayNext = millis() + longPressPlayNext;
    esp_task_wdt_reset();
  }


  //wait other process
  delay(50);
  /*
  if (fs_changed) {
    root.open("/");
    Serial.println("SD contents:");

    // Open next file in root.
    // Warning, openNext starts at the current directory position
    // so a rewind of the directory may be required.
    while (file.openNext(&root, O_RDONLY)) {
      file.printFileSize(&Serial);
      Serial.write(' ');
      file.printName(&Serial);
      if (file.isDir()) {
        // Indicate a directory.
        Serial.write('/');
      }
      Serial.println();
      file.close();
    }

    root.close();

    Serial.println();

    fs_changed = false;
    delay(500);  // refresh every 0.5 second
  }

  */
}



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



//Decryptage fichier

void decryptFile(const char *encryptedFileName, const char *decryptedFileName) {
  File encryptedFile = SD.open(encryptedFileName, FILE_READ);
  File decryptedFile = SD.open(decryptedFileName, FILE_WRITE);

  if (!encryptedFile) {
    timeDecodeCrypt = -1000;
    Serial.println("Erreur lors de l'ouverture du fichier encrypt (read)");
    return;
  }

  if (!decryptedFile) {
    timeDecodeCrypt = -2000;
    Serial.println("Erreur lors de l'ouverture des fichiers decrypt (write)");
    return;
  }

  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_dec(&aes, (const unsigned char *)keycrypt, strlen(keycrypt) * 8);  // Taille de clé en bits

  byte bufferIn[AES_BLOCK_SIZE];
  byte bufferOut[AES_BLOCK_SIZE];

  timeDecodeCrypt = millis();
  while (encryptedFile.available()) {

    //wathdog reset
    esp_task_wdt_reset();

    // Lire un bloc du fichier chiffré
    encryptedFile.read(bufferIn, AES_BLOCK_SIZE);

    // Déchiffrer le bloc
    mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_DECRYPT, bufferIn, bufferOut);

    // Écrire le bloc déchiffré dans le fichier déchiffré
    decryptedFile.write(bufferOut, AES_BLOCK_SIZE);
  }
  timeDecodeCrypt = millis() - timeDecodeCrypt;

  mbedtls_aes_free(&aes);
  encryptedFile.close();
  decryptedFile.close();

  Serial.println("Déchiffrement terminé.");
}



void uartConfig(void) {

  while (Serial.available() > 0) {
    updateConfig = 1;
    char incomingChar = Serial.read();
    receivedUartConfig += incomingChar;
    delay(2);  // Attendre un court instant pour permettre la réception de données
               //wathdog reset
    esp_task_wdt_reset();
  }

  // Si la chaîne JSON n'est pas vide
  if ((receivedUartConfig.length() > 0) && (receivedUartConfig.length() < 160) && (updateConfig == 1)) {
    updateConfig = 0;  //update only when receive data
    // Désérialiser le JSON
    DeserializationError error = deserializeJson(jsonConfig, receivedUartConfig);

    receivedUartConfig = "";

    // Vérifier les erreurs de désérialisation
    if (error) {
      Serial.println("Data format error");
      //Serial.println(error.c_str());
    } else {
      // Extraire les valeurs du JSON
      String acces = jsonConfig["acces"];
      int th1 = jsonConfig["th1"];
      int th2 = jsonConfig["th2"];
      int th6 = jsonConfig["th6"];
      int th7 = jsonConfig["th7"];
      long int sn = jsonConfig["sn"];
      uartdbg = jsonConfig["uartdbg"];
      verSw = jsonConfig["sw"];
      verHw = jsonConfig["hw"];
      gainHp = jsonConfig["gh"];
      gainCasq = jsonConfig["gc"];

      int timeAutoOffuart = jsonConfig["autooff"];


      // Faire quelque chose avec les valeurs extraites
      // Serial.println("Accès : " + acces);
      // Serial.println("Th1 : " + String(th1));
      // Serial.println("Th2 : " + String(th2));
      // Serial.println("Th6 : " + String(th6));
      // Serial.println("Th7 : " + String(th7));
      // Serial.println("SN : " + String(sn));
      // Serial.println("UARTDBG : " + String(uartdbg));

      if (acces == "fillthegoodpassword") {
        Serial.println("ACCESS ALLOWED");

        preferences.begin("my-app", false);

        if (timeAutoOffuart != 0) {
          preferences.putUInt("autooff", timeAutoOffuart);
          timeAutoOff = timeAutoOffuart * 60000;
          timeoutAutoOff = millis() + timeAutoOff;  //relaod autooff
        }

        if (verSw != 0) {
          preferences.putUInt("sw", verSw);
        }
        if (verHw != 0) {
          preferences.putUInt("hw", verHw);
        }
        if (gainHp != 0) {
          preferences.putUInt("gh", gainHp);
        }
        if (gainCasq != 0) {
          preferences.putUInt("gc", gainCasq);
        }

        if (th1 != 0) {
          intMatTheme[1] = th1;
          preferences.putUInt("th1", th1);
        }
        if (th2 != 0) {
          intMatTheme[2] = th2;
          preferences.putUInt("th2", th2);
        }
        if (th6 != 0) {
          intMatTheme[6] = th6;
          preferences.putUInt("th6", th6);
        }
        if (th7 != 0) {
          intMatTheme[7] = th7;
          preferences.putUInt("th7", th7);
        }
        if (sn != 0) {
          numeroSerie = sn;
          preferences.putLong("sn", sn);
        }
        if ((uartdbg == 0) || (uartdbg == 1) || (uartdbg == 2)) {
          preferences.putUInt("uartdbg", uartdbg);
        }

        //read prefs
        Serial.print("bootmode:");
        Serial.println(preferences.getUInt("bootmode", 0));

        Serial.print("audioread:");
        Serial.println(preferences.getUInt("audioread", 0));

        Serial.print("th1:");
        Serial.println(preferences.getUInt("th1", 0));
        Serial.print("th2:");
        Serial.println(preferences.getUInt("th2", 0));
        Serial.print("th3:");
        Serial.println(preferences.getUInt("th6", 0));
        Serial.print("th7:");
        Serial.println(preferences.getUInt("th7", 0));

        Serial.print("sn:");
        Serial.println(preferences.getLong("sn", 0));

        Serial.print("sw:");
        Serial.println(preferences.getUInt("sw", 0));
        Serial.print("hw:");
        Serial.println(preferences.getUInt("hw", 0));
        Serial.print("gh:");
        Serial.println(preferences.getUInt("gh", 0));
        Serial.print("gc:");
        Serial.println(preferences.getUInt("gc", 0));

        Serial.print("gc:");
        Serial.println(preferences.getUInt("gc", 0));

        Serial.print("autooff:");
        Serial.println(preferences.getUInt("autooff", 0));

        //reload data
        verSw = preferences.getUInt("sw", 0);
        verHw = preferences.getUInt("hw", 0);
        gainHp = preferences.getUInt("gh", 0);
        gainCasq = preferences.getUInt("gc", 0);
        uartdbg = preferences.getUInt("uartdbg", 1);  //uart debug by default

        // Close the Preferences
        preferences.end();

      } else {
        Serial.println("ACCESS NOT ALLOWED");
      }
    }
  }

  if (receivedUartConfig.length() > 160) {
    Serial.println("Data format error");
  }

  receivedUartConfig = "";
}

char fichierChoisi[200];

void change_song_name(void) {
  char local_name_directory[200] = "";

  modeRandNorm=backupModeRandom_sleep;

  //get emotion intDirEmotion, intthemeChoice
  switch (intNumeroDossier) {
    case 1:
      // Joie
      sprintf(local_name_directory, "/Joie");
      break;
    case 2:
      // Jalousie
      sprintf(local_name_directory, "/Jalousie");
      break;
    case 3:
      // Mon moment à moi
      sprintf(local_name_directory, "/Mon moment à moi");
      break;
    case 4:
      // Contenu libre
      sprintf(local_name_directory, "/Contenu libre");
      break;
    case 5:
      // Je m'endors paisiblement
      sprintf(local_name_directory, "/Je m'endors paisiblement");
      //modeRandNorm=0;  //force normal mode for sleep
      break;
    case 6:
      // Je me réveille de bonne humeur
      sprintf(local_name_directory, "/Je me réveille de bonne humeur");
      break;
    case 7:
      // Tristesse
      sprintf(local_name_directory, "/Tristesse");
      break;
    case 8:
      // Colère
      sprintf(local_name_directory, "/Colère");
      break;
    case 9:
      // Peur
      sprintf(local_name_directory, "/Peur");
      break;
    default:
      // Contenu libre
      sprintf(local_name_directory, "/Contenu libre");
      break;
  }

  //get thema
  switch (intthemeChoice) {
    case 1:
      // Méditations
      sprintf(local_name_directory, "%s/Méditations", local_name_directory);
      break;
    case 2:
      // Activités
      sprintf(local_name_directory, "%s/Activités", local_name_directory);
      break;
    case 3:
      // Musique
      sprintf(local_name_directory, "%s/Musique", local_name_directory);
      break;
    case 4:
      // Histoires audio
      sprintf(local_name_directory, "%s/Histoires audio", local_name_directory);
      break;
    case 5:
      // Bruits blancs et sons de la nature
      sprintf(local_name_directory, "%s/Bruits blancs et sons de la nature", local_name_directory);
      break;
    default:
      //Musique
      sprintf(local_name_directory, "%s/Musique", local_name_directory);
      break;
  }


  //Serial.println(local_name_directory);
  if (modeRandNorm == 1) {
    //random mode
    choisirFichierAleatoire(local_name_directory);
    //Serial.print("Fichier choisi aléatoirement : ");
    //Serial.println(fichierChoisi);

    sprintf(local_name_directory, "%s/%s", local_name_directory, fichierChoisi);
    //Serial.print("Fichier choisi aléatoirement : ");

    audio.connecttoSD(local_name_directory);
    if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
      Serial.println(local_name_directory);
    }

  } else {
    //normal mode
    choisirFichierSuivant(local_name_directory);
    sprintf(local_name_directory, "%s/%s", local_name_directory, fichierChoisi);
    audio.connecttoSD(local_name_directory);
    if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
      Serial.println(local_name_directory);
    }
  }
}



int choisirFichierAleatoire(const char *cheminDossier) {
  File repertoire = SD.open(cheminDossier);

  // List reading file for not repeating for long name and 1000 files
  static char fichiersParcourus[100000] = "";

  if (!repertoire) {
    Serial.println("Failed to open directory");
    return -1;
  }
  if (!repertoire.isDirectory()) {
    Serial.println("Not a directory");
    return -1;
  }

  int nombreFichiers = 0;

  if (nextSong >= intNbAudioFileInDir) nextSong = 0;  //intNbAudioFileInDir not use in random mode

  if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
    Serial.print("songs qty: ");
    Serial.print(intNbAudioFileInDir);
    Serial.print("-next: ");
    Serial.println(nextSong);
  }

  // Réinit file list
  if (nextSong == 0) {
    fichiersParcourus[0] = '\0';
  }

  while (File fichier = repertoire.openNextFile()) {
    char nomFichier[200];
    strcpy(nomFichier, fichier.name());
    fichier.close();

    // Vérifier si le fichier a déjà été choisi
    if (strstr(fichiersParcourus, nomFichier) != NULL) {
      continue;  // Passer au fichier suivant si celui-ci a déjà été choisi
    }


    nombreFichiers++;
    if ((random(0, nombreFichiers) == 0)) {
      //fichierChoisi = fichier.name();
      //strcpy(fichierChoisi, "");
      //strcat(fichierChoisi, fichier.name());
      strcpy(fichierChoisi, nomFichier);
    }
    //fichier.close();

    //reset watchdog
    esp_task_wdt_reset();
  }

  // Ajouter le fichier choisi à la liste des fichiers parcourus
  if (strlen(fichierChoisi) > 0) {
    strcat(fichiersParcourus, fichierChoisi);
    strcat(fichiersParcourus, ",");
  }

  repertoire.close();

  // Afficher le fichier choisi (ou le message si aucun fichier trouvé)
  if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
    if (strlen(fichierChoisi) > 0) {
      //Serial.print("Fichier choisi : ");
      //Serial.println(fichierChoisi);
    } else {
      Serial.println("Aucun fichier trouvé.");
    }

    //Serial.print("liste de fichier : ");
    //Serial.println(fichiersParcourus);
  }

  return 0;
}



int choisirFichierSuivant(const char *cheminDossier) {
  File repertoire = SD.open(cheminDossier);

  if (!repertoire) {
    Serial.println("Failed to open directory");
    return -1;
  }
  if (!repertoire.isDirectory()) {
    Serial.println("Not a directory");
    return -1;
  }

  int nombreFichiers = 0;

  //if (nextSong >= (intNbAudioFileInDir - 1)) nextSong = 0;  //intNbAudioFileInDir
  if (nextSong >= intNbAudioFileInDir) nextSong = 0;  //intNbAudioFileInDir

  if ((DEBUG_UART == 1) || (uartdbg == 1) || (uartdbg == 2)) {
    Serial.print("songs qty: ");
    Serial.print(intNbAudioFileInDir);
    Serial.print("-next: ");
    Serial.println(nextSong);
  }

  while (File fichier = repertoire.openNextFile()) {

    if (nombreFichiers == nextSong) {
      //fichierChoisi = fichier.name();
      strcpy(fichierChoisi, "");
      strcat(fichierChoisi, fichier.name());
    }
    nombreFichiers++;
    fichier.close();

    //reset watchdog
    esp_task_wdt_reset();
  }
  repertoire.close();

  return 0;
}




char fichierRacine[200];
char fichierToMove[200];

int fileClassifiation(void) {
  File repertoire = SD.open("/");
  int fileToMove = 0;
  unsigned long localTimeMillis = 0;

  if (!repertoire) {
    Serial.println("Failed to open directory");
    return -1;
  }
  if (!repertoire.isDirectory()) {
    Serial.println("Not a directory");
    return -1;
  }

  powerOnLed();
  while (File fichier = repertoire.openNextFile()) {
    fileToMove = 0;
    if (hasMp3Extension(fichier.name())) {

      //rotaring blue light during moving file
      if (millis() > localTimeMillis) {
        localTimeMillis = millis() + 200;  //200ms

        leds2[0] = CRGB(0, 0, 0);
        leds2[1] = CRGB(0, 0, 0);
        leds2[2] = CRGB(0, 0, 0);
        leds2[3] = CRGB(0, 0, 0);
        ii++;
        if (ii >= 4) ii = 0;
        leds2[ii] = CRGB(0, 0, 250);
        FastLED.show();
        delay(10);
      }

      strcpy(fichierRacine, "/");
      strcat(fichierRacine, fichier.name());



      if (containsSubstring(fichier.name(), "CO-AC-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Colère/Activités/");
      }
      if (containsSubstring(fichier.name(), "CO-BE-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Colère/Bruits blancs et sons de la nature/");
      }
      if (containsSubstring(fichier.name(), "CO-HI-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Colère/Histoires audio/");
      }
      if (containsSubstring(fichier.name(), "CO-ME-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Colère/Méditations/");
      }
      if (containsSubstring(fichier.name(), "CO-MU-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Colère/Musique/");
      }



      if (containsSubstring(fichier.name(), "CL-AC-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Contenu libre/Activités/");
      }
      if (containsSubstring(fichier.name(), "CL-BE-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Contenu libre/Bruits blancs et sons de la nature/");
      }
      if (containsSubstring(fichier.name(), "CL-HI-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Contenu libre/Histoires audio/");
      }
      if (containsSubstring(fichier.name(), "CL-ME-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Contenu libre/Méditations/");
      }
      if (containsSubstring(fichier.name(), "CL-MU-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Contenu libre/Musique/");
      }



      if (containsSubstring(fichier.name(), "JA-AC-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Jalousie/Activités/");
      }
      if (containsSubstring(fichier.name(), "JA-BE-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Jalousie/Bruits blancs et sons de la nature/");
      }
      if (containsSubstring(fichier.name(), "JA-HI-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Jalousie/Histoires audio/");
      }
      if (containsSubstring(fichier.name(), "JA-ME-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Jalousie/Méditations/");
      }
      if (containsSubstring(fichier.name(), "JA-MU-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Jalousie/Musique/");
      }



      if (containsSubstring(fichier.name(), "RE-AC-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Je me réveille de bonne humeur/Activités/");
      }
      if (containsSubstring(fichier.name(), "RE-BE-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Je me réveille de bonne humeur/Bruits blancs et sons de la nature/");
      }
      if (containsSubstring(fichier.name(), "RE-HI-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Je me réveille de bonne humeur/Histoires audio/");
      }
      if (containsSubstring(fichier.name(), "RE-ME-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Je me réveille de bonne humeur/Méditations/");
      }
      if (containsSubstring(fichier.name(), "RE-MU-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Je me réveille de bonne humeur/Musique/");
      }



      if (containsSubstring(fichier.name(), "EN-AC-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Je m'endors paisiblement/Activités/");
      }
      if (containsSubstring(fichier.name(), "EN-BE-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Je m'endors paisiblement/Bruits blancs et sons de la nature/");
      }
      if (containsSubstring(fichier.name(), "EN-HI-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Je m'endors paisiblement/Histoires audio/");
      }
      if (containsSubstring(fichier.name(), "EN-ME-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Je m'endors paisiblement/Méditations/");
      }
      if (containsSubstring(fichier.name(), "EN-MU-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Je m'endors paisiblement/Musique/");
      }



      if (containsSubstring(fichier.name(), "JO-AC-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Joie/Activités/");
      }
      if (containsSubstring(fichier.name(), "JO-BE-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Joie/Bruits blancs et sons de la nature/");
      }
      if (containsSubstring(fichier.name(), "JO-HI-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Joie/Histoires audio/");
      }
      if (containsSubstring(fichier.name(), "JO-ME-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Joie/Méditations/");
      }
      if (containsSubstring(fichier.name(), "JO-MU-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Joie/Musique/");
      }



      if (containsSubstring(fichier.name(), "MO-AC-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Mon moment à moi/Activités/");
      }
      if (containsSubstring(fichier.name(), "MO-BE-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Mon moment à moi/Bruits blancs et sons de la nature/");
      }
      if (containsSubstring(fichier.name(), "MO-HI-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Mon moment à moi/Histoires audio/");
      }
      if (containsSubstring(fichier.name(), "MO-ME-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Mon moment à moi/Méditations/");
      }
      if (containsSubstring(fichier.name(), "MO-MU-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Mon moment à moi/Musique/");
      }



      if (containsSubstring(fichier.name(), "PE-AC-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Peur/Activités/");
      }
      if (containsSubstring(fichier.name(), "PE-BE-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Peur/Bruits blancs et sons de la nature/");
      }
      if (containsSubstring(fichier.name(), "PE-HI-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Peur/Histoires audio/");
      }
      if (containsSubstring(fichier.name(), "PE-ME-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Peur/Méditations/");
      }
      if (containsSubstring(fichier.name(), "PE-MU-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Peur/Musique/");
      }



      if (containsSubstring(fichier.name(), "TR-AC-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Tristesse/Activités/");
      }
      if (containsSubstring(fichier.name(), "TR-BE-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Tristesse/Bruits blancs et sons de la nature/");
      }
      if (containsSubstring(fichier.name(), "TR-HI-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Tristesse/Histoires audio/");
      }
      if (containsSubstring(fichier.name(), "TR-ME-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Tristesse/Méditations/");
      }
      if (containsSubstring(fichier.name(), "TR-MU-")) {
        fileToMove = 1;
        strcpy(fichierToMove, "/Tristesse/Musique/");
      }



      //move file
      if (fileToMove == 1) {
        strcat(fichierToMove, fichier.name());
        renameFile(SD, fichierRacine, fichierToMove);
        Serial.print("file move :");
        Serial.print(fichierRacine);
        Serial.print(" -> ");
        Serial.println(fichierToMove);
      }

      fichier.close();

      //reset watchdog
      esp_task_wdt_reset();
    }  //if mp3 file
  }

  //power off led
  leds2[0] = CRGB(0, 0, 0);
  leds2[1] = CRGB(0, 0, 0);
  leds2[2] = CRGB(0, 0, 0);
  leds2[3] = CRGB(0, 0, 0);
  FastLED.show();
  delay(10);

  powerOffLed();
  repertoire.close();
  return 0;
}


bool hasMp3Extension(const char *filename) {
  // Trouver le point dans le nom du fichier
  const char *point = strrchr(filename, '.');

  // Vérifier si le point existe et s'il est suivi de "mp3"
  if (point != NULL && strcmp(point + 1, "mp3") == 0) {
    return true;
  }

  return false;
}



bool containsSubstring(const char *filename, const char *substring) {
  return strstr(filename, substring) != NULL;
}