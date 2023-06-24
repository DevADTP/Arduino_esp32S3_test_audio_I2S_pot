/*
  carte dev
  - esp32 : https://espressif.github.io/arduino-esp32/package_esp32_index.json

  librairies
  - audio I2S : https://github.com/schreibfaul1/ESP32-audioI2S.git
  - neopixel  : https://github.com/FastLED/FastLED.git
  - rotary encoder : https://github.com/igorantolic/ai-esp32-rotary-encoder.git1
*/

#include <Arduino.h>
#include <WiFi.h>
#include <Audio.h>
#include <FastLED.h>
#include <AiEsp32RotaryEncoder.h>
#include "SD.h"
#include "FS.h"
#include "SPIFFS.h"
#include "SPI.h"

// PINOUT SD CARD
#define SD_CS         10
#define SPI_MOSI      11    // SD Card
#define SPI_MISO      13
#define SPI_SCK       12

//PINOUT I2S CARD (AUDIO 3W)
#define I2S_DOUT    35
#define I2S_BCLK    36
#define I2S_LRC     37
#define I2S_GAIN    39

//potentiometre volume
#define PIN_VOLUME   4

//button PLAY (used as gain audio)
#define PIN_BUTTON_PLAY 45

//use for turn off red light indicator after 10s
#define TIME_PICTURE_END  10000    //milliseconds (10s)

#define ROTARY_ENCODER_A_PIN 5
#define ROTARY_ENCODER_B_PIN 6
#define ROTARY_ENCODER_BUTTON_PIN 7
#define ROTARY_ENCODER_VCC_PIN 17
#define ROTARY_ENCODER_STEPS 4
#define CYCLE_ROT  24

//instead of changing here, rather change numbers above
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

//neopixel
#define PIN_NEOPIXEL 40
#define PIN_NEOPIXEL_KIT 38

#define PERIOD_READ_VOLUME   50  //ms

#define PERIOD_CHANGE_GAIN   5000  //ms updateTimeGain

Audio audio;

String ssid =    "*********";
String password = "*******";

long int valVolume = 0;
long int valVolumeold = 0;
int updatevolume = 0;

unsigned long nowTimeMillis = 0;
unsigned long updateTimeVolume = 0;
unsigned long updateTimeGain = 0;

int i = 0; //for

int changeGain = 0;

int readButPlay = 0;
int buttonState;             // the current reading from the input pin
int lastButtonState = HIGH;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers


//fastled
#define NUM_LEDS 24
#define BRIGHTNESS  20
#define DATA_PIN 40
#define DATA_PIN2 38
#define CLOCK_PIN 13  //not use

// Define the array of leds
CRGB leds[NUM_LEDS];
CRGB couleur = CRGB(0, 0, 0);


int numero_led = 0;
int flip_light = 0;
unsigned long ulong_time_now = 0;
unsigned long ulong_time_picture = 0;  //TIME_PICTURE_END

int intNbAudioFileInDir = 0;
int intNumeroDossier = 1;
int intNombreDossier = 5;

void rotary_onButtonClick()
{
  static unsigned long lastTimePressed = 0;
  //ignore multiple press in that time milliseconds
  if (millis() - lastTimePressed < 500)
  {
    return;
  }
  lastTimePressed = millis();
  Serial.print("button pressed ");
  Serial.print(millis());
  Serial.println(" milliseconds after restart");

  if (flip_light == 1)
  {
    flip_light = 0;
  }
  else
  {
    flip_light = 1;
  }

  ulong_time_picture = millis() + TIME_PICTURE_END;

}



void rotary_loop()
{
  //dont print anything unless value changed
  if (rotaryEncoder.encoderChanged())
  {
    //Serial.print("Value: ");
    numero_led = rotaryEncoder.readEncoder();
    intNumeroDossier = (numero_led % intNombreDossier) + 1;

    intNbAudioFileInDir = 0;
    listDir(SD, "/05", 1);

    Serial.print("Dossier:");
    Serial.print("intNumeroDossier");
    Serial.println(numero_led);

    Serial.println("Led anneau");
    Serial.println(numero_led);

    switch ( numero_led ) {
      case 1:
        audio.connecttoFS(SD, "/05/001.mp3");
        break;
      case 2:
        audio.connecttoFS(SD, "/05/002.mp3");
        break;
      default:
        audio.connecttoFS(SD, "/05/003.mp3");
        break;
    }

    ulong_time_picture = millis() + TIME_PICTURE_END;
  }
  if (rotaryEncoder.isEncoderButtonClicked())
  {
    rotary_onButtonClick();
  }
}



void IRAM_ATTR readEncoderISR()
{
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



void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
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

void createDir(fs::FS &fs, const char * path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char * path) {
  Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) {
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char * path) {
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

void writeFile(fs::FS &fs, const char * path, const char * message) {
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

void appendFile(fs::FS &fs, const char * path, const char * message) {
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

void renameFile(fs::FS &fs, const char * path1, const char * path2) {
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char * path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char * path) {
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



void setup() {
  delay(1000);
  Serial.begin(2000000);
  while (!Serial) {
  }
  Serial.println("************************************************************");
  Serial.println("START PROGRAM");
  Serial.println("************************************************************");

  Serial.println("SPI board default:");
  Serial.print("MOSI: ");
  Serial.println(MOSI);
  Serial.print("MISO: ");
  Serial.println(MISO);
  Serial.print("SCK: ");
  Serial.println(SCK);
  Serial.print("SS: ");
  Serial.println(SS);

  //sd card
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, LOW);
  delay(10);
  digitalWrite(SD_CS, HIGH);
  //SPI.begin(SCK, MISO, MOSI, SS);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS);

  //SD card
  initSDCard();

  //  Serial.println("dir racine");
  //  listDir(SD, "/", 0);
  Serial.println("Contenu carte SD : ");
  intNbAudioFileInDir = 0;
  listDir(SD, "/", 1);

  intNbAudioFileInDir = 0;
  listDir(SD, "/05", 0);

  Serial.println("Contenu dossier 01 : ");
  intNbAudioFileInDir = 0;
  listDir(SD, "/01", 1);
  //  Serial.println("dir 01");
  //  listDir(SD, "/01", 0);

  //  writeFile(SD, "/hello.txt", "Hello ");
  //  appendFile(SD, "/hello.txt", "World!\n");
  //  readFile(SD, "/hello.txt");
  //  deleteFile(SD, "/foo.txt");
  //  renameFile(SD, "/hello.txt", "/foo.txt");
  //  readFile(SD, "/foo.txt");
  //  testFileIO(SD, "/test.txt");
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));


  //wifi
  WiFi.disconnect();

  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid.c_str(), password.c_str());

  Serial.print("WIFI:");
  while (WiFi.status() != WL_CONNECTED)
  {

    Serial.print(".");
    delay(500);
  }
  Serial.println("wifi ok");
  delay(1500);

  Serial.println("init neopixel");
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed
  FastLED.setBrightness( BRIGHTNESS );

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
  rotaryEncoder.setBoundaries(0, CYCLE_ROT, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)

  /*Rotary acceleration introduced 25.2.2021.
     in case range to select is huge, for example - select a value between 0 and 1000 and we want 785
     without accelerateion you need long time to get to that number
     Using acceleration, faster you turn, faster wil
     l the value raise.
     For fine tuning slow down.
  */
  //rotaryEncoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
  rotaryEncoder.setAcceleration(1); //250 or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration

  ulong_time_picture = millis() + TIME_PICTURE_END;


  //audio web radio
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(0);
  //stream music
  //audio.connecttohost("http://vis.media-ice.musicradio.com/CapitalMP3");

  //sd musique
  //audio.connecttoFS(SD, "/01/001.mp3");
  audio.connecttoFS(SD, "/05/001.mp3");

  pinMode(PIN_BUTTON_PLAY, INPUT_PULLUP);
}



void loop()
{
  nowTimeMillis = millis();
  ulong_time_now = millis();

  //in loop call your custom function which will process rotary encoder values
  rotary_loop();

  //PART button play change power audio
  readButPlay = digitalRead(PIN_BUTTON_PLAY);

  if (readButPlay != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the readButPlay is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (readButPlay != buttonState) {
      buttonState = readButPlay;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        changeGain++;

        if (changeGain > 2) changeGain = 0;

        switch ( changeGain ) {
          case 0:
            //3db
            Serial.println("gain:15dB");  //15dB
            pinMode(I2S_GAIN, OUTPUT);
            digitalWrite(I2S_GAIN, LOW);
            break;
          case 1:
            //15db
            Serial.println("gain:3dB"); //3dB
            pinMode(I2S_GAIN, OUTPUT);
            digitalWrite(I2S_GAIN, HIGH);
            break;
          default:
            //12db
            Serial.println("gain:12dB"); //12dB
            pinMode(I2S_GAIN, INPUT);
            break;
        }
      }
    }
  }

  lastButtonState = readButPlay;
  //END PART button play change power audio


  //refresh volume with led
  if (nowTimeMillis > updateTimeVolume )
  {
    updateTimeVolume = nowTimeMillis + PERIOD_READ_VOLUME;

    valVolume = analogRead(PIN_VOLUME);
    valVolume = (valVolume * 21) / 4095;

    if (valVolume >= 21) valVolume = 21;
    if (valVolume <= 0) valVolume = 0;

    if ( (valVolume - valVolumeold) > 1) updatevolume = 1;
    if ( (valVolumeold - valVolume) > 1) updatevolume = 1;

    if (updatevolume == 1)
    {
      Serial.print(valVolume);
      Serial.print(",");
      Serial.println(updatevolume);
      updatevolume = 0;
      valVolumeold = valVolume;
      audio.setVolume(valVolume); // 0...21

      for (i = 0; i < NUM_LEDS; i++)
      {
        leds[i] = CRGB::Black;
      }

      for (i = 0; i < map(valVolume, 0, 21, 0, NUM_LEDS); i++)
      {
        couleur = CRGB(200, 0, 0);
        leds[i] = couleur;
      }
      FastLED.show();

    }
    else
    {
      //Serial.println(updatevolume);
      //init all led with OFF or ON
      for (i = 0; i < NUM_LEDS; i++)
      {
        if (flip_light == 1)
        {
          leds[i] = CRGB::White;
        }
        else
        {
          leds[i] = CRGB::Black;
        }
      }

      if (ulong_time_now >= ulong_time_picture)
      {
        if (flip_light == 1)
        {
          leds[numero_led] = CRGB::White;
        }
        else
        {
          leds[numero_led] = CRGB::Black;
        }
      }
      else
      {
        leds[numero_led] = CRGB::Red;
      }

      FastLED.show();
    }

  }

  audio.loop();
}
