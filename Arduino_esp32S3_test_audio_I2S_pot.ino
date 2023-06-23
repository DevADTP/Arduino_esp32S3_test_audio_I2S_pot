#include "Arduino.h"
#include "WiFi.h"
#include "Audio.h"
#include <FastLED.h>
#include "AiEsp32RotaryEncoder.h"

#define I2S_DOUT    35
#define I2S_BCLK    36
#define I2S_LRC     37
#define I2S_GAIN    39

#define PIN_VOLUME   4

#define PIN_BUTTON_PLAY 45


//use for turn off red light indicator after 10s
#define TIME_PICTURE_END  10000    //milliseconds (10s)

#define ROTARY_ENCODER_A_PIN 48
#define ROTARY_ENCODER_B_PIN 47
#define ROTARY_ENCODER_BUTTON_PIN 21
#define ROTARY_ENCODER_VCC_PIN 19
#define ROTARY_ENCODER_STEPS 4
#define CYCLE_ROT  24

//instead of changing here, rather change numbers above
//AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

//neopixel
#define PIN_NEOPIXEL 40
#define PIN_NEOPIXEL_KIT 38

#define PERIOD_READ_VOLUME   50  //ms

#define PERIOD_CHANGE_GAIN   5000  //ms updateTimeGain

Audio audio;

String ssid =    "*******************";
String password = "******************";

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



//void rotary_onButtonClick()
//{
//  static unsigned long lastTimePressed = 0;
//  //ignore multiple press in that time milliseconds
//  if (millis() - lastTimePressed < 500)
//  {
//    return;
//  }
//  lastTimePressed = millis();
//  Serial.print("button pressed ");
//  Serial.print(millis());
//  Serial.println(" milliseconds after restart");
//
//  if (flip_light == 1)
//  {
//    flip_light = 0;
//  }
//  else
//  {
//    flip_light = 1;
//  }
//
//  ulong_time_picture = millis() + TIME_PICTURE_END;
//
//}
//
//
//
//void rotary_loop()
//{
//  //dont print anything unless value changed
//  if (rotaryEncoder.encoderChanged())
//  {
//    //Serial.print("Value: ");
//    numero_led = rotaryEncoder.readEncoder();
//    Serial.println(numero_led);
//    ulong_time_picture = millis() + TIME_PICTURE_END;
//  }
//  if (rotaryEncoder.isEncoderButtonClicked())
//  {
//    rotary_onButtonClick();
//  }
//}
//
//
//
//void IRAM_ATTR readEncoderISR()
//{
//  rotaryEncoder.readEncoder_ISR();
//}



void setup() {
  delay(1000);
  Serial.begin(2000000);
  while (!Serial) {
  }

  Serial.println("START PROGRAM");

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

//  //we must initialize rotary encoder
//  rotaryEncoder.begin();
//  rotaryEncoder.setup(readEncoderISR);
//  //set boundaries and if values should cycle or not
//  //in this example we will set possible values between 0 and 1000;
//  bool circleValues = true;
//  rotaryEncoder.setBoundaries(0, CYCLE_ROT, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
//
//  /*Rotary acceleration introduced 25.2.2021.
//     in case range to select is huge, for example - select a value between 0 and 1000 and we want 785
//     without accelerateion you need long time to get to that number
//     Using acceleration, faster you turn, faster wil
//     l the value raise.
//     For fine tuning slow down.
//  */
//  //rotaryEncoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
//  rotaryEncoder.setAcceleration(1); //250 or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
//
//  ulong_time_picture = millis() + TIME_PICTURE_END;


  //audio web radio
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(0);
  audio.connecttohost("http://vis.media-ice.musicradio.com/CapitalMP3");

  pinMode(PIN_BUTTON_PLAY, INPUT_PULLUP);


}



void loop()
{
  nowTimeMillis = millis();
  ulong_time_now = millis();

  //in loop call your custom function which will process rotary encoder values
  //rotary_loop();

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
    //Serial.print(valVolume);
    //Serial.print(",");

    valVolume = (valVolume * 100) / 4095;
    //    Serial.print(valVolume);
    //    Serial.print(",");

    if (valVolume >= 100) valVolume = 100;
    if (valVolume <= 0) valVolume = 0;
    //    Serial.print(valVolume);
    //    Serial.print(",");

    if ( (valVolume - valVolumeold) > 5)
    {
      updatevolume = 1;
    }

    if ( (valVolumeold - valVolume) > 5)
    {
      updatevolume = 1;
    }

    if (updatevolume == 1)
    {
      Serial.print(valVolume);
      Serial.print(",");
      Serial.println(updatevolume);
      updatevolume = 0;
      valVolumeold = valVolume;
      audio.setVolume(valVolume);

      for (i = 0; i < NUM_LEDS; i++)
      {
        leds[i] = CRGB::Black;
      }

      for (i = 0; i < map(valVolume, 0, 100, 0, NUM_LEDS); i++)
      {
        couleur = CRGB(200, 0, 0);
        leds[i] = couleur;
      }
      FastLED.show();

    }
    else
    {
      //      Serial.println(updatevolume);
      //init all led with OFF or ON
//      for (i = 0; i < NUM_LEDS; i++)
//      {
//        if (flip_light == 1)
//        {
//          leds[i] = CRGB::White;
//        }
//        else
//        {
//          leds[i] = CRGB::Black;
//        }
//      }
//
//      if (ulong_time_now >= ulong_time_picture)
//      {
//        if (flip_light == 1)
//        {
//          leds[numero_led] = CRGB::White;
//        }
//        else
//        {
//          leds[numero_led] = CRGB::Black;
//        }
//      }
//      else
//      {
//        leds[numero_led] = CRGB::Red;
//      }
//
//      FastLED.show();
    }

  }

  audio.loop();
}
