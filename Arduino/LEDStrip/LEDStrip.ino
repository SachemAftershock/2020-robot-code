#include <Wire.h>
#include "FastLED.h"
#include "Timer.h"

#define NUM_LEDS 30
#define DATA_PIN 6
#define COLOR_ORDER GRB
#define BULLET_BUFFER 4

Timer t;

char ch;
CRGB leds[NUM_LEDS];
enum systemModeEnum {Off, Red, Blue, Green, Pink, Teal, Rainbow, WhiteBlink, OrangeBullet, Yellow, Orange, Purple, White, Color, BlueAndRed, YellowAndRed, YellowAndBlue, RedBlink, BlueBlink, YellowBlink, TealBullet, RedBullet, WhiteBullet} currentSystemMode = Rainbow;
systemModeEnum lastSystemMode = Off;
int commandReceivedLedOnboardArduino = 13;

bool blinkOn = false;

void toggleCommandArrivedLed(int LightDuration = 200)
{
  digitalWrite(commandReceivedLedOnboardArduino, HIGH);
  delay(LightDuration);
  digitalWrite(commandReceivedLedOnboardArduino, LOW);
  delay(LightDuration);
}

void processCommand(char theCh) {
  Serial.print("Recieved Command ");
  Serial.print(theCh);
  Serial.print("\n"); 
  switch (theCh)
  {
    case 'r':
      currentSystemMode = Red;
      Serial.write("Commanded RED. \n");
      break;
   
    case 'g':
      currentSystemMode = Green;
      Serial.write("Commanded GREEN. \n");
      break;

    case 'b':
      currentSystemMode = Blue;
      Serial.write("Commanded BLUE. \n");
      break;

    case 'p':
      currentSystemMode = Pink;
      Serial.write("Commanded PINK. \n");
      break;

    case 't':
      currentSystemMode = Teal;
      Serial.write("Commanded TEAL. \n");
      break;

    case 'n':
      currentSystemMode = Off;
      Serial.write("Commanded BLANK. \n");
      break;

    case 'a':
      currentSystemMode = Rainbow;
      Serial.write("Commanded RAINBOW. \n");
      break;

    case 'o':
      currentSystemMode = WhiteBlink;
      Serial.write("Commanded WHITEBLINK. \n");
      break;
      
    case 'l':
      currentSystemMode = OrangeBullet;
      Serial.write("Commanded ORANGEBULLET. \n");
      break;
      
    case 'y':
      currentSystemMode = Yellow;
      Serial.write("Commanded YELLOW. \n");
      break;
      
    case 'm':
      currentSystemMode = Orange;
      Serial.write("Commanded ORANGE. \n");
      break;

    case 'u':
      currentSystemMode = Purple;
      Serial.write("Commanded PURPLE. \n");
      break;

     case 'w':
      currentSystemMode = White;
      Serial.write("Commanded WHITE. \n");
      break;
      
     case 'c':
      currentSystemMode = Color;
      Serial.write("Commanded COLOR. \n");
      break;
       
        case 's':
      currentSystemMode = BlueAndRed;
      Serial.write("Commanded BLUEANDRED. \n");
      break;

        case 'x':
      currentSystemMode = YellowAndRed;
      Serial.write("Commanded YELLOWANDRED. \n");
      break;

        case 'v':
      currentSystemMode = YellowAndBlue;
      Serial.write("Commanded YELLOWANDBLUE. \n");
      break;

      case 'f':
      currentSystemMode = RedBlink;
      Serial.write("Commanded REDBLINK. \n");
      break;

      case 'h':
      currentSystemMode = BlueBlink;
      Serial.write("Commanded BLUEBLINK. \n");
      break;

      case 'j':
      currentSystemMode = YellowBlink;
      Serial.write("Commanded YELLOWBLINK. \n");
      break;
      
       case 'd':
      currentSystemMode = TealBullet;
      Serial.write("Commanded TEALBULLET. \n");
      break;

         case 'e':
      currentSystemMode = RedBullet;
      Serial.write("Commanded REDBULLET. \n");
      break;

   case 'i':
      currentSystemMode = WhiteBullet;
      Serial.write("Commanded WHITEBULLET. \n");
      break;
      
    default:
      Serial.write("Commanded not recognized. \n");
      break;
  }
}

void rainbow(uint8_t wait)
{
  uint16_t hue;
  FastLED.clear();
  for (hue = 10; hue < 255 * 3; hue++)
  {
    fill_rainbow( &(leds[0]), NUM_LEDS /*led count*/, hue /*starting hue*/);
    FastLED.show();
  }
  return;
}

void bullet(byte r, byte g, byte b)
{
  CRGB color = CRGB(r, g, b);
  FastLED.clear();
 
  {
  for (int i = 0; i < BULLET_BUFFER; i++) 
  {
    leds[i] = color;
    delay(15);
    FastLED.show();
  }
  for (int i = BULLET_BUFFER; i < NUM_LEDS; i++) 
  {
    leds[i-BULLET_BUFFER] = CRGB::Black;
    leds[i] = color;
    FastLED.show();
    delay(10);
  }
  for (int i = NUM_LEDS - BULLET_BUFFER; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB::Black;
    FastLED.show();
    delay(10);
    
  }
  }
}

void processCurrentMode() {
  switch (currentSystemMode) {
    FastLED.clear();

    case WhiteBlink:
      blinkOn = !blinkOn;
      if (currentSystemMode == WhiteBlink) {
        if (blinkOn) {
          FastLED.clear();
          fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 255, 239, 213) ); //Amber
          FastLED.show();
        }
        else {
          FastLED.clear();
          fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 0, 0, 0) );
          FastLED.show();
        }
      }
      break;
      
    case Red:
      if (lastSystemMode != Red) {
        FastLED.clear();
        fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 255, 0, 0) );
        FastLED.show();
        lastSystemMode = Red;
      }
      break;
      
    case Green:
      FastLED.clear();
      if (lastSystemMode != Green) {
      FastLED.clear();
      fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 0, 255, 0) );
      FastLED.show();
      lastSystemMode = Green;
      }
      break;

    case Blue:
      if (lastSystemMode != Blue) {
      FastLED.clear();
      fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 0, 0, 255) );
      FastLED.show();
      lastSystemMode = Blue;
      }
      break;
    case Pink:
      if (lastSystemMode != Pink) {
      FastLED.clear();
      fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 255, 20, 147) );
      FastLED.show();
      lastSystemMode = Pink;
      }
      break;

    case Teal:
      if (lastSystemMode != Teal) {
      FastLED.clear();
      fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 0, 255, 255) );
      FastLED.show();
      lastSystemMode = Teal;
      }
      break;

    case Off:
      if (lastSystemMode != Off) {
      FastLED.clear();
      fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 0, 0, 0) );
      FastLED.show();
      lastSystemMode = Off;
      }
      break;

    case Rainbow:
      //if (lastSystemMode != Rainbow) {
      FastLED.clear();
      rainbow(0);
      FastLED.show();
      lastSystemMode = Rainbow;
      //}
      break;
      
    case OrangeBullet:
      //if (lastSystemMode != Bullet) {
        bullet( 255, 40, 0); //Amber colored 255, 194, 0
        lastSystemMode = OrangeBullet;
      //}
      break;
      
      case Yellow:
      if (lastSystemMode != Yellow) {
      FastLED.clear();
      fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 255, 111, 0) );
      /*Yellow was originally 158,128,0*/
      FastLED.show();
      lastSystemMode = Yellow;
      }
      break;
      
      case Orange:
      if (lastSystemMode != Orange) {
      FastLED.clear();
      fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 255, 40, 0) );
      FastLED.show();
      lastSystemMode = Orange;
      }
      break;
      
      case Purple:
      if (lastSystemMode != Purple) {
      FastLED.clear();
      fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 126, 13, 203) );
      FastLED.show();
      lastSystemMode = Purple;
      }
      break;
      case White:
      if (lastSystemMode != White) {
      FastLED.clear();
      fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 128, 128, 128) );
      FastLED.show();
      lastSystemMode = White;
      }
      break;
      
      case Color:
      //if (lastSystemMode != Color) {
      FastLED.clear();
      fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 255, 0, 0) );
      FastLED.show();
      delay(500);
      FastLED.clear();
      fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB(255, 111, 0) );
      FastLED.show();
      delay(500);
      FastLED.clear();
      fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB(0, 255, 0) );
      FastLED.show();
      delay(500);
      FastLED.clear();
      fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 0, 0, 255) );
      FastLED.show();
      delay(500);
      lastSystemMode = Color;
      //}
      break;
      
        case BlueAndRed:
      //if (lastSystemMode != BlueAndRed) {
      FastLED.clear();
      fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 255, 0, 0) );
      FastLED.show();
      delay(500);
      FastLED.clear();
      fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 0, 0, 255) );
      FastLED.show();
      delay(500);
      lastSystemMode = BlueAndRed;
      //}
      break;
      
       case YellowAndRed:
      //if (lastSystemMode != YellowAndRed) {
      FastLED.clear();
      fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 255, 0, 0) );
      FastLED.show();
      delay(500);
      FastLED.clear();
      fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB(255, 111, 0) );
      FastLED.show();
      delay(500);
      lastSystemMode = YellowAndRed;
      //}
      break;
      
        case YellowAndBlue:
      //if (lastSystemMode != YellowAndBlue) {
      FastLED.clear();
      fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB(0, 0, 255) );
      FastLED.show();
      delay(500);
      FastLED.clear();
      fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB(255, 111, 0) );
      FastLED.show();
      delay(500);
      lastSystemMode = YellowAndBlue;
      //}
      break;

           case RedBlink:
      blinkOn = !blinkOn;
      if (currentSystemMode == RedBlink) {
        if (blinkOn) {
          FastLED.clear();
          fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 255, 0, 0) ); 
          FastLED.show();
        }
        else {
          FastLED.clear();
          fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 0, 0, 0) );
          FastLED.show();
        }
      }
      break;

          case BlueBlink:
      blinkOn = !blinkOn;
      if (currentSystemMode == BlueBlink) {
        if (blinkOn) {
          FastLED.clear();
          fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 0, 0, 255) );
          FastLED.show();
        }
        else {
          FastLED.clear();
          fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 0, 0, 0) );
          FastLED.show();
        }
      }
      break;
          case YellowBlink:
      blinkOn = !blinkOn;
      if (currentSystemMode == YellowBlink) {
        if (blinkOn) {
          FastLED.clear();
          fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 255, 111, 0) );
          FastLED.show();
        }
        else {
          FastLED.clear();
          fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 0, 0, 0) );
          FastLED.show();
        }
      }
      break;

        case TealBullet:
      //if (lastSystemMode != Bullet) {
        bullet( 0, 255, 255); //Amber colored 255, 194, 0
        lastSystemMode = TealBullet;
      //}
      break;

        case RedBullet:
      //if (lastSystemMode != Bullet) {
        bullet( 255, 0, 0); //Amber colored 255, 194, 0
        lastSystemMode = RedBullet;
      //}
      break;

        case WhiteBullet:
      //if (lastSystemMode != Bullet) {
        bullet( 255, 239, 213); //Amber colored 255, 194, 0
        lastSystemMode = WhiteBullet;
      //}
      break;
      
    default:
      Serial.print("Unrecognized Mode");
      break;
       
  }
}
void setup()
{
  FastLED.addLeds<WS2812B, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  Wire.begin(1); //communicate on this address
  Wire.onReceive(receiveEvent);//when communication is successful,
  Serial.begin(9600);          //go to the function
  FastLED.show();
  t.every(500, processCurrentMode);
  pinMode(commandReceivedLedOnboardArduino, OUTPUT);
  toggleCommandArrivedLed();
  Serial.write("Arduino-Slave-I2C-NewLEDStrip Initialization Complete. \n");
}

void loop()
{
  t.update();
  if (Serial.available()) {
    ch = Serial.read();
    processCommand(ch);
  }

}

void receiveEvent(int howMany)
{
  Serial.write("receiveEvent Invoked. \n");
  while (Wire.available()) //when you recieve a byte (through i2c)
  {
    toggleCommandArrivedLed();

    unsigned char state = Wire.read();
    Serial.write("Received Char: <");
    Serial.write(state);
    Serial.write("> \n");
    //not sure if it should be a normal or unsigned char

    processCommand(state);
  }
}
