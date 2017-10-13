#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "LedControl.h"

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);
LedControl lc=LedControl(12,10,11,4);

enum emotion_t {
  CUTE,
  SURPRISE,
  NORMAL,
  DANGER
};

enum emotion_t emo = NORMAL;
enum emotion_t emo_n = NORMAL;

unsigned long emotionTime;
unsigned long prevEmoTime;

bool emoChange;

byte won[8] = {
  B01001,
  B10101,
  B01001,
  B00001,
  B11101,
  B01011,
  B00100,
  B00111
};

float r = 20;
float c = r*2*PI/100;

unsigned long prevTime;
unsigned long currTime;
unsigned long timeStamp;

float period;
float velocity;

bool leed;
bool prevleed;

float distance;

float remainD = 140;

int fare = 2800;

bool buttonState[4];
bool prevState[4];

bool state;

const uint32_t cute[][8]=
{
  {
    0x10000020,
    0x8000040,
    0x4000080,
    0x2000100,
    0x4000080,
    0x8000040,
    0x102AA820,
    0x555000
  },
  {
    0x8000010,
    0x4000020,
    0x2000040,
    0x10000c0,
    0x2000040,
    0x4000020,
    0x8155410,
    0x2aa800
   }
};

const uint32_t normal[][8]={
  {0x1e00f00,
  0x2101080,
  0x2101080,
  0x2101080,
  0x2101080,
  0x1e00f00,
  0x44000,
  0x38000
  },
  {
    0x3c00780,
    0x4200840,
    0x4200840,
    0x4200840,
    0x4244840,
    0x3c7c780,
    0x44000,
    0x7c000,
  }
};

const uint32_t surprise[]=
{
  0x3c00780,
  0x4200840,
  0x5a00b40,
  0x5a38b40,
  0x4244840,
  0x3c44780,
  0x44000,
  0x38000,
};

const uint32_t danger[]={
  0x3104220,
  0x491f220,
  0x4904220,
  0x310ae20,
  0x104220,
  0x7d01e20,
  0x1101200,
  0x1101e20,
};

void display32bit(uint32_t image[8]){
  int i, j;
  for(i=0;i<8;i++){
    for(j=3; j>=0; j--){
      byte val = (byte)(image[i] >> (8*j));
      lc.setRow(j, i, val);
    }
  }
}

void displayEmotion()
{
   switch(emo){
    case NORMAL:
      displayNormal();
      emotionTime = 500;
      break;
    case DANGER:
      displayDanger();
      emotionTime = 2000;
      break;
    case CUTE:
      displayCute();
      emotionTime = 50;
      break;
    case SURPRISE:
      displaySurprise();
      emotionTime = 1000;
      break;
   }
   emo_n = NORMAL;
}

void displayCute()
{
  static int i = 0;
  display32bit(cute[i]);
  i = 1-i;
}

void displayNormal()
{
  static int i = 0;
  display32bit(normal[i]);
  i = 1-i;
}

void displaySurprise()
{
  display32bit(surprise);
}

void displayDanger()
{
  display32bit(danger);
}

void printVelocity()
{
  lcd.setCursor(0,1);
  lcd.print(velocity);
  lcd.print("km/h   ");
}

void printDist()
{
  lcd.setCursor(0,0);
  lcd.print((int)remainD);
  //lcd.print(distance);
  lcd.print("m   ");
}

void printFare()
{
  lcd.setCursor(0,1);
  lcd.print(fare);
  lcd.write(0);
}

void blink()
{
  state++;
  
}

void setup()
{
  Serial.begin(9600);
  pinMode(3, INPUT);
  attachInterrupt(1, blink, FALLING);
  
  int devices=lc.getDeviceCount();
  for(int address=0;address<devices;address++) {
    /*The MAX72XX is in power-saving mode on startup*/
    lc.shutdown(address,false);
    /* Set the brightness to a medium values */
    lc.setIntensity(address,4);
    /* and clear the display */
    lc.clearDisplay(address);
  }
  
  lcd.begin();
  lcd.backlight();
  lcd.createChar(0, won);
}

void loop()
{
  int i;
  currTime = millis();
  
  if(state != 0){
    state = 0;
    emo_n = CUTE;
    emoChange = true;
  }
  
  for(int i=0;i<4;i++){
    buttonState[i] = digitalRead(i+5);
    if(prevState[i] == false && buttonState[i] == true){
      switch(i){
        case 0:
          //emo_n = SURPRISE;
          //emoChange = true;
          break;
        case 1:
          //emo_n = DANGER;
          //emoChange = true;
          break;
        case 2:
          //emo_n = CUTE;
          //emoChange = true;
          break;
        case 3:
          emo_n = NORMAL;
          break;
      }
    }
    prevState[i] = buttonState[i];
  }

  if(currTime - prevEmoTime >= emotionTime || emoChange){
    emoChange = false;
    prevEmoTime = currTime;
    emo = emo_n;
    displayEmotion();
  }

  leed = digitalRead(5);
  

  if(currTime-timeStamp >= 1000){
    timeStamp = currTime;
    remainD -= 140/34.0;
    if(remainD < 0){
      remainD += 140;
      fare += 100;
    }
  }
  
  if(prevleed == 0 && leed == 1){
    period = (currTime-prevTime)/1000.0;
    velocity = c/period*3.6;
    distance += c;
    remainD -= c;
    if(remainD < 0){
      remainD += 140;
      fare += 100;
    }
      
    prevTime = currTime;
  }

  if(currTime-prevTime > 2000){
    //lcd.clear();
    velocity = 0;
  }

  printDist();
  //printVelocity();
  printFare();
  
  prevleed = leed;
  delay(10);
}
