#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "LedControl.h"
#include <EEPROM.h> //0 distance, 1 time, 2 kcal, 3 fare

#define powerPin 13
#define leedPin 2
#define modePin 8

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);
LedControl lc=LedControl(12,10,11,4);

enum emotion_t {
  CUTE,
  SURPRISE,
  NORMAL,
  DANGER,
  SICK,
  HAPPY
};

enum lcd_mode {
  DIST,
  TAXI,
  KCAL,
  AVG,
  ACC //accumulated
};

const byte LCD_MODE_NUM = 5;

bool powerswitch;
bool prevpowerswitch;
bool powerstate;

enum lcd_mode lcd_M = DIST;

enum emotion_t emo = NORMAL;
enum emotion_t emo_n = NORMAL;

unsigned long emotionTime;
unsigned long prevEmoTime;
unsigned long shiftEmoTime;

int shift;

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

float r = 35;
float c = r*2*PI/100;

unsigned long startTime;
unsigned long prevTime;
unsigned long currTime;
unsigned long timeStamp;
unsigned long shiftInterval = 300;

float period;
float velocity;
float prevVelocity;

bool leed;
bool prevleed;

float distance;
float dist_acc;

float kcal;
float kcal_acc;

float remainD = 140;

long fare = 2800;
long fare_acc;

bool buttonState[4];
bool prevState[4];

int vibe_state;

const int v[15] = {13, 16, 19, 22, 24, 26, 27, 29, 31, 32, 34, 37};
const float t[15] = {0.065, 0.078, 0.094, 0.113, 0.124, 0.136, 0.149, 0163, 0.179, 0.196, 0.215, 0.259};

void reset()
{
  lcd.clear();
  emo = NORMAL;
  emo_n = NORMAL;
  fare = 2800;
  remainD = 140;
  distance = 0;
  startTime = millis();
  kcal = 0;
}

void EEPROMWriteLong(int addr, long val)
{
  byte a;
  for(int i = 0; i<4; i++){
    a = (val >> (i*8)) & 0xFF;
    EEPROM.write(addr*4+i, a);
  }
}

long EEPROMReadLong(long addr)
{
  long val = 0;
  long temp;
  for(int i = 0; i<4; i++){
    temp = 0x000000FF & EEPROM.read(addr*4+i);
    val |= (temp << (i*8));
  }
  return val;
}

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
   },
   {
    0x1800300,
    0x600c00,
    0x101000,
    0x600c00,
    0x1800300,
    0xe0000e0,
    0xaa000,
    0xe0540e0,
   }
};

const uint32_t normal[][8]={
  {
    0xe00e00,
    0x1301300,
    0x1f01f00,
    0x1f01f00,
    0xe00e00,
    0xe0000e0,
    0x92000,
    0xe06c0e0,
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

const uint32_t sick[]={
  /*
  0x0,
  0x1b01b00,
  0xe00e00,
  0x1b01b00,
  0x0,
  0xe0100e0,
  0x28000,
  0xe0440e0,
  */
  0x1101100,
  0xa00a00,
  0x400400,
  0xa00a00,
  0x1101100,
  0xe0100e0,
  0x28000,
  0xe0440e0,
};

const uint32_t happy[][8]={
  {
    0x400400,
    0xa00a00,
    0x1101100,
    0x0,
    0x0,
    0xe0000e0,
    0x44000,
    0xe0380e0
  },
  {
    0x400400,
    0xa00a00,
    0x1101100,
    0x0,
    0x0,
    0xe0380e0,
    0x28000,
    0xe0380e0,
  }
};

void display32bit(uint32_t image[8]){
  int i, j;
  uint32_t val;
  for(i=0;i<8;i++){
    val = image[i];
    if(shift < 0){
      val <<= shift*-1;
    } else {
      val >>= shift;
    }
    for(j=3; j>=0; j--){
      byte val_byte;
      val_byte = (byte)(val >> (8*j));
      lc.setRow(j, i, val_byte);
    }
  }
}

void displayEmotion()
{
  static int d_shift = 1;
   switch(emo){
    case NORMAL:
      displayNormal();
      emotionTime = 1000;
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
    case SICK:
      display32bit(sick);
      emotionTime = 2000;
      break;
    case HAPPY:
      displayHappy();
      emotionTime = 500;
   }
   if(prevVelocity >= 20 && velocity >= 20){
    emo_n = HAPPY;
   } else {
    emo_n = NORMAL;
   }
   shiftInterval = 300;
   shift+=d_shift;
   if(shift >= 1 || shift <= -1)
    d_shift *= -1;
}

void displayCute()
{
  static int i = 0;
  display32bit(cute[2]);
  i++;
  i %= 3;
}

void displayNormal()
{
  static int i = 0;
  display32bit(normal[0]);
  i++;
  i %= 4;
}

void displaySurprise()
{
  display32bit(surprise);
}

void displayDanger()
{
  display32bit(danger);
}

void displayHappy()
{
  static int i = 0;
  display32bit(happy[i]);
  i++;
  i %= 2;
}

int digit(int n)
{
  int r = 0;
  if(n == 0)
    return 1;
  while(n != 0){
    r++;
    n/=10;
  }
  return r;
}

void printVelocity()
{
  int d = digit(velocity)+7;
  lcd.setCursor(0,1);
  lcd.print("Speed:  ");
  lcd.setCursor(16-d,1);
  lcd.print(velocity);
  lcd.print("km/h");
}

void printDist()
{
  int d = digit(distance)+4;
  lcd.setCursor(0,0);
  lcd.print("Dist:      ");
  lcd.setCursor(16-d,0);
  lcd.print(distance);
  lcd.print("m");
}

void printFare()
{
  int d = digit((int)remainD)+1;
  lcd.setCursor(0,0);
  lcd.print("Remain:      ");
  lcd.setCursor(16-d,0);
  lcd.print((int)remainD);
  lcd.print("m");
  d = digit(fare)+1;
  lcd.setCursor(0,1);
  lcd.print("Fare:");
  lcd.setCursor(16-d,1);
  lcd.print(fare);
  lcd.write(0);
}

void printKcal()
{
  int d = digit(kcal)+7;
  lcd.setCursor(16-d,0);
  lcd.print(kcal);
  lcd.print("Kcal");
}

void printAvg()
{
  int d;
  int v_avg = distance/1000.0/((currTime-startTime)/1000.0/60.0/60.0);
  d = digit(v_avg)+4;
  lcd.setCursor(0,0);
  lcd.print("AVG Speed: ");
  lcd.setCursor(16-d,0);
  lcd.print(v_avg);
  lcd.print("Km/h");
};

void printAcc()
{
  int d = digit(fare_acc)+1;
  lcd.setCursor(0,0);
  lcd.print("fare acc: ");
  lcd.setCursor(16-d,0);
  lcd.print(fare_acc);
  lcd.write(0);

  d = digit((int)kcal_acc)+4;
  lcd.setCursor(0, 1);
  lcd.print("kcal: ");
  lcd.setCursor(16-d, 1);
  lcd.print((int)kcal_acc);
  lcd.print("kcal");
};

void blink()
{
  vibe_state++;
}

void setup()
{
  Serial.begin(9600);
  pinMode(3, INPUT);
  attachInterrupt(1, blink, FALLING);
  
  int devices=lc.getDeviceCount();
  for(int address=0;address<devices;address++)
    lc.shutdown(address, true);

  lcd.begin();
  lcd.noDisplay();
  lcd.noBacklight();
  lcd.createChar(0, won);
  startTime = millis();
}

void loop()
{
  currTime = millis();

  powerswitch = digitalRead(powerPin);
  if(prevpowerswitch == false && powerswitch == true){
    powerstate = !powerstate;
    if(powerstate == 0){
      lcd.noDisplay();
      lcd.noBacklight();
      int devices=lc.getDeviceCount();
      for(int address=0;address<devices;address++)
        lc.shutdown(address, true);
      EEPROMWriteLong(3, fare_acc);
      EEPROMWriteLong(2, (long)kcal_acc);
      EEPROMWriteLong(0, (long)dist_acc);
    } else { //turn on
      lcd.display();
      lcd.backlight();
      int devices=lc.getDeviceCount();
      for(int address=0;address<devices;address++) {
        lc.shutdown(address, false);
        lc.setIntensity(address,4);
        lc.clearDisplay(address);
      }
      reset();
      fare_acc = EEPROMReadLong(3)+2800;
      dist_acc = EEPROMReadLong(0);
      kcal_acc = EEPROMReadLong(2);
    }
  }
  
  if(powerstate == 0){
    prevpowerswitch = powerswitch;
    return;
  }
  
  leed = digitalRead(leedPin);
  if(currTime-timeStamp >= 1000){
    timeStamp = currTime;
    remainD -= 140/34.0;
    if(remainD < 0){
      remainD += 140;
      fare += 100;
      fare_acc+=100;
    }
    float addkcal = 0.05;
    for(int i = 0; i<12; i++){
      if(velocity > v[i]){
        addkcal = t[i];
        break;
      }
    }
    if(velocity == 0)
      addkcal = 0;
    addkcal *= 70/60.0;
    kcal += addkcal;
    kcal_acc += addkcal;
  }
  
  if(prevleed == 0 && leed == 1){
    prevVelocity = velocity;
    period = (currTime-prevTime)/1000.0;
    velocity = c/period*3.6;
    distance += c;
    dist_acc += c;
    remainD -= c;
    if(remainD < 0){
      remainD += 140;
      fare += 100;
      fare_acc+=100;
    }
    prevTime = currTime;
  }
  
  if(currTime-prevTime > 1000){
    if(prevVelocity > 10){
      emo_n = SICK;
      emoChange = true;
    }
    prevVelocity = velocity;
    velocity = 0;
  }
  
  if(vibe_state > 0){
    vibe_state /= 5;
    if(emo != DANGER && emo != SICK){
      emo_n = CUTE;
      emoChange = true;
      shiftInterval = 75;
    }
  }
  
  for(int i=0;i<3;i++){
    buttonState[i] = digitalRead(i+6);
    if(prevState[i] == false && buttonState[i] == true){
      switch(i){
        case 0:
          emo_n = SURPRISE;
          emoChange = true;
          break;
        case 1:
          emo_n = DANGER;
          emoChange = true;
          reset();
          break;
        case 2:
          lcd.clear();
          lcd_M=lcd_M+1;
          if(lcd_M >= LCD_MODE_NUM)
            lcd_M = lcd_M - LCD_MODE_NUM;
          break;
      }
    }
    prevState[i] = buttonState[i];
  }

  if(currTime - prevEmoTime >= emotionTime || emoChange){
    emoChange = false;
    prevEmoTime = currTime;
    emo = emo_n;
  }

  if(currTime - shiftEmoTime >= shiftInterval){
    shiftEmoTime = currTime;
    displayEmotion();
  }

  switch(lcd_M){
    case DIST:
      printDist();
      printVelocity();
      break;
    case TAXI:
      printFare();
      break;
    case KCAL:
      printKcal();
      break;
    case AVG:
      printAvg();
      break;
    case ACC:
      printAcc();
      break;
  }
  prevleed = leed;
  prevpowerswitch = powerswitch;
  delay(10);
}
