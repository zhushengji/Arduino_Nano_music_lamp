/*
 * 物料清单
 * 1.主控：Arduino Nano
 * 2.电位器*2
 * 3.按钮*1
 * 4.ws2812灯带*1
 * 4.max9814麦克风*1
 * 接线方式
 * 1.电位器两端引脚不分正反，分别接到5v，gnd，中间引脚一个接A1一个接A2
 * 2.按钮一个引脚接D3，一个接GND
 * 3.ws2812 +接5v，-接GND，DIN接D2
 * 4.max9814：
   ** GND-GND
   ** VDD-5V
   ** Gain-3v3
   ** OUT-A0
*/
#include <Adafruit_NeoPixel.h>
#include <FastLED.h>
#include <EEPROM.h>
#define PIN 2
#define N_PIXELS  31
#define BG 0
#define COLOR_ORDER GRB  
#define BRIGHTNESS 255   
#define LED_TYPE WS2812B
#define MIC_PIN   A0  
#define DC_OFFSET  0  
#define NOISE     280  
#define SAMPLES   60  
#define TOP       (N_PIXELS + 2) 
#define PEAK_FALL 20  // Rate of peak falling dot
#define N_PIXELS_HALF (N_PIXELS/2)
#define GRAVITY           -9.81              
#define h0                1                  
#define NUM_BALLS         3                  
#define SPEED .20       





float h[NUM_BALLS] ;                         
float vImpact0 = sqrt( -2 * GRAVITY * h0 );  
float vImpact[NUM_BALLS] ;
float tCycle[NUM_BALLS] ;
int   pos[NUM_BALLS] ;
long  tLast[NUM_BALLS] ;
float COR[NUM_BALLS] ;

float
  greenOffset = 30,
  blueOffset = 150;

byte
  peak      = 0,
  dotCount  = 0,
  volCount  = 0;
int
  vol[SAMPLES],
  lvl       = 10,
  minLvlAvg = 0,
  maxLvlAvg = 512;

int brightnessValue, prevBrightnessValue;
int sensorDeviationBrightness = 1;
int sensitivityValue = 128;
int maxSensitivity = 2 * 255;
int ledBrightness = 255;
int val;


Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_PIXELS, PIN, NEO_GRB + NEO_KHZ800);


uint8_t thisbeat =  23;
uint8_t thatbeat =  28;
uint8_t thisfade =   2;
uint8_t thissat = 255;
uint8_t thisbri = 255; 

//FOR JUGGLE
uint8_t numdots = 4;
uint8_t faderate = 2;
uint8_t hueinc = 16;
uint8_t thishue = 0;
uint8_t curhue = 0; 
uint8_t thisbright = 255;
uint8_t basebeat = 5; 
uint8_t max_bright = 255; 


float redStates[N_PIXELS];
float blueStates[N_PIXELS];
float greenStates[N_PIXELS];
float Fade = 0.96;


const uint32_t Red = strip.Color(255, 0, 0);
const uint32_t Yellow = strip.Color(255, 255, 0);
const uint32_t Green = strip.Color(0, 255, 0);
const uint32_t Blue = strip.Color(0, 0, 255);
const uint32_t White = strip.Color(255, 255, 255);
const uint32_t Dark = strip.Color(0, 0, 0);
unsigned int sample;

CRGB leds[N_PIXELS];

int          myhue =   0;


const int buttonPin = 3;     

// Variables will change:

int buttonPushCounter = 0;   
int buttonState = 0;         
int lastButtonState = 0;


int color;
int center = 0;
int step = -1;
int maxSteps = 8;
float fadeRate = 0.80;
int diff;
 

uint32_t currentBg = random(256);
uint32_t nextBg = currentBg;

void setup() {
  delay( 2000 );
  FastLED.addLeds<WS2812B, PIN, COLOR_ORDER>(leds, N_PIXELS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  BRIGHTNESS );
  memset(vol, 0, sizeof(vol));
  LEDS.addLeds<LED_TYPE, PIN, COLOR_ORDER>(leds, N_PIXELS); 
  strip.begin();
  strip.show(); 
  
  Serial.begin(115200);
  pinMode(buttonPin, INPUT);  
  pinMode(2, OUTPUT);
  pinMode(A0,INPUT); 
  pinMode(A1,INPUT); 
  pinMode(A2,INPUT); 
  //initialize the buttonPin as output
  digitalWrite(buttonPin, HIGH); 
  digitalWrite(2, HIGH); 

 for (int i = 0 ; i < NUM_BALLS ; i++) {    
    tLast[i] = millis();
    h[i] = h0;
    pos[i] = 0;                              
    vImpact[i] = vImpact0;                   
    tCycle[i] = 0;
    COR[i] = 0.90 - float(i)/pow(NUM_BALLS,2); 
 
  
  }
}
int n=0;
void loop() {
  n   = analogRead(MIC_PIN);
  brightnessValue = analogRead(A2);
//  Serial.print("亮度：");
//  Serial.print(brightnessValue);
//  Serial.print("声音：");
//  Serial.println(n);
  
  brightnessValue = map(brightnessValue, 0, 1023, 0, 255);
  
  if (abs(brightnessValue - prevBrightnessValue) > sensorDeviationBrightness) {
    ledBrightness = brightnessValue;
    strip.setBrightness(ledBrightness);
    prevBrightnessValue = brightnessValue;
  }
  
  
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int  height;
  buttonPushCounter=EEPROM.read(0);
  buttonState = digitalRead(buttonPin);
   
  if (buttonState != lastButtonState) {
   
    if (buttonState == HIGH) {
      buttonPushCounter++;
      Serial.println("on");
      Serial.print("number of button pushes:  ");
      Serial.println(buttonPushCounter);
      if(buttonPushCounter>=14) {
      buttonPushCounter=1;}
      EEPROM.write(0,buttonPushCounter);
      
    } 
    else {
      
      Serial.println("off"); 
    }
  }
 
  
  lastButtonState = buttonState;

  


switch (buttonPushCounter){
  
           
  case 1:
     buttonPushCounter==1; {
      vu(); 
     break;}
       
  case 2:
     buttonPushCounter==2; {
       vu2(); 
      break;}
      
   case 3:
     buttonPushCounter==3; {
    Vu3(); 
      break;}
         
    case 4:
     buttonPushCounter==4; {
    Vu4(); // 
      break;}  
      
       case 5:
     buttonPushCounter==5; {
      rainbow(150);
       break;}
       
        case 6:
     buttonPushCounter==6; {
      rainbow(20);
       break;}
       
         case 7:
     buttonPushCounter==7; {
      ripple();
       break;}
       
           case 8:
     buttonPushCounter==8; {
      ripple2();
       break;}
       
              case 9:
     buttonPushCounter==9; {
      Twinkle();
       break;}
      
         case 10:
     buttonPushCounter==10; {
      pattern2();
       break;}
       
           case 11:
     buttonPushCounter==11; {
      pattern3();
       break;}
       
           case 12:
     buttonPushCounter==12; {
    Balls(); // 
      break;}    
       
        case 13:
     buttonPushCounter==13; {
    colorWipe(strip.Color(0, 0, 0), 10); // A Black
      break;}
      
    
   
       
   }

  
   
}

void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      if (digitalRead(buttonPin) != lastButtonState)  
       return;        
      delay(wait);
  }
}
void Vu4() {
    uint8_t  i;
  uint16_t minLvl, maxLvl;
  int  height;

  
  val = (analogRead(A1));  
  val= map(val, 0, 1023, -10, 6);
  
//  n   = analogRead(MIC_PIN);                        
  n   = abs(n - 0 - DC_OFFSET); 
  n   = (n <= NOISE) ? 0 : (n - NOISE);            
  
    if(val<0){
        n=n/(val*(-1));
        }
       if(val>0){
        n=n*val;
        }
  
  lvl = ((lvl * 7) + n) >> 3;   
 
  
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);
 
  if(height < 0L)       height = 0;      
  else if(height > TOP) height = TOP;
  if(height > peak)     peak   = height; 
  greenOffset += SPEED;
  blueOffset += SPEED;
  if (greenOffset >= 255) greenOffset = 0;
  if (blueOffset >= 255) blueOffset = 0;
 
  
  for(i=0; i<N_PIXELS_HALF; i++) {
    if(i >= height) {              
      strip.setPixelColor(N_PIXELS_HALF-i-1,   0,   0, 0);
      strip.setPixelColor(N_PIXELS_HALF+i,   0,   0, 0);
    }
    else {
      uint32_t color = Wheel(map(i,0,N_PIXELS_HALF-1,(int)greenOffset, (int)blueOffset));
      strip.setPixelColor(N_PIXELS_HALF-i-1,color);
      strip.setPixelColor(N_PIXELS_HALF+i,color);
    }
    
  }
 
  // Draw peak dot  
  if(peak > 0 && peak <= N_PIXELS_HALF-1) {
    uint32_t color = Wheel(map(peak,0,N_PIXELS_HALF-1,30,150));
    strip.setPixelColor(N_PIXELS_HALF-peak-1,color);
    strip.setPixelColor(N_PIXELS_HALF+peak,color);
  }
  
   strip.show(); 
 
 
    if(++dotCount >= PEAK_FALL) { 
      
      if(peak > 0) peak--;
      dotCount = 0;
    }
 
 
  vol[volCount] = n;                     
  if(++volCount >= SAMPLES) volCount = 0; 
 
  minLvl = maxLvl = vol[0];
  for(i=1; i<SAMPLES; i++) {
    if(vol[i] < minLvl)      minLvl = vol[i];
    else if(vol[i] > maxLvl) maxLvl = vol[i];
  }
  
  if((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
 
}


void Vu3() {
  uint8_t i;
  uint16_t minLvl, maxLvl;
  int n, height;

  
  val = (analogRead(A1));  
  val= map(val, 0, 1023, -10, 6);

  n = analogRead(MIC_PIN);             
  n = abs(n - 0 - DC_OFFSET);        
  n = (n <= NOISE) ? 0 : (n - NOISE);  

      if(val<0){
        n=n/(val*(-1));
        }
       if(val>0){
        n=n*val;
        }
        
  lvl = ((lvl * 7) + n) >> 3;    

  
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if (height < 0L)       height = 0;      
  else if (height > TOP) height = TOP;
  if (height > peak)     peak   = height; 

  greenOffset += SPEED;
  blueOffset += SPEED;
  if (greenOffset >= 255) greenOffset = 0;
  if (blueOffset >= 255) blueOffset = 0;

  
  for (i = 0; i < N_PIXELS; i++) {
    if (i >= height) {
      strip.setPixelColor(i, 0, 0, 0);
    } else {
      strip.setPixelColor(i, Wheel(
        map(i, 0, strip.numPixels() - 1, (int)greenOffset, (int)blueOffset)
      ));
    }
  }
  
  if(peak > 0 && peak <= N_PIXELS-1) strip.setPixelColor(peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
  
   strip.show(); // Update strip
 
 
    if(++dotCount >= PEAK_FALL) { 
      
      if(peak > 0) peak--;
      dotCount = 0;
    }
  strip.show();  

  vol[volCount] = n;
  if (++volCount >= SAMPLES) {
    volCount = 0;
  }

  
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++) {
    if (vol[i] < minLvl) {
      minLvl = vol[i];
    } else if (vol[i] > maxLvl) {
      maxLvl = vol[i];
    }
  }

  
  if ((maxLvl - minLvl) < TOP) {
    maxLvl = minLvl + TOP;
  }
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}


void Balls() {
  for (int i = 0 ; i < NUM_BALLS ; i++) {
    tCycle[i] =  millis() - tLast[i] ;     

    h[i] = 0.5 * GRAVITY * pow( tCycle[i]/1000 , 2.0 ) + vImpact[i] * tCycle[i]/1000;

    if ( h[i] < 0 ) {                      
      h[i] = 0;                            
      vImpact[i] = COR[i] * vImpact[i] ;   
      tLast[i] = millis();

      if ( vImpact[i] < 0.01 ) vImpact[i] = vImpact0;  
    }
    pos[i] = round( h[i] * (N_PIXELS - 1) / h0);       
  }

  //Choose color of LEDs, then the "pos" LED on
  for (int i = 0 ; i < NUM_BALLS ; i++) leds[pos[i]] = CHSV( uint8_t (i * 40) , 255, 255);
  FastLED.show();
  //Then off for the next loop around
  for (int i = 0 ; i < NUM_BALLS ; i++) {
    leds[pos[i]] = CRGB::Black;
  }
}



void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
     if (digitalRead(buttonPin) != lastButtonState)  
       return;         
      delay(wait);
        }
    }


void vu() {
 
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int  height;

  val = (analogRead(A1));  
  val= map(val, 0, 1023, -10, 6);
  n   = abs(n - 0 - DC_OFFSET); 
  n   = (n <= NOISE) ? 0 : (n - NOISE);            
     if(val<0){
        n=n/(val*(-1));
        }
       if(val>0){
        n=n*val;
        }
  lvl = ((lvl * 7) + n) >> 3;   
 
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);
 
  if(height < 0L)       height = 0;      
  else if(height > TOP) height = TOP;
  if(height > peak)     peak   = height; 
 
 
  
  for(i=0; i<N_PIXELS; i++) {
    if(i >= height)               strip.setPixelColor(i,   0,   0, 0);
    else strip.setPixelColor(i,Wheel(map(i,0,strip.numPixels()-1,30,150)));
    
  }
 
 
   
  if(peak > 0 && peak <= N_PIXELS-1) strip.setPixelColor(peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
  
   strip.show(); 
 

 
    if(++dotCount >= PEAK_FALL) { 
      
      if(peak > 0) peak--;
      dotCount = 0;
    }
 
 
 
  vol[volCount] = n;                      
  if(++volCount >= SAMPLES) volCount = 0; 
 
  
  minLvl = maxLvl = vol[0];
  for(i=1; i<SAMPLES; i++) {
    if(vol[i] < minLvl)      minLvl = vol[i];
    else if(vol[i] > maxLvl) maxLvl = vol[i];
  }
  
  if((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
 
}
 

uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}


void vu2() {
  
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int  height;
 
  val = (analogRead(A1));  
  val= map(val, 0, 1023, -10, 6);

  n   = abs(n - 0 - DC_OFFSET); 
  n   = (n <= NOISE) ? 0 : (n - NOISE);             

      if(val<0){
        n=n/(val*(-1));
        }
       if(val>0){
        n=n*val;
        }
  lvl = ((lvl * 7) + n) >> 3;    
 
  
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);
 
  if(height < 0L)       height = 0;      // Clip output
  else if(height > TOP) height = TOP;
  if(height > peak)     peak   = height; // Keep 'peak' dot at top
 
 
  
  for(i=0; i<N_PIXELS_HALF; i++) {
    if(i >= height) {              
      strip.setPixelColor(N_PIXELS_HALF-i-1,   0,   0, 0);
      strip.setPixelColor(N_PIXELS_HALF+i,   0,   0, 0);
    }
    else {
      uint32_t color = Wheel(map(i,0,N_PIXELS_HALF-1,30,150));
      strip.setPixelColor(N_PIXELS_HALF-i-1,color);
      strip.setPixelColor(N_PIXELS_HALF+i,color);
    }
    
  }
 
 
 
   
  if(peak > 0 && peak <= N_PIXELS_HALF-1) {
    uint32_t color = Wheel(map(peak,0,N_PIXELS_HALF-1,30,150));
    strip.setPixelColor(N_PIXELS_HALF-peak-1,color);
    strip.setPixelColor(N_PIXELS_HALF+peak,color);
  }
  
   strip.show(); 
 

 
    if(++dotCount >= PEAK_FALL) { //fall rate 
      
      if(peak > 0) peak--;
      dotCount = 0;
    }
 
 
 
  vol[volCount] = n;                      
  if(++volCount >= SAMPLES) volCount = 0; 
 
  minLvl = maxLvl = vol[0];
  for(i=1; i<SAMPLES; i++) {
    if(vol[i] < minLvl)      minLvl = vol[i];
    else if(vol[i] > maxLvl) maxLvl = vol[i];
  }
  
  if((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
 
}


 void ripple() {
 
    if (currentBg == nextBg) {
      nextBg = random(256);
    }
    else if (nextBg > currentBg) {
      currentBg++;
    } else {
      currentBg--;
    }
    for(uint16_t l = 0; l < N_PIXELS; l++) {
      leds[l] = CHSV(currentBg, 255, 50);         
    }
 
  if (step == -1) {
    center = random(N_PIXELS);
    color = random(256);
    step = 0;
  }
 
  if (step == 0) {
    leds[center] = CHSV(color, 255, 255);         
    step ++;
  }
  else {
    if (step < maxSteps) {
      Serial.println(pow(fadeRate,step));
 
      leds[wrap(center + step)] = CHSV(color, 255, pow(fadeRate, step)*255);       
      leds[wrap(center - step)] = CHSV(color, 255, pow(fadeRate, step)*255);       
      if (step > 3) {
        leds[wrap(center + step - 3)] = CHSV(color, 255, pow(fadeRate, step - 2)*255);     //   strip.setPixelColor(wrap(center + step - 3), Wheel(color, pow(fadeRate, step - 2)));
        leds[wrap(center - step + 3)] = CHSV(color, 255, pow(fadeRate, step - 2)*255);     //   strip.setPixelColor(wrap(center - step + 3), Wheel(color, pow(fadeRate, step - 2)));
      }
      step ++;
    }
    else {
      step = -1;
    }
  }
 
  LEDS.show();
  delay(50);
}
 
 
int wrap(int step) {
  if(step < 0) return N_PIXELS + step;
  if(step > N_PIXELS - 1) return step - N_PIXELS;
  return step;
}
 
 
void one_color_allHSV(int ahue, int abright) {                
  for (int i = 0 ; i < N_PIXELS; i++ ) {
    leds[i] = CHSV(ahue, 255, abright);
  }
}

 
void ripple2() {
  if (BG){
    if (currentBg == nextBg) {
      nextBg = random(256);
    } 
    else if (nextBg > currentBg) {
      currentBg++;
    } else {
      currentBg--;
    }
    for(uint16_t l = 0; l < N_PIXELS; l++) {
      strip.setPixelColor(l, Wheel(currentBg, 0.1));
    }
  } else {
    for(uint16_t l = 0; l < N_PIXELS; l++) {
      strip.setPixelColor(l, 0, 0, 0);
    }
  }
 
 
  if (step == -1) {
    center = random(N_PIXELS);
    color = random(256);
    step = 0;
  }
 
 
 
  if (step == 0) {
    strip.setPixelColor(center, Wheel(color, 1));
    step ++;
  } 
  else {
    if (step < maxSteps) {
      strip.setPixelColor(wrap(center + step), Wheel(color, pow(fadeRate, step)));
      strip.setPixelColor(wrap(center - step), Wheel(color, pow(fadeRate, step)));
      if (step > 3) {
        strip.setPixelColor(wrap(center + step - 3), Wheel(color, pow(fadeRate, step - 2)));
        strip.setPixelColor(wrap(center - step + 3), Wheel(color, pow(fadeRate, step - 2)));
      }
      step ++;
    } 
    else {
      step = -1;
    }
  }
  
  strip.show();
  delay(50);
}

 
 

uint32_t Wheel(byte WheelPos, float opacity) {
  
  if(WheelPos < 85) {
    return strip.Color((WheelPos * 3) * opacity, (255 - WheelPos * 3) * opacity, 0);
  } 
  else if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color((255 - WheelPos * 3) * opacity, 0, (WheelPos * 3) * opacity);
  } 
  else {
    WheelPos -= 170;
    return strip.Color(0, (WheelPos * 3) * opacity, (255 - WheelPos * 3) * opacity);
  }
}


   void pattern2() {
      
       sinelon();                          
  show_at_max_brightness_for_power();                       
} 


void sinelon() {
  fadeToBlackBy( leds, N_PIXELS, thisfade);
  int pos1 = beatsin16(thisbeat,0,N_PIXELS);
  int pos2 = beatsin16(thatbeat,0,N_PIXELS);
    leds[(pos1+pos2)/2] += CHSV( myhue++/64, thissat, thisbri);
}
    void pattern3() {
       ChangeMe();
  juggle();
  show_at_max_brightness_for_power();                         
} 


void juggle() {                                               
  curhue = thishue;                                          
  fadeToBlackBy(leds, N_PIXELS, faderate);
  for( int i = 0; i < numdots; i++) {
    leds[beatsin16(basebeat+i+numdots,0,N_PIXELS)] += CHSV(curhue, thissat, thisbright);   
    curhue += hueinc;
  }
} 


void ChangeMe() {                                             
  uint8_t secondHand = (millis() / 1000) % 30;                
  static uint8_t lastSecond = 99;                             
  if (lastSecond != secondHand) {                             
    lastSecond = secondHand;
    if (secondHand ==  0)  {numdots=1; faderate=2;}  
    if (secondHand == 10)  {numdots=4; thishue=128; faderate=8;}
    if (secondHand == 20)  {hueinc=48; thishue=random8();}                               // Only gets called once, and not continuously for the next several seconds. Therefore, no rainbows.
  }
}

void Twinkle () {
   if (random(25) == 1) {
      uint16_t i = random(N_PIXELS);
      if (redStates[i] < 1 && greenStates[i] < 1 && blueStates[i] < 1) {
        redStates[i] = random(256);
        greenStates[i] = random(256);
        blueStates[i] = random(256);
      }
    }
    
    for(uint16_t l = 0; l < N_PIXELS; l++) {
      if (redStates[l] > 1 || greenStates[l] > 1 || blueStates[l] > 1) {
        strip.setPixelColor(l, redStates[l], greenStates[l], blueStates[l]);
        
        if (redStates[l] > 1) {
          redStates[l] = redStates[l] * Fade;
        } else {
          redStates[l] = 0;
        }
        
        if (greenStates[l] > 1) {
          greenStates[l] = greenStates[l] * Fade;
        } else {
          greenStates[l] = 0;
        }
        
        if (blueStates[l] > 1) {
          blueStates[l] = blueStates[l] * Fade;
        } else {
          blueStates[l] = 0;
        }
        
      } else {
        strip.setPixelColor(l, 0, 0, 0);
      }
    }
    strip.show();
     delay(10);
  
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    
    if (digitalRead(buttonPin) != lastButtonState)  
       return;        
    delay(wait);
  }
}
