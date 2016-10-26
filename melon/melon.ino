/* @file skhydro21.ino
|| @version 2.0
|| @author Kongdej Srisamran
|| @contact kongdejs@gmail.com
||
|| @description
|| | Hydroponics Controller System
|| | Hardware: Mega2560 + keypad4x4 + LCDI2C4x20 + RTCDS3231 + EC Diy
|| #
*/
#include <Arduino.h>
#include <Wire.h> 
#include "RTClib.h"
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Time.h"
#include "TimeAlarms.h"
#include <avr/eeprom.h>
#include <Math.h>

#define noRELAY    8  // number of relays
#define noSEQ     36    // number of schedule : watering AB=12, watering WTR=12, mixAB=12

#define PUMPMAIN   0  // define name of relay
#define PUMPA      1
#define PUMPB      2
#define VALVEAB    3
#define VALVEREC   4
#define VALVEWTR   5
#define VALVEMAIN  6

#define LEVEL_HH   3  // digital input pin for level switch

int relayNo[noRELAY] = {23,25,27,29,31,33,35,37};    // select the pins used on the Relays 
char *relay_name[9] = {
  "1>Pump MAIN      ",
  "2>Pump A         ",
  "3>Pump B         ",
  "4>Valve AB       ",
  "5>Valve REC      ",
  "6>Valve WTR      ",
  "7>Valve MAIN     ",
  "8>Spare..        ",
  "9>Pump A&B       "
};

//=== EEPROM Struct ===//
struct sequence_t {
   int h;   // hour
   int m;   // minute
   int f;   // function: watering AB, watering WTR, mix AB
};

struct water_t {
  unsigned long watering_time=60;
};

struct ab_t {
  unsigned long fill_time;          // limit time filling water to tank
  unsigned long dosing_time;        // dosing time for ab pump
  unsigned long dosing_wait_time;   // waiting time for get EC
  float        plant_consump;       // plant consump water lit/day
  float        drip_rate;           // dripping rate lit/hour
  int          prepare_time;        // recirculation time
  float        ec_sp;               // setpoint for EC  
  float        ec_ll;               // low limit EC to auto start A&B pump 
  unsigned int ec_enabled;          // EC mode auto or manual
  float pump_a_ratio;               // calibration pump A ratio
  float pump_b_ratio;               // calibration pump B ratio
};

struct settings_t {
  sequence_t sequence[noSEQ];
  water_t wtr;
  ab_t  ab;
  float K;
} settings;
// END EEPROM struct =====//

//** Parameters for EC sensor **//
float K=1.66;    // default K for calibreation
int R1= 655;     // resistance 
float Ra=0.5;    //Resistance of powering Pins
int ECPin= A4;
int ECGround=A1;
int ECPower =A0;
int ppm =0;
float PPMconversion=0.7;
float TemperatureCoef = 0.019; //this changes depending on what chemical we are measuring
float Temperature=10;
float EC=0;
float EC25 =0;
float raw= 0;
float Vin= 5;
float Vdrop= 0;
float Rc= 0;
float buffer=0;

// parameters for DS3231
#define ONE_WIRE_BUS 4              // Data wire For Temp Probe is plugged into pin 10 on the Arduino
const int TempProbePossitive =5;    //Temp Probe power connected to pin 9
const int TempProbeNegative=2;      //Temp Probe Negative connected to pin 8
OneWire oneWire(ONE_WIRE_BUS);      // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire);// Pass our oneWire reference to Dallas Temperature.

//** EC Calibration **//
float CalibrationEC=2.5;   // default EC for calibration
float TemperatureStart=0;
float TemperatureFinish=0;
//**//

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
   #define Serial SerialUSB
#endif

// RTC Clock
RTC_DS3231 rtc;   

// define keypad
LiquidCrystal_I2C lcd(0x27, 20, 4);
const byte ROWS = 4; //four rows
const byte COLS = 4; //three columns
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'.','0','#','D'}
};
//byte rowPins[ROWS] = {53, 51, 49, 47}; //connect to the row pinouts of the keypad
//byte colPins[COLS] = {45, 43, 41, 39}; //connect to the column pinouts of the keypad
byte rowPins[ROWS] = {13, 12, 11, 10}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {9, 8, 7, 6}; //connect to the column pinouts of the keypad
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

// menus text
char *menus[8]={"A>EC Set            ",
                "B>Watering Cycle    ",
                "C>EC Calibration    ",
                "D>Test            #>",
                "A>Clock Adjust      ",
                "B>Settings",
                "C>Reset",
                "0>Exit            <*"};

const long interval = 1000;   // refresh clock
const int RTCTimesync = 3600; // sync RTC every 1 hour

// logo character
//byte logo[8] = {0b01010,0b00000,0b00000,0b10101,0b10101,0b00100,0b00100,0b11111}; // splinker
byte logo[8] = {0b11111,0b10011,0b10101,0b11001,0b11001,0b10101,0b10011,0b11111};   // clock

float EC_sp;
float EC_lp;
int EC_enable;
char *yes_no[2]={"No ","Yes"};
int watering_ab_round=0;
int drip_time=0;

void(* resetFunc)(void)=0;
unsigned long prvTickDosing=0;
unsigned long prvTickDosingWait=0;
unsigned long previousMillis = 0;  

//************************************************************************************//
// ***==== SETUP ===================================================================**//  
//************************************************************************************//
void setup(){
  Serial.begin(9600);
  lcd.begin();
  lcd.backlight();
  lcd.createChar(1, logo);  
  if (! rtc.begin()) {
    lcd.print("Couldn't find RTC"); //Serial.println("Couldn't find RTC");
    while (1);
  }
  if (rtc.lostPower()) {
    //Serial.println("RTC lost power, lets set the time!");
    lcd.print("RTC lost power");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); 
    //  adjust = rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }    
    
  lcdPrint(0,0," Welcome to SKhydro ");
  lcdPrint(0,1,"    version 2.0     ");
  lcdPrint(0,2,"                    ");
  lcdPrint(0,3,"   by S.Kongdej     ");
  delay(3000);
  
  // Sync RTC time
  SyncTime();
  Alarm.timerRepeat(RTCTimesync,SyncTime);
  
  // load data from EEPROM
  eeprom_read_block((void*)&settings, (void*)0, sizeof(settings));   // read eeprom 
  EC_sp=settings.ab.ec_sp;
  EC_lp=settings.ab.ec_ll;
  EC_enable= settings.ab.ec_enabled;
  K=settings.K;
  
  // set Alarm 
  for (int i=0; i < noSEQ; i++) {
    if (settings.sequence[i].h != -1 && settings.sequence[i].m != -1){
        Alarm.alarmRepeat(settings.sequence[i].h,settings.sequence[i].m,0, doFunction);
        Serial.print(settings.sequence[i].h);Serial.print(":");Serial.print(settings.sequence[i].m);Serial.print(" >> ");Serial.println(settings.sequence[i].f);
        if (settings.sequence[i].f == 0) watering_ab_round++;
    }
  }
  
//  Serial.print("Watering AB = ");Serial.println(watering_ab_round);
  // set GPIO mode
  
  for (int i=0; i<noRELAY; i++) { // set up the GPIO Mode:
    pinMode(relayNo[i], OUTPUT);
    digitalWrite(relayNo[i],LOW);
  }
  
  //** set EC Setup
  pinMode(TempProbeNegative , OUTPUT );   //seting ground pin as output for tmp probe
  digitalWrite(TempProbeNegative , LOW ); //Seting it to ground so it can sink current
  pinMode(TempProbePossitive , OUTPUT );  //ditto but for positive
  digitalWrite(TempProbePossitive , HIGH );
  pinMode(ECPin,INPUT);
  pinMode(ECPower,OUTPUT);                //Setting pin for sourcing current
  pinMode(ECGround,OUTPUT);               //setting pin for sinking current
  digitalWrite(ECGround,LOW);             //We can leave the ground connected permanantly
  delay(100);                             // gives sensor time to settle
  sensors.begin();
  delay(100);
}

//************************************************************************************//
// ***==== LOOP ====================================================================**//  
//************************************************************************************//
void loop(){  
  lcd.home();lcd.write(1);      //show logo
  printTime();                  // show clock
  GetEC();                      // Calls Code to Go into GetEC() 
  PrintReadings();              // Cals Print routine [below main loop]
  menu();                       // getkey '#' show menu
  if (settings.ab.ec_enabled) { // if EC dosing is AUTO mode
    dosing();   // do dosing 
  }
  printStatus();                // print status of level switch and relays
  Alarm.delay(10);              // wait one second between clock display  

// *for testing * char key = keypad.getKey(); if (key=='A') {    mixAB();  }
}
//**==== END =========================================================================**//
//************************************************************************************//

//***===Function: print time ==========================// 
void printTime() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;    // save the last time you blinked the LED 
    DateTime now = rtc.now();
    lcd.setCursor(1,0);if (now.hour()<10) lcd.print('0');lcd.print(now.hour());
    lcd.setCursor(3,0);  lcd.print(":");if (now.minute()<10) lcd.print('0');lcd.print(now.minute());
    lcd.setCursor(6,0);  lcd.print(":");if (now.second()<10) lcd.print('0');lcd.print(now.second());
  }
}

//***==Funtion: This Loop Is called From Main Loop- Prints to serial usefull info ***//
void PrintReadings(){
  char *level[2] = {"L","H"};
  
  lcd.setCursor(12,0);lcd.print(Temperature);lcd.print((char)223);lcd.print("C"); 
  lcdPrint(0,1,"EC        = ");lcd.print(EC25);lcd.print(" mS");
  lcdPrint(0,2,"Set Point = ");lcd.print(EC_sp);lcd.print(" mS");
  int hh = digitalRead(LEVEL_HH);
  lcd.setCursor(0,3);lcd.print(level[hh]);
};

//***==Function: Show status of level switch and relays ==//
void printStatus() {
  int i;
  for (i=0;i<noRELAY;i++) {
    if (digitalRead(relayNo[i])) {
      lcd.setCursor(i+2,3);lcd.print("*");
    }
    else {
      lcd.setCursor(i+2,3);lcd.print("_");
    }
  }
}

//***==Fuction: check schedule and do functions ============================//
void doFunction() {
  int i,h,m,s;
  h=hour();
  m=minute();
//  Serial.println("dofunction");

  for (i=0; i < noSEQ; i++) {
    if (h == settings.sequence[i].h && m == settings.sequence[i].m) {
      int n = settings.sequence[i].f;
      switch (n) {
        case 0: 
                wateringAB();
                break;
        case 1: 
                wateringWTR();                
                break;
        case 2: 
                mixAB();
                break;
      }
    }
  }
}

//***==Function: mixing A&B =================================//
void mixAB() {
  int t;
  int hh=0;
  char *level[2] = {"L","H"};

  Serial.println("Mix AB");

  if (Temperature < 0 || EC25 < 0) {
    lcd.clear();
    lcdPrint(0,0,"Err-Reading EC");
    delay(3000);
    return;
  }
 
  lcd.clear();
  lcdPrint(0,0,">Rec Valve Open ");digitalWrite(relayNo[VALVEREC],HIGH);
  delay(3000);
  lcdPrint(0,1,">Main Pump Start");digitalWrite(relayNo[PUMPMAIN],HIGH);
  
  hh=digitalRead(LEVEL_HH);
  if (hh == 0) {
    lcdPrint(0,2,">Main Valve Open");digitalWrite(relayNo[VALVEMAIN],HIGH);  
    t = settings.ab.fill_time;
    while(t > 0) {
      GetEC(); 
      lcdPrint(0,3,">");lcd.print(EC25);lcd.print("=>");lcd.print(settings.ab.ec_sp);
      if (EC25 <= settings.ab.ec_sp) {
        dosing();
      }
      lcdPrint(15,2,"     ");
      lcdPrint(15,2," ");lcd.print(t);
      delay(1000);
      t--;
      hh=digitalRead(LEVEL_HH);
      if (hh == 1) break;
    }  
    lcdPrint(0,2,">WTR Valve Close    ");digitalWrite(relayNo[VALVEMAIN],LOW);
  }
  
  while (EC25 <= settings.ab.ec_sp) {
     GetEC(); 
     lcdPrint(0,3,">");lcd.print(EC25);lcd.print("=>");lcd.print(settings.ab.ec_sp);
     dosing();
  }
  
  lcd.clear();
  lcdPrint(3,1,"--Finished--");
  digitalWrite(relayNo[PUMPMAIN],LOW);
  delay(3000);
  digitalWrite(relayNo[VALVEREC],LOW);
  lcd.clear();
  resetFunc();
}

//***==Fuction: dosing step for AB pump =================================//
void dosing() {
  unsigned long curTickDosing;
  unsigned long curTickDosingWait;
  unsigned long cnt;
  float dt;
  
  curTickDosing     = millis();
  curTickDosingWait = millis();

  if (EC25 <= settings.ab.ec_ll && digitalRead(relayNo[PUMPA]) == 0 && digitalRead(relayNo[PUMPB]) == 0) {
       if ((curTickDosingWait-prvTickDosingWait) >= settings.ab.dosing_wait_time*1000) {
          digitalWrite(relayNo[PUMPA],HIGH);
          digitalWrite(relayNo[PUMPB],HIGH);
          prvTickDosingWait=millis();
          prvTickDosing=millis();
       }
   }
    
  if (EC25 >= settings.ab.ec_sp) {
      digitalWrite(relayNo[PUMPA],LOW);
      digitalWrite(relayNo[PUMPB],LOW);    
      prvTickDosingWait=millis();
      prvTickDosing=millis(); 
      lcd.setCursor(15,3);lcd.print("     ");   
  }
  
  if (digitalRead(relayNo[PUMPA]) || digitalRead(relayNo[PUMPB])) {
    cnt = (settings.ab.dosing_time*1000 - (curTickDosing-prvTickDosing))/1000;
    if (cnt <= settings.ab.dosing_time) {
      lcd.setCursor(15,3);lcd.print("    ");    
      lcd.setCursor(15,3);lcd.print(cnt);    
    }
    if ((curTickDosing-prvTickDosing) >= settings.ab.dosing_time*1000) {
      
      if (settings.ab.pump_b_ratio >= settings.ab.pump_a_ratio) {
        digitalWrite(relayNo[PUMPB],LOW);
        dt = (settings.ab.pump_b_ratio - settings.ab.pump_a_ratio)/settings.ab.pump_a_ratio*settings.ab.dosing_time;
        delay(round(dt*1000));
        digitalWrite(relayNo[PUMPA],LOW);
      }
      else {
        digitalWrite(relayNo[PUMPA],LOW);
        dt = (settings.ab.pump_a_ratio - settings.ab.pump_b_ratio)/settings.ab.pump_b_ratio*settings.ab.dosing_time;
        delay(round(dt*1000));
        digitalWrite(relayNo[PUMPB],LOW);      
      }
//      Serial.println(settings.ab.pump_a_ratio);
//      Serial.println(settings.ab.pump_b_ratio);
//      Serial.println(settings.ab.dosing_time);
//      Serial.println(dt);
//      Serial.println(round(dt*1000));
      
      prvTickDosing=millis();        
      prvTickDosingWait=millis();        
    }
  }
  else {
    lcd.setCursor(15,3);lcd.print("     ");
  }
}

//***==Function: watering water only open water valve =========//
void wateringWTR() {
  int i;
  char buff[20];
  
  Serial.println("Fill Water");
  lcd.clear();
  lcdPrint(0,0,"-MAIN Vlv Open");
  digitalWrite(relayNo[VALVEMAIN],HIGH);
  for(i=settings.wtr.watering_time;i>0;i--){
      sprintf(buff,"%3d",i);
      lcdPrint(16,0,buff);
      delay(1000);
      if (digitalRead(LEVEL_HH)){
        lcdPrint(0,1,"-MIAN Vlv Close");
        digitalWrite(relayNo[VALVEMAIN],LOW);
        lcd.clear();
        return;        
      }
  }
  lcdPrint(0,1,"-MIAN Vlv Close");
  digitalWrite(relayNo[VALVEMAIN],LOW);
  lcd.clear();
}

//***==Function: watering from AB tank =====================//
void wateringAB() {
  int i;
  char buff[20];
  
  Serial.println("wateringAB");
  lcd.clear();
//  lcdPrint(0,0,"-Recir Valve Open   ");
//  digitalWrite(relayNo[VALVEREC],HIGH);
  delay(1000);
  lcdPrint(0,0,"-Recir Pump On      ");
  digitalWrite(relayNo[PUMPMAIN],HIGH);  
  for(i=settings.ab.prepare_time;i>0;i--){
      sprintf(buff,"%3d",i);
      lcdPrint(16,0,buff);
      delay(1000);
  }
  lcdPrint(0,0,"-Recir Valve Close  ");
  lcdPrint(0,1,"-Start Wtr AB ");
  digitalWrite(relayNo[VALVEAB],HIGH);
  delay(3000);
  digitalWrite(relayNo[VALVEREC],LOW);
  drip_time =round((settings.ab.plant_consump/settings.ab.drip_rate)*3600/watering_ab_round);
  for(i=drip_time;i>0;i--){
      sprintf(buff,"%3d",i);
      lcdPrint(16,1,buff);
      delay(1000);
  }
  lcdPrint(0,2,"-Stop Main Pump ");  
  digitalWrite(relayNo[PUMPMAIN],LOW);
  lcdPrint(0,3,"-Close AB Valve ");  
  digitalWrite(relayNo[VALVEAB],LOW);
  lcd.clear();
}

//***==Function: sync time with RTC clock ================//
void SyncTime() {
  DateTime now = rtc.now();
  setTime(now.hour(),now.minute(),now.second(),now.day(),now.month(),now.year()-2000);
  lcd.clear();
  lcd.print("Sync RTC ...");
  delay(1000);
  lcd.clear();
}

//**===Function: show menu ================================//
void menu() {
  int i=0;
  char key = keypad.getKey();
  if (key == '#') {
    lcd.clear();
    for (int i=0; i<4; i++) lcdPrint(0,i,menus[i]);
    while(1) {
      key = keypad.waitForKey();
      if (key == '#') {
          lcd.clear();
          for (i=4; i<8; i++) lcdPrint(0,i-4,menus[i]);
       }
       else if (key == '.') {
          lcd.clear();
          for (i=0; i<4; i++) lcdPrint(0,i,menus[i]);
       }
       else if (key == '0') {
          lcd.clear();
          break;
       }
       // Press A 
       else if (key == 'A') {
          if (i < 5) {
            setEC();
            for (i=0; i<4; i++) lcdPrint(0,i,menus[i]);
          }
          else {
            setDatetime();       
            for (i=4; i<8; i++) lcdPrint(0,i-4,menus[i]);
          }
       }
       // Press B
       else if (key == 'B') {
         if (i < 5) {
            sequences();
            for (i=0; i<4; i++) lcdPrint(0,i,menus[i]);
          }
          else {
            systemSettings();
            for (i=4; i<8; i++) lcdPrint(0,i-4,menus[i]);
          }
       }
       
       // Press C 
       else if (key == 'C') {
          if (i < 5) {
            calibrateEC();
            for (i=0; i<4; i++) lcdPrint(0,i,menus[i]);
          }
          else {
            resetSettings();       
            for (i=4; i<8; i++) lcdPrint(0,i-4,menus[i]);
          }
       }       
       // Press D
       else if (key == 'D') {
           Test();
           for (i=0; i<4; i++) lcdPrint(0,i,menus[i]);         
       }
    }
    lcd.clear();
  }
}

//***==Function: set pareameters of EC ===================================================//
// Auto mode:  AB dosing when EC is lower than low limit EC and STOP when reach EC setpoint
// Manual mode: dosing AB on mix AB schedule
void setEC() {
  char key;
  char buffers[20];
  
  lcd.clear();
  lcdPrint(0,0,"<*-- EC --          ");  
  lcdPrint(0,1,"A>SetPoint  EC: ");lcd.print(EC_sp);
  lcdPrint(0,2,"B>Low Limit EC: ");lcd.print(EC_lp);
  lcdPrint(0,3,"C>Auto Mode   : "); lcd.print(yes_no[EC_enable]);
  while(key = keypad.waitForKey()) {
    switch (key) {
      case 'A':  
               dtostrf(EC_sp, 2, 2, buffers);
               EC_sp = getKeyEC(16,1,buffers); //Serial.println(EC_sp);
               break;
      case 'B':
               dtostrf(EC_lp, 2, 2, buffers);
               EC_lp = getKeyEC(16,2,buffers); //Serial.println(EC_lp);
               break;
      case 'C':
               EC_enable = getKeyYesNo(16,3,EC_enable);
               break;
      case '.':
               lcd.clear();
               settings.ab.ec_sp=EC_sp;
               settings.ab.ec_ll=EC_lp;
               settings.ab.ec_enabled=EC_enable;
               eeprom_write_block((const void*)&settings, (void*)0, sizeof(settings)); //save all configuration to eeprom
               return;
    }
  }
}

//***==Function: getkey yes/no for setEC) ==========================//
int getKeyYesNo(int c, int r,int sel){
  char key;
  
  lcd.setCursor(c,r);
  lcd.blink();
  while(key = keypad.waitForKey()) {
    switch (key) {
      case 'C':
          sel = sel ? 0:1;
          lcd.setCursor(c,r);
          lcd.print(yes_no[sel]);
          break;
          
      case '#':
          lcd.noBlink();
          return sel;
    }
  }
}

//***==Function: assign schedule for do functions ===========================//
void sequences() {
  char key;
  
  lcd.clear();
  lcdPrint(0,0,"<*--Watering Cycle--");
  lcdPrint(0,1,"A>Watering AB      ");
  lcdPrint(0,2,"B>Watering WTR     ");
  lcdPrint(0,3,"C>Mix AB           ");
  
  while(key=keypad.waitForKey()){
    switch(key) {
      case '.': lcd.clear();
                return;
      case 'D': resetFunc();
                return;
      case 'A': setTimes(0);
                break;
      case 'B': setTimes(1);
                break;
      case 'C': setTimes(2);
                break;
      }

    lcd.clear();
    lcdPrint(0,0,"<*--Watering Cycle--");
    lcdPrint(0,1,"A>Watering AB       ");
    lcdPrint(0,2,"B>Watering WTR      ");
    lcdPrint(0,3,"C>Mix AB       D>Rst");
  }
}

//**==Function: with function set times ==========================//
void setTimes(int f) {
  char key;
  char *fname[3] = {"A:","B:","C:"};
  int r=0;
  int c=3;
  char data[20][4];
  char hbuff[20];
  char mbuff[20];
  int i,j,s;

  lcd.clear();  
  for (i=0; i<20;i++){
    for (j=0; j<4;j++){
      data[i][j] = '-';
    }
  }
  s=f*12;
  for (i=0; i<20;i++){
     for (j=0; j<4;j++){
       if (i==3 || i==9 || i==15) {
          lcd.setCursor(i,j);
          if (settings.sequence[s].h == -1) 
            lcd.print("--");
          else {
            sprintf(hbuff,"%02d",settings.sequence[s].h);
            data[i][j]=hbuff[0];
            data[i+1][j]=hbuff[1];            
            lcd.print(hbuff);
          }
          lcd.print(":");
          if (settings.sequence[s].m == -1)
            lcd.print("--");
          else {
            sprintf(mbuff,"%02d",settings.sequence[s].m);
            data[i+3][j]=mbuff[0];
            data[i+4][j]=mbuff[1];
            lcd.print(mbuff);
          }
          
          lcd.print(" ");
          s++;
       }
     }
  }
  
  lcd.setCursor(0,0);lcd.print(fname[f]);
  lcd.setCursor(0,2);lcd.print("#-");
  lcd.setCursor(0,3);lcd.print("<*");
  lcd.setCursor(c,r);lcd.blink();
  
  while(key=keypad.waitForKey()){
   // Serial.println(key);
    switch (key) {
      // navigate cursor
      case 'A': case 'B': case 'C': case 'D':
                if (key == 'A') {
                  c++;
                  if (c == 5 || c == 8 || c == 11 || c == 14 || c == 17) c++;
                }
                else if (key == 'B') {
                  c--;
                  if (c == 5 || c == 8 || c == 11 || c == 14 || c == 17) c--;
                }
                else if (key == 'C') r--;
                else if (key == 'D') r++;
                if (c < 3) c=3;
                else if (c > 19)c=19;
                if (r < 0) r=0;
                else if (r > 3) r=3;
     
                lcd.setCursor(c,r);
                break;
  
      case '.': 
                s=f*12;
                Serial.println("===save===");
                for (i=0; i<20;i++){
                   for (j=0; j<4;j++){
                     if (i==3 || i==9 || i==15) {
                        sprintf(hbuff,"%c%c",data[i][j],data[i+1][j]);
                        sprintf(mbuff,"%c%c",data[i+3][j],data[i+4][j]);
//                        Serial.print(hbuff);
//                        Serial.println(mbuff);
                        if (data[i][j] == '-' || data[i+1][j] == '-') 
                          settings.sequence[s].h = -1;
                        else 
                          settings.sequence[s].h = String(hbuff).toInt();
                          
                        if (data[i+3][j] == '-' || data[i+4][j] == '-') 
                          settings.sequence[s].m = -1;
                        else 
                          settings.sequence[s].m = String(mbuff).toInt();
                        settings.sequence[s].f = f;
                        s++;
                     }
                   }
                }
                eeprom_write_block((const void*)&settings, (void*)0, sizeof(settings)); //save all configuration to eeprom
                
                lcd.noBlink();
                lcd.clear();
                return;
      case '#':
                lcd.setCursor(c,r);
                lcd.print("-");
                data[c][r]='-';
                c++;
                if (c == 5 || c == 8 || c == 11 || c == 14 || c == 17) c++;
                    if (c > 19) {
                      c=3;
                      r++;
                      if (r > 3) r=0;
                    }  
                    lcd.setCursor(c,r);
                break;
      case '0': case '1': case '2': case '3': case '4':
      case '5': case '6': case '7': case '8': case '9':
                if (c == 3 || c == 9 || c==15){
                  if (key == '0' || key == '1' || key == '2') {
                    lcd.setCursor(c,r);
                    lcd.print(key);
                    data[c][r]=key;
                    c++;
                    if (c == 5 || c == 8 || c == 11 || c == 14 || c == 17) c++;
                    if (c > 19) {
                      c=3;
                      r++;
                      if (r > 3) r=0;
                    }  
                    lcd.setCursor(c,r);
                  }
                }
                else if (c == 6 || c == 12 || c == 18) {
                  if (key == '0' || key == '1' || key == '2' || key == '3' || key == '4' || key == '5') {
                    lcd.setCursor(c,r);
                    lcd.print(key);
                    data[c][r]=key;
                    c++;
                    if (c == 5 || c == 8 || c == 11 || c == 14 || c == 17) c++;
                    if (c > 19) {
                      c=3;
                      r++;
                      if (r > 3) r=0;
                    }  
                    lcd.setCursor(c,r);                  
                  }
                }
                else {
                  lcd.setCursor(c,r);
                  lcd.print(key);

                  data[c][r]=key;

                  c++;
                  if (c == 5 || c == 8 || c == 11 || c == 14 || c == 17) c++;
                  if (c > 19) {
                    c=3;
                    r++;
                    if (r > 3) r=0;
                  }  
                  lcd.setCursor(c,r);
                }
                break;
    }
  }  
}

//***==Function: system parameters setting =====================================//
void systemSettings(){
  char key;
  
  lcd.clear();
  lcdPrint(0,0,"<*--Settings--    ");
  lcdPrint(0,1,"A>Watering AB     "); // config watering consump  for caculation of dripping time
  lcdPrint(0,2,"B>Watering WTR    "); // watering water time
  lcdPrint(0,3,"C>Mix AB          "); // adjust dosing pump AB time
  
  while(key=keypad.waitForKey()){
    switch(key) {
      case '.':
                lcd.clear();
                return;
      case 'D':
                resetFunc();
                return;
      case 'A': 
                setWateringAB();
                break;
      case 'B': 
                setWateringWTR();
                break;
      case 'C': ;
                setMixAB();
                break;
    }
    lcdPrint(0,0,"<*--Settings--      ");
    lcdPrint(0,1,"A>Watering AB       "); // config watering consump  for caculation of dripping time
    lcdPrint(0,2,"B>Watering WTR      "); // watering water time
    lcdPrint(0,3,"C>Mix AB       D>Rst"); // adjust dosing pump AB time
  }
}

//***==Function: set parameter for Mix AB =========================//
/* 
 * A> Time limit for filling water
 * B> Time for dosing A&B pump
 * C> Waiting time for getting EC
 * D> Adjust A and B pump feed rate
 *    calibration: using pure water for dosing and then press
 *      1> start pump A&B about 60 seconds
 *      D> enter A/B ratio (weight) 
 *      2> start pump A&B about 60 seconds plus some delay time 
 */
void setMixAB() {
  char key;
  char buff[20];
  
  lcd.clear();
  lcdPrint(0,0,"A>Main WTR    =    s");
  lcdPrint(0,1,"B>Dosing Time =    s");
  lcdPrint(0,2,"C>Waiting Time=    s");
  lcdPrint(0,3,"D>A/B Ratio =   /   ");  
  lcd.setCursor(16,0);lcd.print(settings.ab.fill_time);
  lcd.setCursor(16,1);lcd.print(settings.ab.dosing_time);
  lcd.setCursor(16,2);lcd.print(settings.ab.dosing_wait_time);
  lcd.setCursor(13,3);lcd.print(round(settings.ab.pump_a_ratio));
  lcd.setCursor(17,3);lcd.print(round(settings.ab.pump_b_ratio));
  while(key=keypad.waitForKey()){
    switch(key) {
      case '.':
                eeprom_write_block((const void*)&settings, (void*)0, sizeof(settings));
                lcd.clear();
                return;
      case 'A':
                dtostrf(settings.ab.fill_time, 2, 0, buff);
                settings.ab.fill_time = getKeyEC(16,0,buff);
                break;
      case 'B':
                dtostrf(settings.ab.dosing_time, 2, 0, buff);
                settings.ab.dosing_time = getKeyEC(16,1,buff);
                break;
      case 'C':
                dtostrf(settings.ab.dosing_wait_time, 2, 0, buff);
                settings.ab.dosing_wait_time = getKeyEC(16,2,buff);
                break;
      case 'D':
                setABRatio();
                break;
      case '1':
                calABRatio();
                break;
      case '2':
                testCalABRatio();
                break;
     }
   }
}

//**===Funciton set A/B ratio ===================//
void setABRatio() {
  char buffa[20];
  char buffb[20];
  int c=13;
  char key;

  dtostrf(settings.ab.pump_a_ratio, 3, 0, buffa);
  dtostrf(settings.ab.pump_b_ratio, 3, 0, buffb);  
  lcdPrint(c,3,buffa);
  lcdPrint(c+4,3,buffb);
  lcd.setCursor(c,3);
  lcd.blink();
  
  while(key = keypad.waitForKey()) {
    switch (key) {
      case 'A': case 'B': case 'C': case 'D': 
              lcd.setCursor(c,3);
              c=13;
              break;
      case '#': 
                lcd.noBlink();
                settings.ab.pump_a_ratio = String(buffa).toFloat();
                settings.ab.pump_b_ratio = String(buffb).toFloat();
                return;
      default :
               lcd.print(key);
               if (c >= 13 && c <= 15) {
                 buffa[c-13]=key;
               }
               if (c >= 17 && c <= 19) {
                 buffb[c-17]=key;
               }
               
               c = (++c >= 19) ? 19:c;
               if (c == 16) c++;
               lcd.setCursor(c,3);
    }
  }

}

//**===Function: calibrate AB ratio ======================//
void calABRatio() {
  int i;
  char buffa[20];
  char buffb[20];

  digitalWrite(relayNo[PUMPA], HIGH);
  digitalWrite(relayNo[PUMPB], HIGH);
  for(i=60; i>0; i--){
    lcdPrint(13,3,"       ");  
    lcdPrint(13,3,"Cal:");lcd.print(i);
    delay(1000);
  }
  digitalWrite(relayNo[PUMPA], LOW);
  digitalWrite(relayNo[PUMPB], LOW);
  dtostrf(settings.ab.pump_a_ratio, 3, 0, buffa);
  dtostrf(settings.ab.pump_b_ratio, 3, 0, buffb);  
  lcdPrint(13,3,buffa);
  lcdPrint(17,3,buffb);
}

//**===Function: test calibrate AB ratio ======================//
void testCalABRatio() {
  int i;
  char buffa[20];
  char buffb[20];
  float dt;
  
  digitalWrite(relayNo[PUMPA], HIGH);
  digitalWrite(relayNo[PUMPB], HIGH);
  for(i=60; i>0; i--){
    lcdPrint(13,3,"       ");  
    lcdPrint(13,3,"AB:");lcd.print(i);
    delay(1000);
  }
  if (settings.ab.pump_b_ratio >= settings.ab.pump_a_ratio) {
      digitalWrite(relayNo[PUMPB],LOW);
      dt = (settings.ab.pump_b_ratio - settings.ab.pump_a_ratio)/settings.ab.pump_a_ratio*settings.ab.dosing_time;
      lcdPrint(13,3,"       ");  
      lcdPrint(13,3,"A:");lcd.print(round(dt*1000));
      delay(round(dt*1000));
      digitalWrite(relayNo[PUMPA],LOW);
    }
    else {
      digitalWrite(relayNo[PUMPA],LOW);
      dt = (settings.ab.pump_a_ratio - settings.ab.pump_b_ratio)/settings.ab.pump_b_ratio*settings.ab.dosing_time;
      lcdPrint(13,3,"       ");  
      lcdPrint(13,3,"B:");lcd.print(round(dt*1000));
      delay(round(dt*1000));
      digitalWrite(relayNo[PUMPB],LOW);      
    }
    
  dtostrf(settings.ab.pump_a_ratio, 3, 0, buffa);
  dtostrf(settings.ab.pump_b_ratio, 3, 0, buffb);  
  lcdPrint(13,3,buffa);
  lcdPrint(17,3,buffb);
}

//***==Function: set time for watering water ==============//
void setWateringWTR() {
  char key;
  char buff[20];
  
  lcd.clear();
  lcdPrint(0,0,"A>WTR Time =      s");
  lcdPrint(0,3,"<*                 ");  
  lcd.setCursor(13,0);lcd.print(settings.wtr.watering_time);
  while(key=keypad.waitForKey()){
    switch(key) {
      case '.':
                eeprom_write_block((const void*)&settings, (void*)0, sizeof(settings));
                lcd.clear();
                return;
      case 'A':
                dtostrf(settings.wtr.watering_time, 2, 0, buff);
                settings.wtr.watering_time = getKeyEC(13,0,buff);
     }
   }
}

//***=== Function: set parameters for watering AB ======================//
/* 
 * A> consumption of plant lits per day
 * B> drip rate for each dripping head
 * C> recirculation time before watering 
 * then show calculation time for each dripping 
 */
void setWateringAB() {
  char key;
  char buff[20];
  
  lcd.clear();
  lcdPrint(0,0,"A>Consump =      l/d");
  lcdPrint(0,1,"B>Dp Rate =      l/h" );
  lcdPrint(0,2,"C>Pre Time=        s");
  lcdPrint(0,3,"Drip (s/n)=      /  ");
  
  lcd.setCursor(12,0);lcd.print(settings.ab.plant_consump);
  lcd.setCursor(12,1);lcd.print(settings.ab.drip_rate);
  lcd.setCursor(12,2);lcd.print(settings.ab.prepare_time);
  lcd.setCursor(18,3);lcd.print(watering_ab_round);
  
  drip_time =round((settings.ab.plant_consump/settings.ab.drip_rate)*3600/watering_ab_round);
  lcd.setCursor(12,3);lcd.print(drip_time);
  while(key=keypad.waitForKey()){
    switch(key) {
      case '.':
                eeprom_write_block((const void*)&settings, (void*)0, sizeof(settings)); //save all configuration to eeprom
                lcd.clear();
                return;
      case 'A':
                dtostrf(settings.ab.plant_consump, 2, 2, buff);
                settings.ab.plant_consump = getKeyEC(12,0,buff);
                break;
      case 'B':
                dtostrf(settings.ab.drip_rate, 2, 2, buff);
                settings.ab.drip_rate = getKeyEC(12,1,buff);
                break;
      case 'C':
                dtostrf(settings.ab.prepare_time, 2, 0, buff);
                settings.ab.prepare_time = getKeyEC(12,2,buff);
                break;
    }
    drip_time =round((settings.ab.plant_consump/settings.ab.drip_rate)*3600/watering_ab_round);
    lcd.setCursor(12,3);;lcd.print(drip_time);lcd.print("  ");
  }
}

//***===Function: Testing relays ============================//
/*
 *  press 1-8 for relays
 *  press 0 for all relays
 *  press 9 for pump A and B
 */
void Test() {
  char key;
  int rno;
  int rstatus;
  int i;
  
  lcd.clear();
  lcdPrint(0,0,"<*-- Test--   ");
  lcdPrint(0,3,"Sel 0-9   A/D:on/off");
  while(key=keypad.waitForKey()){
    switch(key) {
      case '.':
               lcd.clear();
               return;
      case '1': case '2': case '3': case '4':
      case '5': case '6': case '7': case '8': case '9':
              lcd.setCursor(0,1);
              if (key == '9') {
                rno = String(key).toInt()-1;            
                lcd.print(relay_name[rno]);
                if(digitalRead(relayNo[PUMPA]) && digitalRead(relayNo[PUMPB])) {
                  rstatus = 1;
                }
                else {
                  rstatus = 0;
                }
              }
              else {
                rno = String(key).toInt()-1;            
                lcd.print(relay_name[rno]);
                rstatus = digitalRead(relayNo[rno]);
              }
              
              if (rstatus == 1) {
                 lcdPrint(15,1,"ON ");
              }
              else {
                 lcdPrint(15,1,"OFF");
              }
             break;
       case '0':
             rno = -1;
             lcdPrint(0,1,">All Relay          ");             
             break;
       case 'A': 
             lcdPrint(15,1,"ON ");
             if (rno == 8) {
                 digitalWrite(relayNo[PUMPA], HIGH);
                 digitalWrite(relayNo[PUMPB], HIGH);
             }
             else if (rno == -1) {
               for(i=0;i<noRELAY;i++)                 
                 digitalWrite(relayNo[i], HIGH);
             }
             else {
              if (key == 7 && digitalRead(LEVEL_HH)) {
               digitalWrite(relayNo[rno], LOW);                
              }
              else {
               digitalWrite(relayNo[rno], HIGH);
              }
             }
             break;             
       case 'D': 
             lcdPrint(15,1,"OFF");
             if (rno == 8) {
                 digitalWrite(relayNo[PUMPA], LOW);
                 digitalWrite(relayNo[PUMPB], LOW);
             }
             else if (rno == -1) {
               for(i=0;i<noRELAY;i++)                 
                 digitalWrite(relayNo[i], LOW);
             }
             else {
               digitalWrite(relayNo[rno], LOW);
             }
             break;     
    }
  }
}

//***===Function: set parameters to default ========================//
void resetSettings() {
  char key;
  lcd.clear();
  lcdPrint(0,0,"--Reset--");
  lcdPrint(0,1,"Set data?(#/*)");
  lcdPrint(0,3,"#:yes / *:No");
  key=keypad.waitForKey();
  if (key == '*') {
    lcd.clear();
    return;
  }
  else if (key == '#') {
    for (int i=0;i<noSEQ;i++) {
      settings.sequence[i].h=-1;
      settings.sequence[i].m=-1;
      settings.sequence[i].f=-1;
    }    
    settings.ab.fill_time=300;
    settings.ab.dosing_time=30;
    settings.ab.dosing_wait_time=30;
    settings.ab.plant_consump=1;
    settings.ab.drip_rate=4;
    settings.ab.prepare_time=60;
    settings.ab.ec_sp=2.5;
    settings.ab.ec_ll=1.0;
    settings.ab.ec_enabled=0;
    settings.ab.pump_a_ratio=1;
    settings.ab.pump_b_ratio=1;
    settings.wtr.watering_time=60;
    settings.K=1.89;

    eeprom_write_block((const void*)&settings, (void*)0, sizeof(settings)); //save all configuration to eeprom
  
    lcd.clear();
    lcdPrint(0,0,"Initial data done.");
    eeprom_read_block((void*)&settings, (void*)0, sizeof(settings));
    EC_sp=settings.ab.ec_sp;
    EC_lp=settings.ab.ec_ll;
    EC_enable= settings.ab.ec_enabled;
    delay(2000);
    lcd.clear();
    resetFunc();
    return;
  }
}

//***===Function: get calibration EC ============================================//
void calibrateEC() {
  lcd.clear();
  lcdPrint(0,0,"<D--EC Calibration--");
  lcdPrint(0,1,"? Cal EC = ");
  lcdPrint(0,3,"#=Start");
  float data = getKeyData(11,1);
  if (data == -1) {
    lcd.clear();
    return;
  }
  else {
    CalibrationEC=data;
    doCalibration();
  }
}

//***===Function:getkey data ============================//
float getKeyData(int c,int r) {
  char key;
  char keybuf[10];
  int i=0;
  int old_c=c;
  int old_r=r;
  
  lcd.setCursor(c,r);
  lcd.blink();
  for(int m=0;m<10;m++) keybuf[m]='0';
  
  while (key=keypad.waitForKey()) {
    lcd.setCursor(c,r);
    if (key == '#') {
      lcd.noBlink();
      lcd.print(keybuf);
      return atof(keybuf);
    }
    else if (key == 'D') {
      lcd.noBlink();
      return -1;
    }
    else if (key == 'A' || key == 'B') {
      c--;
      i--;
      c = c <= old_c ? old_c:c;
      i = i <= 0 ? 0:i;
      keybuf[i] = ' ';
      lcd.setCursor(c,r);
      lcd.print(keybuf[i]);
      lcd.setCursor(c,r);
    }
    else if (key == 'C') {
      c=old_c;
      lcd.setCursor(c,r);
      for(i=0;i<10;i++) {
        keybuf[i] = ' ';
        lcd.print(keybuf[i]);
      }
      lcd.setCursor(c,r);
      i=0;
    }
    else {
      keybuf[i++] = key;
      c++;
      lcd.print(key);
    }
  }
}

//***===Function: do Calibraion ===============================//
void doCalibration() {
  float oK=K;   // saving old K
  char key;
  
  lcd.clear();
  lcdPrint(0,0,"Cal EC=");
  lcd.print(CalibrationEC);
  while (1) {
      int i=1;
      buffer=0;
   
      sensors.requestTemperatures();                        // Send the command to get temperatures
      TemperatureStart=sensors.getTempCByIndex(0);          //Stores Value in Variable
      lcdPrint(0,1,"Temp Start = ");lcd.print(TemperatureStart);
      
      //******Estimates Resistance of Liquid ***************//
      while(i<=10){
        digitalWrite(ECPower,HIGH);
        raw= analogRead(ECPin);
        raw= analogRead(ECPin);                              // This is not a mistake, First reading will be low
        digitalWrite(ECPower,LOW);
        buffer=buffer+raw;
        lcdPrint(0,2,"Raw ");lcd.print(i);lcd.print(" = ");lcd.print(raw);        
        i++;
        delay(5000);
      }
      
      raw=(buffer/10);                              // cal average raw
      sensors.requestTemperatures();                // get finish temperatures
      TemperatureFinish=sensors.getTempCByIndex(0); //Stores Value in Variable
      lcdPrint(0,3,"Temp Finish=");lcd.print(TemperatureFinish);
 
      //*************Compensating For Temperaure********************//
      EC =CalibrationEC*(1+(TemperatureCoef*(TemperatureFinish-25.0))) ;
       
      //***************** Calculates R relating to Calibration fluid **************************//
      Vdrop= (((Vin)*(raw))/1024.0);
      Rc=(Vdrop*R1)/(Vin-Vdrop);
      Rc=Rc-Ra;
      K= 1000/(Rc*EC);
      if (TemperatureStart == TemperatureFinish){
        lcd.clear();
        lcdPrint(0,0,"Result is OK.");
        lcdPrint(0,1,"K=");lcd.print(K); 
        lcdPrint(0,2,"Save and Exit (#/.)");
        key= keypad.waitForKey();
        if (key == '#') {
          settings.K=K;
          eeprom_write_block((const void*)&settings, (void*)0, sizeof(settings)); //save all configuration to eeprom
          lcdPrint(0,3,"Save data done.");
          delay(1000);
          lcd.clear();        
        }
        else {
          K=oK;   // if not restore old value
        }      
        return;        
      }
      else{
        delay(2000);
        lcdPrint(0,3,"Err-Temp not settle");
      }
  }
}

//***===Function: get EC for calibration =========================================//
float getKeyEC(int c,int r, char buffers[20]) {
  char key;
  int rc=c; // save original reference column
  float val;
  
  lcd.setCursor(c,r);
  lcdPrint(c,r,buffers);
  lcd.setCursor(c,r);
  lcd.blink();
  while(key = keypad.waitForKey()) {
    switch (key) {
      case 'A': case 'B': case 'C': case 'D': 
              lcd.setCursor(rc,r);
              c=rc;
              break;
      case '#': 
                lcd.noBlink();
                val = String(buffers).toFloat();
                return val;
      default :
               lcd.print(key);
               buffers[c-rc]=key;
               c = (++c >= 19) ? 19:c;
              lcd.setCursor(c,r);
    }
  }
}

//************ This Loop Is called From Main Loop************************//
void GetEC(){
  //*********Reading Temperature Of Solution *******************//
  sensors.requestTemperatures();// Send the command to get temperatures
  Temperature=sensors.getTempCByIndex(0); //Stores Value in Variable
   
  //************Estimates Resistance of Liquid ****************//
  digitalWrite(ECPower,HIGH);
  raw= analogRead(ECPin);
  raw= analogRead(ECPin);// This is not a mistake, First reading will be low beause if charged a capacitor
  digitalWrite(ECPower,LOW);
   
  //***************** Converts to EC **************************//
  Vdrop= (Vin*raw)/1024.0;
  Rc=(Vdrop*R1)/(Vin-Vdrop);
  Rc=Rc-Ra; //acounting for Digital Pin Resitance
  EC = 1000/(Rc*K);
   
  //*************Compensating For Temperaure********************//
  EC25  =  EC/ (1+ TemperatureCoef*(Temperature-25.0));
  ppm=(EC25)*(PPMconversion*1000);
}
//************************** End OF EC Function ***************************//

//***===Function: set date time  to RTC (Real Time Clock)========================================//
void setDatetime() {
  char buffers[20]; // buffers
  char *dt;         // new datetime
  
  lcd.clear();
  lcdPrint(0,0,"--Clock Adjust--    ");
  lcdPrint(0,3,"*/#,D=Save&Exit");
  
  DateTime now = rtc.now();
  sprintf(buffers,"%02d/%02d/%02d %02d:%02d",now.day(),now.month(),now.year(),now.hour(),now.minute());
  dt = getKeyDatetime(0,1,buffers);
  String datetime = String(dt);
  int d = datetime.substring(0,2).toInt();
  int m = datetime.substring(3,5).toInt();
  int y = datetime.substring(6,10).toInt();
  int h = datetime.substring(11,13).toInt();
  int i = datetime.substring(14,16).toInt();
  if (isValidDatetime(y,m,d,h,i)) {
    rtc.adjust(DateTime(y,m,d,h,i,0));
  }
  else {
    lcd.clear();
    lcd.print("Err-input format.");
    delay(2000);
  }
  lcd.clear();
}

//***==Function: check valid time =================================================//
bool isValidDatetime(int yy, int mm,int dd,int hh,int ih){
    if (hh >= 0 && hh <= 23 && mm >=0 && mm <= 59) { // check time
      if(yy>=1900 && yy<=9999){ // check year
          //check month
          if(mm>=1 && mm<=12){
              //check days
              if((dd>=1 && dd<=31) && (mm==1 || mm==3 || mm==5 || mm==7 || mm==8 || mm==10 || mm==12))
                  return true;
              else if((dd>=1 && dd<=30) && (mm==4 || mm==6 || mm==9 || mm==11))
                  return true;
              else if((dd>=1 && dd<=28) && (mm==2))
                  return true;
              else if(dd==29 && mm==2 && (yy%400==0 ||(yy%4==0 && yy%100!=0)))
                  return true;
              else
                  return false;
          }
          else{
                  return false;
          }
      }
      else{
          return false;
      }
    }
    else {
       return false;
    }
}

//***==Function: get key for datatime =====================================//
char *getKeyDatetime(int cur,int r, char buffers[20]) {
  char key;
  int rc=cur; // save original reference column
  
  lcdPrint(cur,r,buffers);
  lcd.setCursor(cur,r);
  lcd.blink();
  while(key = keypad.waitForKey()) {
    switch (key) {
      case '#': cur++; if (cur >= 15) cur=rc+15;  if (cur==rc+2 || cur==rc+5 || cur==rc+13 || cur==rc+10) cur++; break;
      case '.': cur--; if (cur <=0) cur = 0; if (cur==rc+2 || cur==rc+5 || cur==rc+13 || cur==rc+10) cur--; break;
      case 'D': 
                lcd.noBlink();
                return buffers;
      case 'A':
      case 'B':
      case 'C': break;
      default:
                lcd.print(key);
                buffers[cur-rc]=key;
                cur++;
                if (cur >= 15) cur=rc+15;  if (cur==rc+2 || cur==rc+5 || cur==rc+13 || cur==rc+10) cur++;
                break;
    }
    lcd.setCursor(cur,r);    
  }
}

//*** Utilities functions ****************************//
//***===Function print LCD at location ===============//
void lcdPrint(int c, int r, char *msg) {
  lcd.setCursor(c,r);
  lcd.print(msg);
}

