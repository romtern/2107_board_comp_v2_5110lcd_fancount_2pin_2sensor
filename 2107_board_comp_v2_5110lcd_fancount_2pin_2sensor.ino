
#include <LCD5110_Graph.h>
#include <iarduino_RTC.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <avr/pgmspace.h>
const uint8_t FAN_image_mas[] PROGMEM= 
{ 0x00, 0x18, 0x3E, 0x7E, 0xFF, 0xFF, 0xFE, 0xC0,
  0x60, 0xF8, 0xFC, 0x7C, 0x7E, 0x7E, 0x7C, 0x38,
  0x18, 0x3C, 0x7C, 0x7C, 0x7C, 0x7E, 0x3F, 0x0E,
  0x04, 0x06, 0xFF, 0xFE, 0xFE, 0xFC, 0x78, 0x00};

const uint8_t VAZ_LOGO[] PROGMEM= 
{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x80, 0x80, 0x80, 0xC0, 0x40, 0x40, 0x60, 0x20, 0x20, 0x20, 0x30,
  0x10, 0x90, 0x90, 0x90, 0x9C, 0x9C, 0xDC, 0xCC, 0xCC, 0xCC, 0xCC, 0x4C,
  0x0C, 0x0C, 0xFC, 0xFE, 0xFE, 0xFE, 0xEE, 0xCE, 0x0E, 0x0E, 0x0C, 0x0C,
  0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x1C, 0x1C, 0x10, 0x90, 0x90,
  0x10, 0x30, 0x20, 0x20, 0x20, 0x60, 0x40, 0x40, 0xC0, 0x80, 0x80, 0x80,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xC0, 0xE0, 0x30, 0x10, 0x08, 0x0C, 0x04, 0x07,
  0xC3, 0xE3, 0xF0, 0xF0, 0xF8, 0xF8, 0xF8, 0xFC, 0xFC, 0xFC, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0x00,
  0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x0C, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xF0, 0xFC, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFC, 0xFC, 0xFC, 0xF8, 0xF8, 0xF0, 0xF0, 0xE0, 0xE3,
  0xC3, 0x07, 0x04, 0x0C, 0x08, 0x10, 0x30, 0xE0, 0xC0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xFC, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFE, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0x00, 0xF0, 0xFE,
  0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x0F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x80, 0xC0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFE, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0E, 0xFC, 0x00,
  0x00, 0x00, 0x01, 0x0E, 0x1C, 0x20, 0x60, 0xC0, 0x80, 0x01, 0x0F, 0x1F,
  0x3F, 0x3F, 0x7F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x81, 0x80, 0x1E, 0x0F, 0x8F,
  0x83, 0x81, 0x80, 0xC0, 0xC0, 0xE0, 0xE0, 0xF0, 0xF0, 0xDC, 0x8E, 0x0E,
  0x0F, 0x0F, 0x4F, 0x4F, 0xCF, 0xCF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x7F, 0x3F,
  0x1F, 0x0F, 0x03, 0x81, 0x80, 0xC0, 0x60, 0x30, 0x1C, 0x0F, 0x01, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x07, 0x04,
  0x04, 0x08, 0x08, 0x18, 0x10, 0x10, 0x33, 0x23, 0x23, 0x27, 0xE7, 0xC7,
  0xCF, 0xCF, 0xCF, 0xCF, 0x1F, 0x1B, 0x1B, 0x1B, 0x1B, 0x13, 0x13, 0x13,
  0x13, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x30, 0x30,
  0x10, 0x10, 0x10, 0x18, 0x1C, 0x1F, 0x1F, 0x1F, 0xCF, 0xCF, 0xCF, 0xCF,
  0xCF, 0xC7, 0xE7, 0x27, 0x23, 0x23, 0x33, 0x10, 0x10, 0x18, 0x08, 0x0C,
  0x04, 0x04, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};



LCD5110 myGLCD(8,7,6,4,5);
extern uint8_t TinyFont[]; 
extern uint8_t SmallFont[]; 
extern uint8_t BigNumbers[];
extern uint8_t MediumNumbers[];

iarduino_RTC time(RTC_DS3231);

#define ONE_WIRE_BUS_1 A3                //ds18b20 conected to pin 17(A3)
#define ONE_WIRE_BUS_2 A6                //ds18b20 conected to pin 17(A3)
Wire oneWire_in(ONE_WIRE_BUS_1);
Wire oneWire_out(ONE_WIRE_BUS_2);                   
DallasTemperature sensor_in(&oneWire_in);
DallasTemperature sensor_out(&oneWire_out);
//DeviceAddress sensor1 = {0x28, 0x3D, 0x30, 0x54, 0x06, 0x0, 0x0, 0x3A};  // address sensor 1
//DeviceAddress sensor2 = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};        // address sensor 2
float temp1 = 0;
float temp2 = 0;

#define FAN 9   //  mosfet of fan control, conected to pin D9

#define PIN_VOLT_CONTROL A7   // pin for measurment input voltage 12V

// BUTTONS
const int SW_set = A0;        // push button for set_clock/fan_max on pin A0
const int SW_up = A2;         // push button for up on pin A1
const int SW_down = A1;       // push button for down on pin A2

uint8_t VAR_mode_SET = 0;     // ?????????? ?????????????????? ??????????????: 0-???? 1-?????? 2-???? 3-?????? 4-???????? 5-?????? 6-?????? 
uint16_t FANPWM = 0;
uint8_t FANPWM_min = 215;     // ???????????????? ?????????? ?????????????????? ?????????????? ??????????
uint8_t FANPWM_max = 255;
uint8_t FAN_range = 9;
uint8_t FANPWM_K = ((FANPWM_max - FANPWM_min)/(FAN_range-1));   // ????????????????????, n-???? ?????????????? ???????????????????? ???????????? ??????????????????????
uint8_t s=0;
uint8_t delay_button = 10;
float multiplerV = 3.02;

uint32_t timer1;
int time_temp_meas = 5000;

void setup() {

  //set PWM Frequency
  TCCR1A = 0b00000001;  // 8bit
  TCCR1B = 0b00000001;  // x1 phase correct PWM 31.4 KHz
  
  pinMode(SW_set, INPUT_PULLUP);
  pinMode(SW_up, INPUT_PULLUP);
  pinMode(SW_down, INPUT_PULLUP);

  Serial.begin(9600);     //-------------- tempory for debug --------------

  myGLCD.InitLCD(); 
  myGLCD.setContrast(55);
    
  Wire. begin();
  sensor_in.begin();                  //start ds18b20 sensor 1
  sensor_out.begin();                 //start ds18b20 sensor 2
  sensor_in.setResolution(9);        // ds18b20 resolution sensor 9,10,11,12 (time of measurment)
  sensor_out.setResolution(9);
  
  time.begin();  //start RTC Library
//?????????????????? ????????
  //time.settime(0,1,2,3,4,5,6);            // manual set time: 00 s, 01 min, 02 hour, 03 day, 04 mounth, 05 year, 06 day of week
 
  analogWrite (FAN, FANPWM);                // fan off
  myGLCD.drawBitmap(0, 0, VAZ_LOGO, 84, 40);
  myGLCD.setFont(SmallFont);
  myGLCD.print("BY ROMTERN", CENTER, 41);
  myGLCD.update();
  delay (2000);
  }

void loop() {  

  myGLCD.clrScr(); // ?????????????? ????????????
  myGLCD.setFont(BigNumbers); // ?????????????????? ???????????? ????????????????
  myGLCD.print(time.gettime("H-i"), LEFT, 0); // ?????????? ??????????
  myGLCD.setFont(SmallFont); // ?????????????????? ???????????? ????????????????
  myGLCD.print(time.gettime("s"), RIGHT, 0);
  
  if(VAR_mode_SET == 0){
    if(millis() - timer1 >= time_temp_meas){
      sensor_in.requestTemperatures();
      sensor_out.requestTemperatures();
      temp1 = sensor_in.getTempCByIndex(0);
      temp2 = sensor_out.getTempCByIndex(0);
      timer1 += time_temp_meas;
    }
    String outtemp = "Out:" + String(int(temp2)) + "~C"; 
    String intemp = "In:" + String(int(temp1)) + "~C";
    myGLCD.print(outtemp, LEFT, 25);
    myGLCD.print(intemp, LEFT, 33);
    }else{myGLCD.print(time.gettime("d/m/Y"), LEFT, 25);}
  
  

  
  float voltage = (float)((analogRead(PIN_VOLT_CONTROL) * 5.0) / 1024)* multiplerV;
  String BAT_V = "BAT:" + String(voltage) + "V";
  myGLCD.print(BAT_V, LEFT, 41);

  
  if(s > 0){
    myGLCD.drawBitmap(68, 32, FAN_image_mas, 16, 16);
    String fanspeed = String(int(s));
    if(s==FAN_range){fanspeed = "MAX";}
    myGLCD.print(fanspeed, RIGHT, 25);
  }
    
  myGLCD.update();
    

  Func_buttons_control();      // ???????????????? ???????????????????? ??????????????

}

// ?????????????? ???????????????????? ????????????????:
void Func_buttons_control(){
  uint8_t i=0;
  time.blinktime(VAR_mode_SET);                                    // ???????????? ?????????????????????????????? ???????????????????? (???????? VAR_mode_SET ???????????? 0)
//???????? ???????? ?????????????????? ?? ???????????? ?????????????????? ????????/??????????????
  if(VAR_mode_SET){
//  ???????? ???????????? ???????????? UP
    if(digitalRead(SW_up)==0){Serial.println("up");
      while(digitalRead(SW_up)){delay(delay_button);}                // ???????? ???????? ???? ???? ???????????????? ???????????? UP
      switch (VAR_mode_SET){                               // ?????????????????? (????????????????????) ???????????????????????????????? ????????????????
        /* ?????? */ case 1: time.settime(0,                                   -1, -1, -1, -1, -1, -1); break;
        /* ?????? */ case 2: time.settime(-1, (time.minutes==59?0:time.minutes+1), -1, -1, -1, -1, -1); break;
        /* ?????? */ case 3: time.settime(-1, -1, (time.Hours==23?0:time.Hours+1),     -1, -1, -1, -1); break;
        /* ?????? */ case 4: time.settime(-1, -1, -1, (time.day==31?1:time.day+1),         -1, -1, -1); break;
        /* ?????? */ case 5: time.settime(-1, -1, -1, -1, (time.month==12?1:time.month+1),     -1, -1); break;
        /* ?????? */ case 6: time.settime(-1, -1, -1, -1, -1, (time.year==99?0:time.year+1),       -1); break;
        }
      }
    
//  ???????? ???????????? ???????????? DOWN
    if(digitalRead(SW_down)==0){Serial.println("down");
      while(digitalRead(SW_down)){delay(delay_button);}                    // ???????? ???????? ???? ???? ???? ????????????????
      switch (VAR_mode_SET){                                     // ?????????????????? (????????????????????) ???????????????????????????????? ????????????????
        /* ?????? */ case 1: time.settime(0,                                   -1, -1, -1, -1, -1, -1); break;
        /* ?????? */ case 2: time.settime(-1, (time.minutes==0?59:time.minutes-1), -1, -1, -1, -1, -1); break;
        /* ?????? */ case 3: time.settime(-1, -1, (time.Hours==0?23:time.Hours-1),     -1, -1, -1, -1); break;
        /* ?????? */ case 4: time.settime(-1, -1, -1, (time.day==1?31:time.day-1),         -1, -1, -1); break;
        /* ?????? */ case 5: time.settime(-1, -1, -1, -1, (time.month==1?12:time.month-1),     -1, -1); break;
        /* ?????? */ case 6: time.settime(-1, -1, -1, -1, -1, (time.year==0?99:time.year-1),       -1); break;
        }
      }
      if(digitalRead(SW_set)==0){Serial.println("set");
        while(digitalRead(SW_set)==0){
          delay(delay_button);                //???????????? ???????????? ????????????
          if(i<200){i++;}else{VAR_mode_SET=0;}      // ???????? ???????????? SET ???????????????????????? ???????????? 2 ????????????, ???? ?????????????????? ?????????? ???? ???????????? ?????????????????? ????????/??????????????
        }
        if(i<200){                                              // ???????? ???????????? SET ???????????????????????? ???????????? 2 ????????????
          VAR_mode_SET++;                                       // ?????????????????? ?? ???????????????????? ???????????????????????????????? ??????????????????
          if(VAR_mode_SET>6){VAR_mode_SET=1;}                   // ???????????????????????? ?? ?????????????? ???????????????????????????????? ??????????????????
        }
      }
        
      
  }else{    
//  ???????? ???????????? ???????????? SET       ???????????????????????? ???????????????????? ???????????? SET ?????????????? ???????????????????? ???? ????????????????, ???????????????? ???????????????????????? ???????????????????? ???????????? ???????????????? ????????????????????
    if(digitalRead(SW_set)==0){Serial.println(" ");Serial.println("set");
      while(digitalRead(SW_set)==0){
        delay(delay_button);                          // ???????? ???????? ???????????????? ???????????? SET  
        if(i<200){i++;}
      } 
      if(i<200){
                if(FANPWM >= 0 & FANPWM != 255) {FANPWM = FANPWM_max; analogWrite(FAN, FANPWM); s=FAN_range; Serial.print(" FANPWM:");Serial.println(FANPWM);Serial.print(" fanspeed:");Serial.println(s);}      
                else if(FANPWM == FANPWM_max) {FANPWM = 0; analogWrite(FAN, FANPWM); s=0; Serial.print(" FANPWM:");Serial.println(FANPWM);Serial.print(" fanspeed:");Serial.println(s);}                
                }      
      if(i>=200){VAR_mode_SET++;}           // ???????? ???????????? SET ???????????????????????? ???????????? 2 ???????????? ???????????????????? ???? ???????????????????????? ????????
    }



//  ???????? ???????????? ???????????? UP    ???????????????????????? ???????????????????? ???????????????? ???????????? ?????????????????????? ???? 1/10 ?????? ???????????????? ??????????????????
    if(digitalRead(SW_up)==0){
      Serial.println(" ");Serial.println("up");
      while(digitalRead(SW_up)){delay(delay_button);}                // ???????? ???????? ???? ???? ???????????????? ???????????? UP
      if(FANPWM == 0){FANPWM = FANPWM_min; analogWrite(FAN, FANPWM); s++; Serial.print(" FANPWM:");Serial.println(FANPWM);Serial.print(" fanspeed:");Serial.println(s);}
      else{  
        if(FANPWM >= FANPWM_min & FANPWM < FANPWM_max) {FANPWM = FANPWM + FANPWM_K; s++; Serial.print(" FANPWM resault:");Serial.println(FANPWM);Serial.print(" fanspeed-resault:");Serial.println(s);} 
        if(FANPWM > FANPWM_max) {FANPWM = FANPWM_max; s=FAN_range; Serial.print(" FANPWM:");Serial.println(FANPWM);Serial.print(" fanspeed:");Serial.println(s);}
        else{analogWrite(FAN, FANPWM);Serial.print(" FANPWM:");Serial.println(FANPWM);Serial.print(" fanspeed:");Serial.println(s);}
      }   
    }
      
    
//  ???????? ???????????? ???????????? DOWN   ???????????????????????? ???????????????????? ?????????????? ???????????? ?????????????????????? ???? 1/10 ?????? ???????????????? ??????????????????
    if(digitalRead(SW_down)==0){Serial.println(" ");Serial.println("down");
      while(digitalRead(SW_down)){delay(delay_button);}                // ???????? ???????? ???? ???? ???????????????? ???????????? down
      if(FANPWM == FANPWM_min) {
        FANPWM = 0; analogWrite(FAN, FANPWM); s=0; Serial.print(" FANPWM:");Serial.println(FANPWM);Serial.print(" fanspeed:");Serial.println(s);}
      else{
        if(FANPWM <= FANPWM_max & FANPWM >= FANPWM_min) {FANPWM = FANPWM - FANPWM_K; s--; Serial.print(" FANPWM resault:");Serial.println(FANPWM);Serial.print(" fanspeed-resault:");Serial.println(s);}
        if(FANPWM <= FANPWM_max & FANPWM >= FANPWM_min) {analogWrite(FAN, FANPWM); Serial.print(" FANPWM:");Serial.println(FANPWM);Serial.print(" fanspeed:");Serial.println(s);}
        if(FANPWM == 0) {s=0; Serial.print(" FANPWM resault:");Serial.println(FANPWM);Serial.print(" fanspeed:");Serial.println(s);}
        else if(FANPWM <= FANPWM_min | FANPWM < FANPWM_min) {FANPWM = FANPWM_min; analogWrite(FAN, FANPWM); s=1; Serial.print(" FANPWM:");Serial.println(FANPWM);Serial.print(" fanspeed:");Serial.println(s);} 
      }
      
    }
   }
}
