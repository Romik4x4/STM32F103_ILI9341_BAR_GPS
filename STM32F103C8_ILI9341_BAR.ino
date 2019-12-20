#include <TinyGPS++.h>
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include <Wire.h>
#include <I2CEEPROM.h>
#include <DS3231.h>
#include <OneWire.h>
#include <DS18B20.h>
#include <Average.h>
#include <TimeLib.h>
#include <Sunrise.h>
#include <BME280I2C.h>

#define ILI9341_BLACK       0x0000  ///<   0,   0,   0
#define ILI9341_NAVY        0x000F  ///<   0,   0, 123
#define ILI9341_DARKGREEN   0x03E0  ///<   0, 125,   0
#define ILI9341_DARKCYAN    0x03EF  ///<   0, 125, 123
#define ILI9341_MAROON      0x7800  ///< 123,   0,   0
#define ILI9341_PURPLE      0x780F  ///< 123,   0, 123
#define ILI9341_OLIVE       0x7BE0  ///< 123, 125,   0
#define ILI9341_LIGHTGREY   0xC618  ///< 198, 195, 198
#define ILI9341_DARKGREY    0x7BEF  ///< 123, 125, 123
#define ILI9341_BLUE        0x001F  ///<   0,   0, 255
#define ILI9341_GREEN       0x07E0  ///<   0, 255,   0
#define ILI9341_CYAN        0x07FF  ///<   0, 255, 255
#define ILI9341_RED         0xF800  ///< 255,   0,   0
#define ILI9341_MAGENTA     0xF81F  ///< 255,   0, 255
#define ILI9341_YELLOW      0xFFE0  ///< 255, 255,   0
#define ILI9341_WHITE       0xFFFF  ///< 255, 255, 255
#define ILI9341_ORANGE      0xFD20  ///< 255, 165,   0
#define ILI9341_GREENYELLOW 0xAFE5  ///< 173, 255,  41
#define ILI9341_PINK        0xFC18  ///< 255, 130, 198

// Version: Roma Kuzmin 06.09.2019

#define TFT_DC   PA4
#define TFT_CS   PB1
#define TFT_MOSI PA7
#define TFT_CLK  PA5
#define TFT_RST  PB0
#define TFT_MISO PA6

#define FOUR_DAYS 345600
#define ONE_DAY   86400

#define UTC 3

unsigned long currentMillis;

unsigned long showInterval = 0;    
unsigned long sleepInterval = 0;    
unsigned long saveInterval = 0;    
unsigned long showDataInterval = 0;
unsigned long showFromEeprom = 0;
unsigned long showTimeInterval = 0;

bool sleepMode = false;
bool disableSleepMode = false;

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// I2C Address
// 0x68 - DS2331 RTC
// 0x50 - 24LC128N
// 0x51 - 24LC128N
// 0x53 - 24LC256N
// I2c = PB7 = SDA 
//       PB6 = SCL
// PA0 - Кнопки
// PC13 - LED on Board

int but = 0;      // Кнопка

byte Display = 0; // Display for start

DS3231 Clock;
RTClib RTC;

BME280I2C bme; 

BME280I2C::Settings settings(
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::Mode_Forced,
   BME280::StandbyTime_1000ms,
   BME280::Filter_Off,
   BME280::SpiEnable_False,
   0x76 // I2C address. I2C specific.
); 

#define ONE_WIRE_BUS PC14

OneWire oneWire(ONE_WIRE_BUS);
DS18B20 sensor(&oneWire);
TinyGPSPlus gps;  // GPS Serial(1) 9600

I2CEEPROM ee1(0x50); // 24C128 // Temperature DS18B20
I2CEEPROM ee2(0x51); // 24C128
I2CEEPROM ee3(0x53); // 24C256 // BME280

// 24lc128  EEPROM_24LC128  16 kBytes
// 24lc256  EEPROM_24LC256  32 kBytes

byte _sec = -1;
byte _min = -1;
byte _hou = -1;
int  _yer = -1;
byte _mon = -1;
byte _day = -1;

byte _ssH = -1;
byte _srH = -1;
byte _ssM = -1;
byte _srM = -1;

int _tem = -100;
int _hum = -100;
int _bar = -100;
int _alt = -100;

int _maxt = -100;
int _mint = -100;
int _maxb = -100;
int _minb = -100;

int _avgt = -100;
int _avgt1 = -100;
int _avgt2 = -100;

int _avgb = -100;
int _avgb1 = -100;
int _avgb2 = -100;

byte _h[9] = {-100,-100,-100,-100,-100,-100,-100,-100,-100 };

float _cpu  = -100.0;

double _lng = -100.0;
double _lat = -100.0;

int _hdop   = -100;
int _satcnt = -100;
int _cog    = -100;
int _speed  = -100;

bool Century=false;
bool h12;
bool PM;

#define Temperature 0
#define Humidity    1

byte T_H;

// ---------------------------- Structure for save ----------------

struct TempDataStruct {    // Температура (EE1)
  int temperature;
  unsigned long unix_time; 
} temp_data;

struct bme280Struct {  // Данные о давлении,высоте и влажности (EE3)   
  int bar,alt,hum;
  unsigned long unix_time; 
} bme280_data;

struct GPS_Struct {  // Данные с GPS (EE2)
  double lat,lng;
  int alt,cur,speed;
  unsigned long unix_time; 
} gps_data;

double last_lat = 0.0;
double last_lng = 0.0;
   
// ----------------------------------------------------------------

#define MAX 192 // 96 x 2

#define BUT_DELAY 5 // Задержка после нажатия кнопки
  
Average<int> bar(MAX);
int bar_data[MAX];

Average<int> tem(MAX);
int tem_data[MAX];

uint32_t start_bme;

// ------------------------- Setup ----------------------------------------

void setup() {

  Serial1.begin(9600); // GPS

  pinMode(PB10,OUTPUT);     // LED Control
  digitalWrite(PB10,HIGH);  // LED Turn ON    

  pinMode(PA8,OUTPUT);     // GPS Power Control
  digitalWrite(PA8,HIGH);  // GPS Power

  pinMode(PC13,OUTPUT);
  digitalWrite(PC13,LOW); // LED on Board turn ON for visual testing

  // Выключаем не нужные ноги
   pinMode(PB9,INPUT);
   pinMode(PB8,INPUT);
   pinMode(PB5,INPUT);
   pinMode(PB4,INPUT);
   pinMode(PB3,INPUT);
   pinMode(PA15,INPUT);
   pinMode(PA12,INPUT);
   pinMode(PA11,INPUT);   // Check Charging mode is ON or OFF
   
   // Выключили
  
  Wire.begin();

  start_bme = millis();
  
  while(!bme.begin()) {
   if ((millis()- start_bme) > 750) break;  // Не всегда иницилизируется правильно
  }   
  
  sensor.begin(); 
  tft.begin();
  ee1.begin();
  ee3.begin();
  ee2.begin();
  
  bme.setSettings(settings);
  
  i2c_scanner(); 

  //Clock.setClockMode(false);  
  //Clock.setYear(19);
  //Clock.setMonth(11);
  //Clock.setDate(14);
  //Clock.setDoW(DoW);
  //Clock.setHour(15);
  //Clock.setMinute(12);
  //Clock.setSecond(00);

      tft.setTextColor(ILI9341_GREEN); 

      tft.println("");
      
      tft_print(Clock.getHour(h12, PM)); //24-hr
      tft.print(":");
      tft_print(Clock.getMinute());
      tft.print(":");
      tft_print(Clock.getSecond());
      
      tft.println(" ");
      tft.println(" ");
      
      tft_print(Clock.getDate());
      tft.print("/");      
      tft_print(Clock.getMonth(Century));
      tft.print("/");
      tft_print(Clock.getYear()+2000);
      
      tft.println(" ");
      
      
  delay(4000);

  if (digitalRead(PA11) == HIGH) { // External power suply detected - Chraging Li-Ion
    disableSleepMode =  true;
  }
   
  T_H = Temperature;
  Reset_Display(true);   
  tft_show_time();
  
  check_last_pos_from_gps_data(); // Проверим были ли старые данные о GPS позиции
  
  Save_Temp_Data();
  Save_Bar_Data();    

  digitalWrite(PC13,HIGH); 
 
}

// -------------------------------- Main Loop --------------------------------

void loop(void) {


 if (!sleepMode || disableSleepMode) {  
  if (Serial1.available()) {
   gps.encode(Serial1.read());
  }
 }
  
  // BUT1 34xx
  // BUT2 28xx
  // BUT3 22xx
  // BUT4 17xx
  // BUT5 11xx
  // BUT6 6xx

  // DateTime now = RTC.now();
  
  but = analogRead(PA0);

   // check_but(); // Проверка кнопок   
   // BME280  BUT6 BUT5 BUT4 BUT3 BUT2 BUT1

  if (but > 500 && but < 700) { // BUT6 - Switch Display
   delay(BUT_DELAY);
   but = 0;
   if (!sleepMode) { // Когда спим не нажимать эти кнопки
    Display++;
    if (Display > 2) Display = 0;
    Reset_Display(true);   
   }
  }

  if (but > 1100 && but < 1200) { // BUT5 - Disable Sleep Mode
   delay(BUT_DELAY);
   but = 0;
   if (!disableSleepMode) {
    sleepMode = false;
    digitalWrite(PB10,HIGH); // LED On
    digitalWrite(PA8,HIGH);  // GPS On
    disableSleepMode = true;
    Reset_Display(true);   
   }   
  }
  
  if (but > 1700 && but < 1800) { // BUT4 - Enable sleep mode and turn OFF all
   delay(BUT_DELAY);
   but = 0;
   if (disableSleepMode) {
    sleepMode = true;
    digitalWrite(PB10,LOW); // LED Off
    digitalWrite(PA8,LOW);  // GPS Off
    disableSleepMode = false;
   }  
  }

  if (but > 2200 && but < 2300) { // BUT3 -- Show without TH and Temp
   delay(BUT_DELAY);
   but = 0;
   if (!sleepMode) { // Когда спим не нажимать эти кнопки
    Reset_Display(false);   
   }
  }

  if (but > 2800 && but < 2900) { // BUT2 -- Go to sleep mode
   delay(BUT_DELAY);   
   but = 0;
   if (!disableSleepMode && !sleepMode) { // Принужденно выключаем -> засыпаем
    sleepMode = true;
    digitalWrite(PB10,LOW); // LED On
    digitalWrite(PA8,LOW);  // GPS On
   }
  }
  
  if (but > 3400 && but < 3500) { // BUT1 - WeekUp from sleepMode
   delay(BUT_DELAY);
   but = 0;
   if (!disableSleepMode && sleepMode) { // Проснуться показать данные
    sleepMode = false;
    digitalWrite(PB10,HIGH); // LED On
    // digitalWrite(PA8,HIGH);  // GPS On
    // Reset_Display(true);    
    if ((Display == 0 || Display == 2 ) && !sleepMode) {
      Display_0(T_H,true); 
      show_real_data(T_H);
    } else {
       digitalWrite(PA8,HIGH);  // GPS On
       Display_1();
    }
   }  
  }

 currentMillis = millis();
 
  if(currentMillis - sleepInterval > (1000*60*2) ) {  // 2 Minute to sleep
    sleepInterval = currentMillis;  
    if (!sleepMode && !disableSleepMode) {
     sleepMode = true;   
     digitalWrite(PB10,LOW); // LED Off
     digitalWrite(PA8,LOW);  // GPS Off
    }
  }

   if(currentMillis - saveInterval > 1000*60*15 ) {  // 15 Минут Save DATA to EEPROM
    saveInterval = currentMillis;    
    Save_Temp_Data();   
    Save_Bar_Data();    
    if (!sleepMode) {
     save_gps_data(); 
     set_GPS_DateTime();
     check_last_pos_from_gps_data();
    }
   }

  if(currentMillis - showInterval > (1000*5) ) {  // 1 Seconds show Time
    showInterval = currentMillis;      
    tft_show_time(); // Time and Date up
    if (Display == 1 && !sleepMode) {         
     digitalWrite(PA8,HIGH);  // GPS On           
     int gps_c = gps.satellites.value();
     if (gps_c == 0) gps_c = 1;
     draw_sattelite(80,95,70,gps_c,gps.hdop.hdop()); // x,y,radius,sat count,hdop
     Display_Time_SunRise();
     show_GPS_data();
    }
  }

  if(currentMillis - showFromEeprom > (1000*60*5) ) {  // 5 minute show data from eeprom
   showFromEeprom = currentMillis;   
   if ((Display == 0 || Display == 2 ) && !sleepMode) {
     Display_0(T_H,true);  
   } else if (!sleepMode) {
    digitalWrite(PA8,HIGH);  // GPS On
    Display_1();
   }
  }

  if(currentMillis - showDataInterval > (1000*10) ) {  // 10 Seconds show Real Data
    showDataInterval = currentMillis;      
    if ((Display == 0 || Display == 2) && !sleepMode) {
     show_real_data(T_H);
    } else if (!sleepMode) {
      digitalWrite(PA8,HIGH);  // GPS On
      Display_1();
    }
  }

  if(currentMillis - showTimeInterval > 1000 ) {  // 1 Show Date and Time
   showTimeInterval = currentMillis;      
   if (!sleepMode) {  
    tft_show_time();
   }
  }
   
  
}

// ----------------------------- Functions -----------------------------------

void Reset_Display( bool temp_th_show ) {
  
   if (Display == 0 && !sleepMode) { // Weather Bar Temp etc.
    T_H = Temperature;
    tft.fillScreen(ILI9341_BLACK);
    tft.drawRect(20,20,195,150,ILI9341_BLUE); // --- X,Y,W,H (W:195)
    tft.setCursor(0, 0);    
    Display_0(T_H,temp_th_show);    
   } else if (Display == 1 && !sleepMode) { // --- Sattelite
    tft.fillScreen(ILI9341_BLACK);
    tft.setCursor(0, 0);
    Display_1();
   } else if (Display == 2 && !sleepMode) { // --- Humidity
    T_H = Humidity;
    tft.fillScreen(ILI9341_BLACK);
    tft.drawRect(20,20,195,150,ILI9341_BLUE); // X,Y,W,H (W:195)
    tft.setCursor(0, 0);
    Display_0(T_H,temp_th_show);    
   }
  
}

// ------------------------------------------- Functions Displays 0 AND 1 -------------------
void Display_0(byte T__H, bool show_temp_th) {

  show_bar_from_eeprom();
  
  if (show_temp_th) show_temp_from_eeprom(T__H);
  
  show_real_data(T__H);
  Display_Time_SunRise();
  
}

void Display_1(void) {
  
    int gps_c = gps.satellites.value();
    if (gps_c == 0) gps_c = 1;
    draw_sattelite(80,95,70,gps_c,gps.hdop.hdop()); // x,y,radius,sat count,hdop
    Display_Time_SunRise();
    show_GPS_data();    
}

// ------------------------------------------- Draw Sattelite -------------------------------

void draw_sattelite(int mx,int my,int r,byte sat_count,double hdop ) {
  
  int xc,yc; // Sattelite
  int d;     // Result

  int xy[sat_count][2];
  byte i=0;
  bool found;
  int cHDOP = ILI9341_YELLOW;
  int ihdop;
  
  tft.fillCircle(mx,my,r+5,ILI9341_BLACK); // Для спутников      
  tft.drawCircle(mx,my,r+5,ILI9341_BLUE); // Для спутников      

  ihdop = round(hdop / 100.0);
  
  switch(ihdop) {
    case 0:
    cHDOP = ILI9341_BLUE;
    break;
   case 1:
    cHDOP = ILI9341_GREEN;
    break;
   case 2:
    cHDOP = ILI9341_RED;
    break;   
   case 3:
    cHDOP = ILI9341_YELLOW;
    break;
   default:
    cHDOP = ILI9341_MAROON;
  }
   
  do {
   xc = random(35,190);
   yc = random(35,150);
   d = int(sqrt(sq(xc-mx)+sq(yc-my))); 
   found = false;
   
   if (i==0 && d <= (r-5) ) {
    xy[i][0]=xc;
    xy[i][1]=yc;   
    i++;
   } else if (i > 0 && d <= (r-5)) {
    for(byte j=0;j<i;j++) {
      if ((xy[j][0] == xc) || (xy[j][1] == yc))  found = true;
    }  
    if (!found) { 
     xy[i][0]=xc;
     xy[i][1]=yc;   
     i++;
    }
   }
    
  } while (i<sat_count);
    
   
  for(byte i=0;i<sat_count;i++) {
    xc=xy[i][0];
    yc=xy[i][1];            
    //d = int(sqrt(sq(xc-mx)+sq(yc-my))); // Входит ли точка в окружность
    //if ( d <= (r-5) ) 
    if (sat_count == 1) 
     tft.drawCircle(xc,yc,5,cHDOP);
    else 
     tft.fillCircle(xc,yc,5,cHDOP);    
  }

}

// --------------------------------- Functions --------------------------------

int get_alt( void ) {
  
  int alt = 0;
  
  if (gps.altitude.isValid()) {
   alt = gps.altitude.meters();
  }
  
  return(alt); // from GPS
  
}

// --------------------------------- Get Data from BME 280 ------------------------------------

int get_bar( void ) {
  
  float pres = bme.pres();
  return(int(pres/133.3));
}

int get_hum() {
  
  float hum = bme.hum();
  return(int(hum));
  
}
// ------------------- Get Temperature from DS18B20 -------------------------------------------

int get_tem( void ) {

   uint32_t start;
  
   sensor.setResolution(4);
   sensor.requestTemperatures();
   start = millis();
   while(!sensor.isConversionComplete()) {
     if ((millis()- start ) > 750) break;     
   }   
   return(sensor.getTempC());  
}

// ------------------------------- Show Temperature from EEPROM -------------------------------

void show_temp_from_eeprom( byte temp_hum ) {  // --- temp_hum = 0 Temperature
                                               // --- temp_hum = 1 Humidity 
  #define MaxMin 1
  
  int current_position;
  int avg,avg1,avg2;  
  int x_pos = 213; // Start 22 + Tottal 192;
  int show_pos = 0;
  int h;
  
  Average<int> tempData(MAX); // Вычисление максимального и минимального значения
  int tempArray[MAX]; 
  
  byte xL[192],yL[192],k=0; // Точки для рисования
  
  unsigned long unix__time[MAX];  
  unsigned long EEPROM_POS = 0;

 DateTime now = RTC.now();
 current_position = (now.unixtime()/1800)%192;  

 if (temp_hum == 0) { // --- Temperature 0
  for(byte j = 0;j < MAX; j++) {           
    byte* pp = (byte*)(void*)&temp_data; 
    for (unsigned int i = 0; i < sizeof(temp_data); i++)
      *pp++ = ee1.read(EEPROM_POS++); 
       if ((now.unixtime() - temp_data.unix_time) <= FOUR_DAYS) {
        tempArray[j] = temp_data.temperature;
        unix__time[j] = temp_data.unix_time;
        tempData.push(temp_data.temperature);
       } else {
        tempArray[j] = -100; // No Data
        unix__time[j] = -100; // No Data        
       }      
     }
 } else if (temp_hum == 1) {  // --- Humidity
  for(byte j = 0;j < MAX; j++) {           
    byte* pp = (byte*)(void*)&bme280_data; 
    for (unsigned int i = 0; i < sizeof(bme280_data); i++)
      *pp++ = ee3.read(EEPROM_POS++); // EE3
      
       if ((now.unixtime() - bme280_data.unix_time) <= FOUR_DAYS) {
        tempArray[j]= bme280_data.hum;
        unix__time[j] = bme280_data.unix_time;
        tempData.push(bme280_data.hum);
       } else {
        tempArray[j] = -100; // No Data
        unix__time[j] = -100; // No Data        
       }      
     }
  }
   
      tft.setTextSize(1);     
      
      tft.setCursor(220,20);  // X,Y Up Temp Max
      _maxt = erase_show_data_int(_maxt,tempData.maximum()+MaxMin,ILI9341_GREEN,' ');

      avg = (tempData.minimum()-MaxMin + tempData.maximum()+MaxMin ) / 2;      
      tft.setCursor(220,90);  // X,Y Down Temp Min
      _avgt = erase_show_data_int(_avgt,avg,ILI9341_GREEN,' ');

      avg1 = (avg + tempData.maximum()+MaxMin)/2;      
      tft.setCursor(220,55);  // X,Y Down Temp Min
      _avgt1 = erase_show_data_int(_avgt1,avg1,ILI9341_GREEN,' ');

      avg2 = (avg + tempData.minimum()-MaxMin) / 2;      
      tft.setCursor(220,125);  // X,Y Down Temp Min
      _avgt2 = erase_show_data_int(_avgt2,avg2,ILI9341_GREEN,' ');

      tft.setCursor(220,160);  // X,Y Down Temp Min
      _mint = erase_show_data_int(_mint,tempData.minimum()-MaxMin,ILI9341_GREEN,' ');
         
  
   for(int j=0;j<MAX;j++) { // Show Temperature Data
    if(tempArray[current_position] != -100) {

     if (show_pos <= 160) {
      show_time_scale(unix__time[current_position],show_pos); // 0,20,40,60,80,100,120,140,160
      show_pos++;  
     }
     
     h = map(tempArray[current_position],tempData.minimum()-MaxMin,tempData.maximum()+MaxMin,1,146);
     
     //tft.drawFastVLine(x_pos,22,146,ILI9341_BLACK);
     //tft.drawFastVLine(x_pos,(146+22)-h,h,ILI9341_GREEN); // Рисовали линии.
     
     xL[k] = x_pos;
     yL[k] = (146+22)-h;
     k++;
     
     x_pos--;         
    } else { // Если нет данных на эти дни
     x_pos--;
     show_pos++;
    }
    if (current_position == 0) current_position = 191;
    else current_position--;
   } 
   
   for(byte j=0;j<k;j++) {
    if ((j+1) < k) tft.drawLine(xL[j],yL[j],xL[(j+1)],yL[(j+1)],ILI9341_GREEN); // Рисуем График
   }
}

// -------------------------- Time Scale ----------------------------------

void show_time_scale(unsigned long uTime, int show_pos ) {

      int Y=173;
      int X[9];
      int Apos;
            
      X[8] = 21;
      X[7] = 43;
      X[6] = 65;
      X[5] = 87;
      X[4] = 110;
      X[3] = 132;
      X[2] = 155;
      X[1] = 177;
      X[0] = 200; 

      if ((show_pos%20)==0) {
       if (show_pos == 0) Apos = 0;
       else Apos = int(show_pos / 20);      
       if (uTime != -100) {
        tft.setCursor(X[Apos],173); 
        tft.setTextSize(1);
        _h[Apos] = erase_show_data_int(_h[Apos],hour(uTime),ILI9341_GREEN,' ');        
       }
      }      
}

// ------------------------------- Save Temperature from DS18B20 to EEPROM --------------------

void Save_Temp_Data( void ) {

  // 96 часов * 60 минут = 5760 Минут
  // 5760 минут / 30 минут = 192 Ячеек
  // (UnixTime / 1800) % 192 = номер ячейки

  DateTime now = RTC.now();
  
  temp_data.unix_time = now.unixtime(); 
  temp_data.temperature = get_tem();

  unsigned long EEPROM_POS;

  EEPROM_POS = ( (now.unixtime()/1800)%192 ) * sizeof(temp_data); // Номер ячейки памяти.

  const byte* p = (const byte*)(const void*)&temp_data;
  
  for (unsigned int i = 0; i < sizeof(temp_data); i++) {
    ee1.write(EEPROM_POS++,*p++);
    delay(10);
  }
}

// ------------------ Function Float -------------------------------------------------------------------

float erase_show_data_float(float w, float in, int COLOR,char add) { // Erase data before display

  int X,Y;
  
  X = tft.getCursorX();
  Y = tft.getCursorY();
  
  if (w == -100.0) {
    tft.setTextColor(COLOR);      
    w = in; 
    tft.print(in);
    if (add != ' ') tft.print(add);
  } else {
    tft.setCursor(X, Y);          
    tft.setTextColor(ILI9341_BLACK);  
    tft.print(w);
    if (add != ' ') tft.print(add);
    w = in;
    tft.setCursor(X, Y);      
    tft.setTextColor(COLOR);  
    tft.print(w);    
    if (add != ' ') tft.print(add);
  }
  
  return(w);
}

// ------------------ Function Double -------------------------------------------------------------------

double erase_show_data_double(double w, double in, int COLOR,char add) { // Erase data before display

  int X,Y;
  
  X = tft.getCursorX();
  Y = tft.getCursorY();
 
  if (w == -100.0) {
    tft.setTextColor(COLOR);      
    w = in; 
    tft.print((double)in,6);
    if (add != ' ') tft.print(add);
  } else {
    tft.setCursor(X, Y);          
    tft.setTextColor(ILI9341_BLACK);  
    tft.print((double)w,6);
    if (add != ' ') tft.print(add);
    w = in;
    tft.setCursor(X, Y);      
    tft.setTextColor(COLOR);  
    tft.print((double)w,6);    
    if (add != ' ') tft.print(add);
  }
  
  return(w); 
}
// --------------------------- Erase Show Data Int ------------------------------------------

int erase_show_data_int(int w, int in, int COLOR,char add) { // Erase data before display

  int X,Y;
  
  X = tft.getCursorX();
  Y = tft.getCursorY();
   
  if (w == -100) {
    tft.setTextColor(COLOR);      
    w = in; 
    tft.print(in);
    if (add != ' ') tft.print(add);
  } else {
    tft.setCursor(X, Y);
    tft.setTextColor(ILI9341_BLACK);  
    tft.print(w);
    if (add != ' ') tft.print(add);
    w = in;
    tft.setCursor(X, Y);      
    tft.setTextColor(COLOR);  
    tft.print(w);    
    if (add != ' ') tft.print(add);
  }
  
  return(w);
}

int show_time_by(int what,int in_data,int cColor) { // _sec = show_time_by(_sec,now.second())
  
  int X,Y;
  
  X = tft.getCursorX();
  Y = tft.getCursorY();
  
  if (what == -1) {
    what = in_data; if (what < 10) tft.print(0); tft.print(what, DEC);
   } else {
    tft.setTextColor(ILI9341_BLACK);  
    if (what < 10) tft.print(0); tft.print(what, DEC);
    tft.setCursor(X, Y);      
    tft.setTextColor(cColor); 
    what = in_data;
    if (what < 10) tft.print(0); tft.print(what, DEC);    
   }

   return(what);
}

// -------------------------- Show Time ------------------------------

void tft_show_time() {

   DateTime now = RTC.now();

   tft.setCursor(0, 0);  
   tft.setTextColor(ILI9341_YELLOW); 
   tft.setTextSize(2);   
   
   _hou = show_time_by(_hou,now.hour(),ILI9341_YELLOW);    tft.print(":");
   _min = show_time_by(_min,now.minute(),ILI9341_YELLOW);  tft.print(":");   
   _sec = show_time_by(_sec,now.second(),ILI9341_YELLOW); 
   
   tft.print(" ");
   
   _day = show_time_by(_day,now.day(),ILI9341_YELLOW);   tft.print("/");   
   _mon = show_time_by(_mon,now.month(),ILI9341_YELLOW); tft.print("/");   
   _yer = show_time_by(_yer,now.year(),ILI9341_YELLOW);
   
}

// --------------------------------- I2C Scanner ------------------------------

void i2c_scanner( void ) {

  byte error, address;
  int nDevices;

  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_GREEN);
  tft.println("Roma Kuzmin 06.09.19");
  tft.println("--------------------");
  tft.println("Scanning I2C ...");
  tft.println("");
 
  nDevices = 0;
  
  for(address = 1; address < 127; address++ ) {
  
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    delay(10);
 
    if (error == 0) {
      tft.setTextColor(ILI9341_GREEN);
      tft.print("I2C device: ");
      tft.setTextColor(ILI9341_BLUE);
      tft.print("0x");
      if (address<16) tft.print("0");
      tft.print(address,HEX);
      tft.println("");
      nDevices++;
     } else if (error==4) {
      tft.setTextColor(ILI9341_RED);      
      tft.print("Error at address 0x");
      if (address<16) tft.print("0");
      tft.println(address,HEX);
     }    
  } 
  
  if (nDevices == 0) {
     tft.setTextColor(ILI9341_RED);            
     tft.println("No I2C devices found.");
  } else {
    tft.println("");
    tft.setTextColor(ILI9341_MAGENTA);              
    tft.println("Done.");
  }
}

// ---------------------------- Sun Rise and Sun Set -------------------------------------

void Display_Time_SunRise(void ) {

  // Moscow 55.740404, 37.619706    
  // Sunrise mySunrise(gps.location.lat(),gps.location.lng(),UTC);

  int t;
  byte h_rise=0;
  byte m_rise=0;
  byte h_set=0;
  byte m_set=0;
  double lat,lng,hdop;
  bool ok = false;
  
  DateTime now = RTC.now();  
  
  hdop = gps.hdop.value()/100.0;

  // Sunrise mySunrise(55.740404,37.619706,UTC); 
  
  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid() && hdop < 5.0) {
   lat = gps.location.lat();
   lng = gps.location.lng();  
   ok = true;
  } else if (last_lng != 0.0 && last_lat != 0.0) {
   lat = last_lat;
   lng = last_lng;
   ok = true;
  }

  if (ok) {
    
    Sunrise mySunrise(lat,lng,UTC);  

    mySunrise.Actual();

    t = mySunrise.Rise(now.month(),now.day()); // Month,Day

      if(t >= 0) {
       h_rise = mySunrise.Hour();
       m_rise = mySunrise.Minute();
      } else { 
       h_rise = 0; 
       m_rise = 0; 
      }    

    t = mySunrise.Set(now.month(),now.day()); // Month,Day

      if(t >= 0) {
       h_set = mySunrise.Hour();
       m_set = mySunrise.Minute();  
      } else { 
       h_set = 0; 
       m_set = 0; 
      }
  }
  
   tft.setTextSize(2);      
   tft.setCursor(15,290);
   tft.setTextColor(ILI9341_YELLOW);    
   tft.print("Sr:");
   tft.setTextColor(ILI9341_GREEN);       
   _srH = show_time_by(_srH,h_rise,ILI9341_GREEN);
   tft.print(":");
   _srM = show_time_by(_srM,m_rise,ILI9341_GREEN);   
   tft.print(" "); 
   tft.setTextColor(ILI9341_YELLOW);       
   tft.print("Ss:");
   tft.setTextColor(ILI9341_RED);          
   _ssH = show_time_by(_ssH,h_set,ILI9341_RED);
   tft.print(":");
   _ssM = show_time_by(_ssM,m_set,ILI9341_RED);
   
}

// ---------------- Show Real Data -----------------------------------
 
void show_real_data( byte T__H ) {

   int t_h_color;
   float cpu_clock_temp = 0.0;
   
   tft.setTextSize(3);   
   
   #define SM 2
   
   tft.setCursor(13,185+SM);
   tft.setTextColor(ILI9341_YELLOW); 
   tft.print("T:");

     if (T__H == 0) t_h_color = ILI9341_GREEN;
     else  t_h_color = ILI9341_BLUE;   
     _tem = erase_show_data_int(_tem,get_tem(),t_h_color,'C');
   
   tft.setCursor(125,185+SM);
   tft.setTextColor(ILI9341_YELLOW);    
   tft.print("H:");

     if (T__H == 0) t_h_color = ILI9341_BLUE;
     else  t_h_color = ILI9341_GREEN;   
     _hum = erase_show_data_int(_hum,get_hum(),t_h_color,'%');

   tft.setCursor(13,220+SM);   
   tft.setTextColor(ILI9341_YELLOW);    
   tft.print("B:");
   
   _bar = erase_show_data_int(_bar,get_bar(),ILI9341_MAGENTA,' ');
   
   tft.setCursor(125,220+SM);   
   tft.setTextColor(ILI9341_YELLOW);    
   tft.print("A:");
   
   _alt = erase_show_data_int(_alt,get_alt(),ILI9341_RED,' ');
      
   tft.setCursor(13,255+SM);
   tft.setTextColor(ILI9341_YELLOW);    
   tft.print("CPU:");

   // cpu_clock_temp = Clock.getTemperature();  
   
   cpu_clock_temp = bme.temp();
   
   //if (cpu_clock_temp > 99.0) {
   // for (int i=0;i<5;i++) {
   //   cpu_clock_temp = Clock.getTemperature();
   //   if (cpu_clock_temp < 99.0) break;
   // }
   //}
   
   _cpu = erase_show_data_float(_cpu,cpu_clock_temp,ILI9341_OLIVE,'C');

   if (!disableSleepMode) {
    tft.fillRect(200,250,25,30,ILI9341_WHITE);   
    tft.setCursor(205,255);
    tft.setTextColor(ILI9341_BLUE);    
    tft.print(Display);
   }

   if (disableSleepMode) {
    tft.fillTriangle(200,280,212,250,225,280,ILI9341_RED);
    tft.setCursor(205,258);
    tft.setTextColor(ILI9341_WHITE);    
    tft.print("!");
   }   
   
}

// ---------------------------------- GPS Data show -----------------------------
 
void show_GPS_data( void ) {

   //double lng = 33.654321;
   //double lat = 56.123456;

   double lat = 0.0;
   double lng = 0.0;
   double hdop;
   double speed;
   int gps_count = 0;
   double course = 0.0;
   
  hdop = gps.hdop.value()/100.0;
  gps_count =  gps.satellites.value();

  // --------------- Получили первую точку старта ---------------------------

  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid() && hdop < 5.0) {
   speed = gps.speed.kmph();
   lat = gps.location.lat();
   lng = gps.location.lng();   
   course = gps.course.deg();
  }
  
   tft.setTextSize(2);   

   // --- HDOP
   tft.setCursor(165,25);
   tft.setTextColor(ILI9341_YELLOW); 
   tft.print("HDOP:");
   tft.setCursor(175,50);
   _hdop = erase_show_data_int(_hdop,hdop,ILI9341_GREEN,' ');
   
   // --- Sat Count
   tft.setCursor(165,75);   
   tft.setTextColor(ILI9341_YELLOW);    
   tft.print("SatCn:");   
   tft.setCursor(175,100);   
   _satcnt = erase_show_data_int(_satcnt,gps_count,ILI9341_GREEN,' ');
   
   // --- COG
   tft.setCursor(165,125);   
   tft.setTextColor(ILI9341_YELLOW);    
   tft.print("COG:");   
   tft.setCursor(175,150);   
   _cog = erase_show_data_int(_cog,course,ILI9341_GREEN,' ');
   
   // -- LAT,LNG

   tft.setTextSize(3);   
   
   tft.setCursor(2,185);
   tft.setTextColor(ILI9341_YELLOW); 
   tft.print("Lat:");   
   _lat = erase_show_data_double(_lat,lat,ILI9341_MAGENTA,' ');
         
   tft.setCursor(2,215);
   tft.setTextColor(ILI9341_YELLOW);    
   tft.print("Lng:");
   _lng = erase_show_data_double(_lng,lng,ILI9341_MAGENTA,' ');
   
   tft.setCursor(2,250);
   tft.setTextColor(ILI9341_BLUE);    
   tft.print("Speed:");   
   _speed = erase_show_data_int(_speed,speed,ILI9341_RED,' ');

   if (!disableSleepMode) {
    tft.fillRect(200,250,25,30,ILI9341_WHITE);   
    tft.setCursor(205,255);
    tft.setTextColor(ILI9341_BLUE);    
    tft.print(Display);
   }

   if (disableSleepMode) {
    tft.fillTriangle(200,280,212,250,225,280,ILI9341_RED);
    tft.setCursor(205,258);
    tft.setTextColor(ILI9341_WHITE);    
    tft.print("!");
   }   
   
}

// ---------------------------------- Show_Bar_From_EEProm ----------------------

void show_bar_from_eeprom( void ) {

   int maxB,minB;
   int x_pos = 213;
   int h;
   int pmax;
   int pmin;
   double scale;

   int barData_maximum;
   int barData_minimum;
   
   int max_B;
   int min_B;
   
   Average<int> barData(MAX); // Вычисление максимального и минимального значения
  
   int barArray[MAX]; 
   unsigned long unix__time[MAX];  
  
   unsigned long EEPROM_POS = 0;
   
   DateTime now = RTC.now();  

   int current_position = (now.unixtime()/1800)%192;  

   for(byte j = 0;j < MAX; j++) {           
    byte* pp = (byte*)(void*)&bme280_data; 
    for (unsigned int i = 0; i < sizeof(bme280_data); i++)
      *pp++ = ee3.read(EEPROM_POS++); // EE3
      
       if ((now.unixtime() - bme280_data.unix_time) <= FOUR_DAYS && bme280_data.bar > 0) {
        barArray[j] = bme280_data.bar*10;
        unix__time[j] = bme280_data.unix_time;
        barData.push(bme280_data.bar*10);
       } else {
        barArray[j] = -100; // No Data
        unix__time[j] = -100; // No Data        
       }      
     }

   maxB = barData.maximum();
   minB = barData.minimum();

   barData_maximum = barData.maximum()/10;
   barData_minimum = barData.minimum()/10;

   max_B = barData_maximum;
   min_B = barData_minimum;
   
   tft.setTextSize(1);
         
   tft.setCursor(0,20);  // Up Max Bar
   _maxb = erase_show_data_int(_maxb,barData_maximum,ILI9341_GREEN,' ');

   tft.setCursor(0,90);  // Down Min Bar   
   _avgb = erase_show_data_int(_avgb,int((max_B+min_B)/2),ILI9341_GREEN,' ');

   tft.setCursor(0,55);  // Down Min Bar   
   _avgb1 = erase_show_data_int(_avgb1,int((max_B+_avgb)/2),ILI9341_GREEN,' ');

   tft.setCursor(0,125);  // Down Min Bar   
   _avgb2 = erase_show_data_int(_avgb2,int((min_B+_avgb)/2),ILI9341_GREEN,' ');

   tft.setCursor(0,160);  // Down Min Bar
   _minb = erase_show_data_int(_minb,barData_minimum,ILI9341_GREEN,' ');
  
  pmin = minB;
  pmax = maxB;

  if ((pmax % 5) > 0) pmax=((pmax /5)+1)*5;
  pmin=(pmin/10)*10;

  scale=(pmax-pmin)/146.0; // --- 146-Max Heght of Line
  
  if (scale>50) scale=100;
  else if (scale>20) scale=50;
  else if (scale>10) scale=20;
  else if (scale>5) scale=10;
  else if (scale>2) scale=5;
  else if (scale>1) scale=2;
  else scale=1;

    for(int j=0;j<MAX;j++) { // Show Bar Data
     if(barArray[current_position] != -100 && barArray[current_position] >= minB && barArray[current_position] <= maxB) {       
      // --- h = map(barArray[current_position],barData.minimum(),barData.maximum(),1,146);         
      tft.drawFastVLine(x_pos,22,146,ILI9341_BLACK);       
      h = (barArray[current_position]-pmin)/scale;
      if (h==0) h=1;
      tft.drawFastVLine(x_pos,(146+22)-h,h,ILI9341_MAGENTA); // Рисовали линии.              
      x_pos--;         
    } else {
      tft.drawFastVLine(x_pos,22,146,ILI9341_BLACK); // Стираем данные
      x_pos--; // Пропускаем дни
    }
    if (current_position == 0) current_position = 191;
    else current_position--;
   } 
   
}

// --------------------------------- Save_Bar_Data --------------------------------

void Save_Bar_Data( void ) {

  unsigned long EEPROM_POS;
  
  DateTime now = RTC.now();  

  bme280_data.alt = get_alt(); 
  bme280_data.hum = get_hum();
  bme280_data.bar = get_bar();
  bme280_data.unix_time = now.unixtime(); 
  
  EEPROM_POS = ((now.unixtime()/1800)%192) * sizeof(bme280_data); // Номер ячейки памяти.

  const byte* p = (const byte*)(const void*)&bme280_data;
  for (unsigned int i = 0; i < sizeof(bme280_data); i++) {
    ee3.write(EEPROM_POS++,*p++);
    delay(10);
  }

}

// -------------------- Проверка кнопок - какие данные мы с них получаем ----

void check_but() {
 
  int page = 0;
  int but = 0;
  
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
  
  while (true) {
    but = analogRead(PA0);
    tft.println(but);
    delay(1000);
    page++;
    if (page > 10) {
     page=0;
     tft.fillScreen(ILI9341_BLACK);
     tft.setCursor(0, 0);    
    }
  }
}

// ---------------- set time and Date from GPS ---------------------------

void set_GPS_DateTime( void ) {

  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) { // Установка времени 

    Clock.setClockMode(false);  
    Clock.setYear(gps.date.year()-2000);
    Clock.setMonth(gps.date.month());
    Clock.setDate(gps.date.day());
    Clock.setHour(gps.time.hour()+UTC);
    Clock.setMinute(gps.time.minute());
    Clock.setSecond(gps.time.second());
  
  }

}

// ----------------------------- Save GPS data to EEProm 2 -----------------------

void save_gps_data( void ) {

 double hdop;
 
 hdop = gps.hdop.value()/100.0;

 DateTime now = RTC.now();
 
 if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid() && hdop < 5.0) { // Если данные верны
    
  gps_data.unix_time = now.unixtime(); 
  gps_data.lat = gps.location.lat();
  gps_data.lng = gps.location.lng();
  gps_data.alt = gps.altitude.meters();
  gps_data.cur = gps.course.deg();
  gps_data.speed = gps.speed.kmph();

  unsigned long EEPROM_POS;

  EEPROM_POS = ( (now.unixtime()/1800)%192 ) * sizeof(gps_data); // Номер ячейки памяти.

  const byte* p = (const byte*)(const void*)&gps_data;
  
  for (unsigned int i = 0; i < sizeof(gps_data); i++) {
    ee2.write(EEPROM_POS++,*p++);
    delay(10);
  }

 }
}

// --------------- Get Last Position from GPS Data ---------------------------

void check_last_pos_from_gps_data( void ) {

   unsigned long EEPROM_POS = 0;
   
   last_lat = 0.0; // Глобальные данные LAT
   last_lng = 0.0; // Глобальные данные LNG
   
   DateTime now = RTC.now();  

   for(byte j = 0;j < MAX; j++) {           
    byte* pp = (byte*)(void*)&gps_data; 
    for (unsigned int i = 0; i < sizeof(gps_data); i++)
      *pp++ = ee2.read(EEPROM_POS++); // EE2
            
       if ((now.unixtime() - gps_data.unix_time) <= ONE_DAY) {
        last_lat = gps_data.lat;
        last_lng = gps_data.lng;
        break;
       }
     }
 
 }

void tft_print(int data) {
  if (data < 10) {
    tft.print("0");
    tft.print(data,DEC);
  } else {
    tft.print(data,DEC);
  }
}
      
