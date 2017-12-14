//#include <FRAM_MB85RC_I2C.h>


// ToyotaOBD1_Reader
// In order to read the data from the OBD connector, short E1 + TE2. then to read the data connect to VF1.
// Note the data line output is 12V - connecting it directly to one of the arduino pins might damage (proabably) the board
// This is made for diaply with an OLED display using the U8glib - which allow wide range of display types with minor adjusments.
// Many thanks to GadgetFreak for the greate base code for the reasding of the data.
// If you want to use invert line - note the comments on the MY_HIGH and the INPUT_PULLUP in the SETUP void.
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341_STM.h"
#include "CPMonoP9pt7b.h"
#include "Orbitron11pt7b.h"
#include "CPMonoL97b.h"
#include "CPMono_L6pt7b.h"
#include "CPMono_L13pt7b.h"
int charWd = 10;
int charHt = 16;
int charYoffs = 0;
#include <MD_KeySwitch.h>
#define VREF_MEASURED 3.32              //Измеренное опорное напряжение с 3.3В стабилизатора
#define TT_PIN_DIVIDER 3.13
// выбрать один вариант логирования
#define LOGGING_MINIMAL //запись минимума данных
//#define LOGGING_FULL    //Запись на SD всех данных
//#define LOGGING_DEBUG    //Запись на SD всех данных + статистику расхода и пробега
//#define SECOND_O2SENS  // Включение 2го сенсора кислорода для V движков
#define DEBUG_OUTPUT true // for debug option - swith output to Serial

//DEFINE пинов под входы-выходы
//#define LED_PIN         33  //встроенный светодиод
//#define OX_PIN          3 //D3-PB0 для сенсора кислорода
//#define TT_PIN          8 //D8-PA3 для сенсора ТТ АКПП
#define ENGINE_DATA_PIN 18 //D18-PB5-EXTI5 VF1 PIN
//#define TOGGLE_BTN_PIN  18 // D4 screen change PIN
#define TFT_CS         7
#define TFT_DC         12
Adafruit_ILI9341_STM tft = Adafruit_ILI9341_STM(TFT_CS, TFT_DC); // Use hardware SPI

//DEFINE констант расходомера
#define Ls 0.004020653 //производительсность форсунки литров в секунду // базовый 0.004 или 240cc
#define Ncyl 6 //кол-во цилиндров

//DEFINE OBD READER
#define  MY_HIGH  HIGH //LOW    // I have inverted the Eng line using an Opto-Coupler, if yours isn't then reverse these low & high defines.
#define  MY_LOW   LOW //HIGH
#define  TOYOTA_MAX_BYTES  24
#define OBD_INJ 1 //Injector pulse width (INJ)
#define OBD_IGN 2 //Ignition timing angle (IGN)
#define OBD_IAC 3 //Idle Air Control (IAC)
#define OBD_RPM 4 //Engine speed (RPM)
#define OBD_MAP 5 //Manifold Absolute Pressure (MAP)
#define OBD_ECT 6 //Engine Coolant Temperature (ECT)
#define OBD_TPS 7 // Throttle Position Sensor (TPS)
#define OBD_SPD 8 //Speed (SPD)
#define OBD_OXSENS 9 // Лямбда 1

#ifdef SECOND_O2SENS
#define OBD_OXSENS2 10 // Лямбда 2 на V-образных движка. У меня ее нету.
#endif
float OBDDATA[11], OLDOBDDATA[11];
bool OBDFLAG[11], OLDOBDFLAG[11];

#define FONT_X 18
#define FONT_Y 18

//MD_KeySwitch S(TOGGLE_BTN_PIN, HIGH);
byte CurrentDisplayIDX = 1, TT_last = 0, TT_curr = 0;
float total_fuel_consumption = 0, trip_fuel_consumption = 0;
float trip_avg_fuel_consumption;
float cycle_obd_inj_dur = 0;
float cycle_trip = 0;
float trip_inj_dur = 0;
float total_inj_dur_ee = 0;
float current_trip = 0;
float total_trip = 0;
float total_avg_consumption;
float total_avg_speed;
float trip_avg_speed;
unsigned long current_time = 0;
unsigned long total_time = 0;
unsigned long t;
unsigned long last_log_time = 0;
unsigned long odometer;
bool flagNulSpeed = true, isActive = false;
unsigned int OX, TT;

volatile uint8_t ToyotaNumBytes, ToyotaID, ToyotaData[TOYOTA_MAX_BYTES];
volatile uint16_t ToyotaFailBit = 0;

boolean LoggingOn = false; // dfeine connection flag and last success packet - for lost connection function.


void setup() {
  Serial.begin(9600);
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);
  tft.drawCircle(140, 120, 180, ILI9341_YELLOW);
  //EEPROM.get(104, total_trip);
  //EEPROM.get(108, total_time);
  //EEPROM.get(200, odometer);
  //EEPROM.get(204, total_inj_dur_ee);
  /*
    S.begin();
    S.enableDoublePress(true);
    S.enableLongPress(true);
    S.enableRepeat(false);
    S.enableRepeatResult(false);
    S.setDoublePressTime(300);
    S.setLongPressTime(2000);
  */
  //u8g.setFont(u8g_font_profont15r);
  /*if (DEBUG_OUTPUT) {
    Serial.println("system Started");
    Serial.print("Read float from EEPROM: ");
    Serial.println(total_trip, 3);
    Serial.println(total_time, 3);
    Serial.println(odometer, 1);
    Serial.println(total_inj_dur_ee, 3);
    }*/


  //  writeHeader();

  pinMode(ENGINE_DATA_PIN, INPUT); // VF1 PIN
  // pinMode(LED_PIN, OUTPUT);
  //  pinMode(TT_PIN, INPUT_ANALOG);
  //  pinMode(OX_PIN, INPUT_ANALOG);
  attachInterrupt(ENGINE_DATA_PIN, ChangeState, CHANGE); //setup Interrupt for data line
  //  pinMode(TOGGLE_BTN_PIN, INPUT);           // кнопка СЛЕД. ЭКРАН
  CurrentDisplayIDX = 1; // set to display 1
  //drawScreenSelector();
  //Расходомер
  t = millis();
  last_log_time = millis();

  //initscreen();
  delay(100);
} // END VOID SETUP

void loop(void) {
  unsigned long new_t;
  unsigned int diff_t;
  /*  switch (S.read())
    {
    case MD_KeySwitch::KS_NULL: break;
    case MD_KeySwitch::KS_PRESS:    ent(); break;
    case MD_KeySwitch::KS_DPRESS:   {
        if (LoggingOn == false) LoggingOn = true; else LoggingOn = false;
      } break;
    case MD_KeySwitch::KS_LONGPRESS: cleardata(); break;
    case MD_KeySwitch::KS_RPTPRESS: break;
    }
  */
  if (ToyotaNumBytes > 0)  {    // if found bytes
    getOBD();
    new_t = millis();
    if (new_t > t && OBDDATA[OBD_RPM] > 100 ) {// выполняем только когда на работающем двигателе
      diff_t = new_t - t;
      cycle_obd_inj_dur = OBDDATA[OBD_RPM]  * Ncyl * (float)diff_t  * OBDDATA[OBD_INJ] / 120000; //Время открытых форсунок за 1 такт данных. В МС
      //ОБ/М           ОБ/С
      //форсунка срабатывает раз в 2 оборота КВ
      //6форсунок в с
      //время цикла мс в с. Получаем кол-во срабатываний за время цикла. Умножаем на время открытия форсунки, получаем время открытия 6 форсунок В МИЛЛИСЕКУНДАХ

      trip_inj_dur += cycle_obd_inj_dur;                                                              //Время открытых форсунок за поездку        В МС
      total_inj_dur_ee += cycle_obd_inj_dur;                                                           //Время открытых форсунок за все время. EEPROM    В МС

      trip_fuel_consumption = trip_inj_dur  * Ls / 1000;    //потребление топлива за поездку в литрах
      total_fuel_consumption = total_inj_dur_ee  * Ls / 1000;  //потребление топлива за все время. Из ЕЕПРОМ в литрах


      cycle_trip = (float)diff_t  * OBDDATA[OBD_SPD] / 3600000;  //расстояние пройденное за такт обд данных
      current_trip += cycle_trip;  //Пройденное расстояние с момента включения. В КМ
      total_trip += cycle_trip;    //Полное пройденное расстояние. EEPROM. В КМ
      odometer += cycle_trip;       //электронный одометр. Хранится в еепром и не стирается кнопкой

      current_time += diff_t;             //Время в пути в миллисекундах с момента включения
      total_time += diff_t;                         //полное пройденное время в миллисекундах лимит ~49 суток. EEPROM

      trip_avg_speed = current_trip  * 3600000 / (float)current_time ;       //средняя скорость за поездку
      total_avg_speed = total_trip  * 3600000 / (float)total_time;           // средняя скорость за все время. км\ч

      trip_avg_fuel_consumption = 100 * trip_fuel_consumption / current_trip; //средний расход за поездку
      total_avg_consumption = 100 * total_fuel_consumption / total_trip;      //среднее потребление за все время - Л на 100км

      t = new_t;//тест

      //  if (LoggingOn == true) logData();         //запись в лог данных по двоному нажатию на кнопку
      //updateEepromData();   //запись данных при остановке

      if (millis() - last_log_time > 180000) {        //Запись данных в EEPROM каждые 3 минуты. Чтобы не потерять данные при движении на трассе
        //        rtc.eeprom_write(104, total_trip);
        //      rtc.eeprom_write(108, total_time);
        //   rtc.eeprom_write(200, odometer);
        //   rtc.eeprom_write(204, total_inj_dur_ee);
        last_log_time = millis();
      }
    }
    //drawScreenSelector(); // draw screen
    drawAllData();
   //mainscreen();

    ToyotaNumBytes = 0;     // reset the counter.
  } // end if (ToyotaNumBytes > 0)

  if (millis() % 50 == 0 && LoggingOn == true && CurrentDisplayIDX == 6) { //каждые 50мс, когда включено логирование и выбран экран с флагами(!)
    //    OX = analogRead(OX_PIN);
    if (OX < 400) { //исключаю ложные показание > ~1.3В
      //      file.write(';');
      //      file.print(((float)OX * VREF_MEASURED) / 1024, 3 );
      //      file.println();
    }
  }

  if (millis() % 500 == 0) {  //каждые пол секунды читаем состояние АКПП
    //    TT = analogRead(TT_PIN);
    //  TT_curr = (int)(TT * VREF_MEASURED * TT_PIN_DIVIDER / 1024  + 0.5);
    //    drawScreenSelector();
    // if (TT_last != TT_curr) {
    //      drawScreenSelector();
    //  TT_last = TT_curr;
    //}
    //Serial.println((float)TT * VREF_MEASURED / 1024 * 3.13, 3);
    // Serial.println((int)(TT * VREF_MEASURED / 1024 * 3.13+0.5));
  }


}

//  if (millis() % 5000 < 50) autoscreenchange();      // ротация экранов

void updateEepromData() {
  if (OBDDATA[OBD_SPD] == 0 && flagNulSpeed == false)  {   //Запись данных в еепром когда остановка авто
    //    EEPROM.put(104, total_trip);
    //    EEPROM.put(108, total_time);
    //    EEPROM.put(200, odometer);
    //EEPROM.put(204, total_inj_dur_ee);
    flagNulSpeed = true;                                  //запрет повторной записи
    last_log_time = millis();                             //чтобы не писать лишний раз
  }
  if (OBDDATA[OBD_SPD] != 0) flagNulSpeed = false;     //начали двигаться - разрешаем запись
}

void cleardata() {
  int i;
  for (i = 104; i <= 112; i++) {
    //    EEPROM.update(i, 0);
  }
  for (i = 204; i <= 208; i++) {
    //    EEPROM.update(i, 0);
  }
  //  EEPROM.get(104, total_trip);
  //  EEPROM.get(108, total_time);
  //  EEPROM.get(204, total_inj_dur_ee);


}

/*
  void writeHeader() {
  #ifdef LOGGING_FULL
  file.print(F(";OX_RAW;INJ TIME;IGN;IAC;RPM;MAP;ECT;TPS;SPD;VF1;OX;ASE;COLD;KNOCK;OPEN LOOP;Acceleration Enrichment;STARTER;IDLE;A/C;NEUTRAL;AVG SPD;LPK_OBD;LPH_OBD;TOTAL_OBD;AVG_OBD;CURR_OBD;CURR_RUN;total_trip"));
  #endif
  #ifdef LOGGING_DEBUG
  file.print(F(";OX_RAW;INJ TIME;IGN;IAC;RPM;MAP;ECT;TPS;SPD;VF1;OX;AVG SPD;LPK_OBD;LPH_OBD;TOTAL_OBD;AVG_OBD;CURR_OBD;CURR_RUN;total_trip"));
  #endif
  #ifdef LOGGING_MINIMAL
  file.print(F(";OX_RAW;INJ TIME;IGN;IAC;RPM;MAP;ECT;TPS;SPD;VF1;OX;LPH_OBD;TT"));
  #endif
  file.println();
  file.sync();
  }
*/
//обнуление данных

/*
  void logData() {
  file.print(float(millis()) / 60000, 3); file.write(';')   ; file.write(';');
  file.print(getOBDdata(OBD_INJ)); file.write(';'); file.print(getOBDdata(OBD_IGN));  file.write(';');  file.print(getOBDdata(OBD_IAC));  file.write(';');
  file.print(getOBDdata(OBD_RPM)); file.write(';'); file.print(getOBDdata(OBD_MAP));  file.write(';'); file.print(getOBDdata(OBD_ECT));  file.write(';');
  file.print(getOBDdata(OBD_TPS)); file.write(';');  file.print(getOBDdata(OBD_SPD));  file.write(';'); file.print(getOBDdata(OBD_OXSENS)); file.write(';'); file.print(getOBDdata(20)); file.write(';');
  #ifdef LOGGING_FULL
  file.print(getOBDdata(11)); file.write(';'); file.print(getOBDdata(12)); file.write(';'); file.print(getOBDdata(13)); file.write(';'); file.print(getOBDdata(14)); file.write(';');
  file.print(getOBDdata(15)); file.write(';'); file.print(getOBDdata(16)); file.write(';'); file.print(getOBDdata(17)); file.write(';'); file.print(getOBDdata(18)); file.write(';');
  file.print(getOBDdata(19)); file.write(';');
  #endif
  #ifdef  LOGGING_DEBUG
  file.print(total_avg_speed); file.write(';');                                                                   //AVG_SPD       ok
  file.print(100 / getOBDdata(OBD_SPD) * (getOBDdata(OBD_INJ) * getOBDdata(OBD_RPM)*Ls * 0.18)); file.write(';');  //LPK_OBD      ok
  #endif

  file.print(getOBDdata(OBD_INJ) * getOBDdata(OBD_RPM)*Ls * 0.18); file.write(';');                                //LPH_OBD    ok
  file.print((float)TT * VREF_MEASURED  * TT_PIN_DIVIDER / 1024, 2); file.write(';');         //ТТ пин напряжение
  #ifdef LOGGING_DEBUG
  file.print(total_fuel_consumption); file.write(';');   //TOTAL_OBD     ok
  file.print(trip_avg_fuel_consumption); file.write(';');   //!AVG_OBD
  file.print(trip_fuel_consumption); file.write(';');  //!CURR_OBD
  file.print(current_trip);   file.write(';');    //CURR_RUN ok
  file.print(total_trip); file.write(';');//RUN_TOTAL      ok
  #endif
  file.println();
  file.sync();
  }
*/

/*
  void drawScreenSelector(void) {
  if (CurrentDisplayIDX == 1) {DrawCurrentFuelConsuption(); isActive=false;}
  else if (CurrentDisplayIDX == 2) {DrawTotalFuelConsuption(); isActive=false;}
  else if (CurrentDisplayIDX == 3) {drawTimeDistance(); isActive=false;}
  else if (CurrentDisplayIDX == 4) {drawTripTimeDistance(); isActive=false;}
  else if (CurrentDisplayIDX == 5) {drawAllData(); isActive=false;}
  else if (CurrentDisplayIDX == 6) {drawExtraData(); isActive=false;}
  } // end drawScreenSelector()
*/



void mainscreen(void) {
  tft.setFont(&CPMono_v07_Light6pt7b );
  if (isActive == false) {                      //если первый раз экран появился то рисуются подписи
    tft.setCursor(165, 13);
    tft.print("Instant");
    tft.setCursor(236, 27);
    tft.print("Liter");
    tft.setCursor(236, 40);
    tft.print("100km");
    tft.fillRect(235, 29, 44, 2, 0x07E0);

    tft.setCursor(165, 232);
    tft.print("Average trip");
    tft.setCursor(236, 208);
    tft.print("Liter");
    tft.setCursor(236, 221);
    tft.print("100km");
    tft.fillRect(235, 210, 44, 2, 0x07E0);
    isActive == true;                         // если экран не меняется то подписи не обновляются
  }

  tft.setFont(&CPMono_v07_Plain13pt7b );
  tft.setCursor(165, 38);
  tft.println( /*100  * (OBDDATA[OBD_INJ] * OBDDATA[OBD_RPM] / OBDDATA[OBD_SPD]*Ls * 0.18), 1*/"99.9");

  tft.setCursor(165, 219);
  tft.println( /*100  * (OBDDATA[OBD_INJ] * OBDDATA[OBD_RPM] / OBDDATA[OBD_SPD]*Ls * 0.18), 1*/"99.9");











}
/*

    u8g.drawStr( 0, 15, "TRIP" );
    u8g.drawStr( 74, 15, "L" );
    u8g.setPrintPos(35, 15) ;
    u8g.print(trip_fuel_consumption, 1);
    if (LoggingOn == true)  u8g.drawHLine(20, 24, 88);
    u8g.setPrintPos(90, 20) ;
    u8g.print((float)TT * VREF_MEASURED  * TT_PIN_DIVIDER / 1024, 2);

        u8g.setFont(u8g_font_profont22r);
        switch (TT_curr) { //для делителя 10k + 4.7k
          case 0: u8g.drawStr( 95, 20, "1" );   break;
          case 2: u8g.drawStr( 95, 20, "2" ); break;
          case 4: u8g.drawStr( 95, 20, "3" ); break;
          case 5: u8g.drawStr( 95, 20, "3L" ); break;
          case 6: u8g.drawStr( 95, 20, "4" ); break;
          case 7: u8g.drawStr( 95, 20, "4L" ); break;
        }
    if (getOBDdata(OBD_SPD) > 1)
    {
      u8g.setFont(u8g_font_profont15r);
      u8g.drawStr( 0, 42, "L/100Km" );
      u8g.setFont(u8g_font_profont22r);
      u8g.setPrintPos(0, 60) ;
      u8g.print( 100 / getOBDdata(OBD_SPD) * (getOBDdata(OBD_INJ) * getOBDdata(OBD_RPM)*Ls * 0.18), 1);
    } else {
      u8g.setFont(u8g_font_profont15r);
      u8g.drawStr( 0, 42, "L/Hour" );
      u8g.setFont(u8g_font_profont22r);
      u8g.setPrintPos(0, 60) ;
      u8g.print(getOBDdata(OBD_INJ) * getOBDdata(OBD_RPM)*Ls * 0.18, 1);
    }
    u8g.setFont(u8g_font_profont15r);
    u8g.drawStr( 60, 42, "Average" );
    u8g.setFont(u8g_font_profont22r);
    u8g.setPrintPos(60, 60) ;
    if (trip_avg_fuel_consumption < 100)
      u8g.print( trip_avg_fuel_consumption, 1);
    else u8g.drawStr( 60, 60, "---" );
  }
  while ( u8g.nextPage() );
  }
*/

/*
  void DrawTotalFuelConsuption(void) {
  u8g.setFont(u8g_font_profont15r);
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_profont15r);
    u8g.drawStr( 0, 15, "TOTAL" );
    u8g.drawStr( 74, 15, "L" );
    u8g.setPrintPos(42, 15) ;
    u8g.print(total_fuel_consumption, 1);
    if (LoggingOn == true)  u8g.drawHLine(20, 24, 88);
    u8g.setFont(u8g_font_profont22r);
    switch (TT_curr) { //для делителя 10k + 4.7k
      case 0: u8g.drawStr( 95, 20, "1" );   break;
      case 2: u8g.drawStr( 95, 20, "2" ); break;
      case 4: u8g.drawStr( 95, 20, "3" ); break;
      case 5: u8g.drawStr( 95, 20, "3L" ); break;
      case 6: u8g.drawStr( 95, 20, "4" ); break;
      case 7: u8g.drawStr( 95, 20, "4L" ); break;
    }
    if (getOBDdata(OBD_SPD) > 1)
    {
      u8g.setFont(u8g_font_profont15r);
      u8g.drawStr( 0, 42, "L/100Km" );
      u8g.setFont(u8g_font_profont22r);
      u8g.setPrintPos(0, 60) ;

    } else {
      u8g.setFont(u8g_font_profont15r);
      u8g.drawStr( 0, 42, "L/Hour" );
      u8g.setFont(u8g_font_profont22r);
      u8g.setPrintPos(0, 60) ;
      u8g.print(getOBDdata(OBD_INJ) * getOBDdata(OBD_RPM)*Ls * 0.18, 1);
    }
    u8g.setFont(u8g_font_profont15r);
    u8g.drawStr( 60, 42, "Average" );
    u8g.setFont(u8g_font_profont22r);
    u8g.setPrintPos(60, 60) ;
    if (total_avg_consumption < 100)
      u8g.print( total_avg_consumption, 1);
    else u8g.drawStr( 60, 60, "---" );
  }
  while ( u8g.nextPage() );
  }
*/

/*
  void drawTimeDistance(void) {
  u8g.setFont(u8g_font_profont15r);
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_profont15r);
    u8g.drawStr( 0, 15, "TOTAL" );
    //u8g.drawStr( 90, 15, "KM" );
    u8g.setPrintPos(44, 15) ;
    u8g.print(total_trip, 1);
    if (LoggingOn == true)  u8g.drawHLine(20, 24, 88);
    u8g.setFont(u8g_font_profont22r);
    switch (TT_curr) { //для делителя 10k + 4.7k
      case 0: u8g.drawStr( 95, 20, "1" );   break;
      case 2: u8g.drawStr( 95, 20, "2" ); break;
      case 4: u8g.drawStr( 95, 20, "3" ); break;
      case 5: u8g.drawStr( 95, 20, "3L" ); break;
      case 6: u8g.drawStr( 95, 20, "4" ); break;
      case 7: u8g.drawStr( 95, 20, "4L" ); break;
    }
    u8g.setFont(u8g_font_profont15r);
    u8g.drawStr( 0, 42, "Avg SPD" );
    u8g.setFont(u8g_font_profont22r);
    u8g.setPrintPos(0, 60) ;
    u8g.print(total_avg_speed, 1);

    u8g.setFont(u8g_font_profont15r);
    u8g.drawStr( 60, 42, "Time (M)" );
    u8g.setFont(u8g_font_profont22r);
    u8g.setPrintPos(60, 60) ;
    u8g.print( float(total_time) / 60000, 1);
  }
  while ( u8g.nextPage() );
  }
*/

/*
  void drawTripTimeDistance(void) {
  u8g.setFont(u8g_font_profont15r);
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_profont15r);
    u8g.drawStr( 0, 15, "TRIP" );
    //u8g.drawStr( 90, 15, "KM" );
    u8g.setPrintPos(44, 15) ;
    u8g.print(current_trip, 1);
    if (LoggingOn == true)  u8g.drawHLine(20, 24, 88);
    u8g.setFont(u8g_font_profont22r);
    switch (TT_curr) { //для делителя 10k + 4.7k
      case 0: u8g.drawStr( 95, 20, "1" );   break;
      case 2: u8g.drawStr( 95, 20, "2" ); break;
      case 4: u8g.drawStr( 95, 20, "3" ); break;
      case 5: u8g.drawStr( 95, 20, "3L" ); break;
      case 6: u8g.drawStr( 95, 20, "4" ); break;
      case 7: u8g.drawStr( 95, 20, "4L" ); break;
    }
    u8g.setFont(u8g_font_profont15r);
    u8g.drawStr( 0, 42, "Avg SPD" );
    u8g.setFont(u8g_font_profont22r);
    u8g.setPrintPos(0, 60) ;
    u8g.print(trip_avg_speed, 1);

    u8g.setFont(u8g_font_profont15r);
    u8g.drawStr( 60, 42, "Time (M)" );
    u8g.setFont(u8g_font_profont22r);
    u8g.setPrintPos(60, 60) ;
    u8g.print( float(current_time) / 60000, 1);
  }
  while ( u8g.nextPage() );
  }
*/

void update_tft_float(uint8_t i) {
  if  (OLDOBDDATA[i] != OBDDATA[i]) {
    tft.fillRect(tft.getCursorX(), tft.getCursorY() - 16, 76, 19, ILI9341_BLACK);
    tft.print(OBDDATA[i], 1);
  }
}

void update_tft_int(uint8_t i) {
  if  (OLDOBDDATA[i] != OBDDATA[i]) {
    tft.fillRect(tft.getCursorX(), tft.getCursorY() - 16, 76, 19, ILI9341_BLACK);
    tft.print(int(OBDDATA[i]));
  }
}

void initscreen(void) {
  tft.setTextColor(ILI9341_GREEN);
  //tft.setFont(&Orbitron_Medium9pt7b);
  tft.setCursor(15, 18);
  tft.println("COMMAND.COM");
  tft.println("LOAD BIOS");
  tft.println("BIOS SYSTEM CHECK");
  tft.println("RAM CHECK");
  tft.println("CONFIG.SYS");
  tft.println("TOYOTA IFACE");
  tft.println("TO ROM I/O");
  tft.println("CONTROLLER");
  tft.println("COMSPEC.EXE");
  tft.println("CAR UTILS");
  tft.println("SYSTEM BUFFER");
  tft.println("PARAMETERS");
  tft.println("MEMORY SET");
  tft.println("SYSTEM STATUS");
  tft.println("OK _");
  delay (1000);
  tft.fillScreen(ILI9341_BLACK);
}


void drawAllData(void) {
  uint8_t i;
  tft.setTextColor(ILI9341_GREEN);
  tft.setFont(&CPMono_v07_Plain9pt7b);
  if (isActive == false) {
    tft.setCursor(15, 18);
    tft.print("INJ");
    tft.setCursor(15, 36);
    tft.print("IGN");
    tft.setCursor(15, 54);
    tft.print("IAC");
    tft.setCursor(15, 72);
    tft.print("RPM");
    tft.setCursor(15, 90);
    tft.print("MAP");
    tft.setCursor(15, 108);
    tft.print("ECT");
    tft.setCursor(15, 126);
    tft.print("TPS");
    tft.setCursor(15, 144);
    tft.print("SPD");
    tft.setCursor(15, 162);
    tft.print("VF");
  }
  tft.setCursor(69, 18);
  update_tft_float(OBD_INJ);
  tft.setCursor(69, 36);
  update_tft_int(OBD_IGN);
  tft.setCursor(69, 54);
  update_tft_int(OBD_IAC);
  tft.setCursor(69, 72);
  update_tft_int(OBD_RPM);
  tft.setCursor(69, 90);
  update_tft_int(OBD_MAP);
  tft.setCursor(69, 108);
  update_tft_int(OBD_ECT);
  tft.setCursor(69, 126);
  update_tft_int(OBD_TPS);
  tft.setCursor(69, 144);
  update_tft_int(OBD_SPD);
  tft.setCursor(69, 162);
  update_tft_float(OBD_OXSENS);


  if ((OBDFLAG[0] == true) && (OBDFLAG[0] != OLDOBDFLAG[0])) {
    tft.setCursor(160, 18);
    tft.print("START");
    tft.setCursor(186, 36);
    tft.print("ENRICH");
  }
  else if ((OBDFLAG[0] == false) && (OBDFLAG[0] != OLDOBDFLAG[0])) tft.fillRect(160, 2, 160, 42, ILI9341_BLACK);

  if ((OBDFLAG[1] == true) && (OBDFLAG[1] != OLDOBDFLAG[1])) {
    tft.setCursor(160, 54);
    tft.print("COLD");
  } else if ((OBDFLAG[1] == false) && (OBDFLAG[1] != OLDOBDFLAG[1])) tft.fillRect(160, 42, 160, 19, ILI9341_BLACK);

  if ((OBDFLAG[2] == true) && (OBDFLAG[2] != OLDOBDFLAG[2])) {
    tft.setCursor(160, 72);
    tft.print("KNOCKING");
  } else if ((OBDFLAG[2] == false) && (OBDFLAG[2] != OLDOBDFLAG[2])) tft.fillRect(160, 59, 160, 19, ILI9341_BLACK);

  if ((OBDFLAG[3] == true) && (OBDFLAG[3] != OLDOBDFLAG[3])) {
    tft.setCursor(160, 90);
    tft.print("OPEN LOOP");
  } else if ((OBDFLAG[3] == false) && (OBDFLAG[3] != OLDOBDFLAG[3]))  tft.fillRect(160, 78, 160, 19, ILI9341_BLACK);

  if ((OBDFLAG[4] == true) && (OBDFLAG[4] != OLDOBDFLAG[4])) {
    tft.setCursor(160, 108);
    tft.print("ACCEL");
    tft.setCursor(186, 126);
    tft.print("ENRICH");
  } else if ((OBDFLAG[4] == false) && (OBDFLAG[4] != OLDOBDFLAG[4])) tft.fillRect(160, 95, 160, 42, ILI9341_BLACK);

  if ((OBDFLAG[5] == true) && (OBDFLAG[5] != OLDOBDFLAG[5])) {
    tft.setCursor(160, 144);
    tft.print("STARTER");
  } else if ((OBDFLAG[5] == false) && (OBDFLAG[5] != OLDOBDFLAG[5])) tft.fillRect(160, 130, 160, 19, ILI9341_BLACK);

  if ((OBDFLAG[6] == true) && (OBDFLAG[6] != OLDOBDFLAG[6])) {
    tft.setCursor(160, 162);
    tft.print("IDLE");
  } else if ((OBDFLAG[6] == false) && (OBDFLAG[6] != OLDOBDFLAG[6])) tft.fillRect(160, 149, 160, 19, ILI9341_BLACK);

  if ((OBDFLAG[7] == true) && (OBDFLAG[7] != OLDOBDFLAG[7])) {
    tft.setCursor(160, 180);
    tft.print("AIR COND");
  } else if ((OBDFLAG[7] == false) && (OBDFLAG[7] != OLDOBDFLAG[7])) tft.fillRect(160, 168, 160, 19, ILI9341_BLACK);

  if ((OBDFLAG[8] == true) && (OBDFLAG[8] != OLDOBDFLAG[8])) {
    tft.setCursor(160, 198);
    tft.print("NEUTRAL");
  } else  if ((OBDFLAG[8] == false) && (OBDFLAG[8] != OLDOBDFLAG[8])) tft.fillRect(160, 185, 160, 19, ILI9341_BLACK);

  tft.setCursor(160, 216);
  if ((OBDFLAG[9] == true) && (OBDFLAG[9] != OLDOBDFLAG[9])) {
    tft.fillRect(160, 203, 160, 19, ILI9341_BLACK);
    tft.print("LEAN");
  } else if ((OBDFLAG[9] == false) && (OBDFLAG[9] != OLDOBDFLAG[9])) {
    tft.fillRect(160, 203, 160, 19, ILI9341_BLACK);
    tft.print("RICH");
  }

  for (i = 0; i <= 11; i++) {
    OLDOBDDATA[i] = OBDDATA[i];
    OLDOBDFLAG[i] = OBDFLAG[i];
  }
} // end void drawalldata


void ent() {//ПЕРЕКЛЮЧЕНИЕ ЭКРАНОВ
  CurrentDisplayIDX++;
  if (CurrentDisplayIDX > 6) CurrentDisplayIDX = 1;
  // drawScreenSelector();
}

void getOBD() {
  OBDDATA[OBD_INJ] = ToyotaData[OBD_INJ] * 0.125;              //1
  OBDDATA[OBD_IGN] = ToyotaData[OBD_IGN] * 0.47 - 30;          //2
  OBDDATA[OBD_IAC] = ToyotaData[OBD_IAC] * 0.39215;            //3
  OBDDATA[OBD_RPM] = ToyotaData[OBD_RPM] * 25;                 //4
  OBDDATA[OBD_MAP] = ToyotaData[OBD_MAP] * 2; //MAF            5
  if (ToyotaData[OBD_ECT] >= 243)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT] - 243) * 9.8) + 122;
  else if (ToyotaData[OBD_ECT] >= 237)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT] - 237) * 3.83) + 99;
  else if (ToyotaData[OBD_ECT] >= 228)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT] - 228) * 2.11) + 80.0;
  else if (ToyotaData[OBD_ECT] >= 210)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT] - 210) * 1.11) + 60.0;
  else if (ToyotaData[OBD_ECT] >= 180)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT] - 180) * 0.67) + 40.0;
  else if (ToyotaData[OBD_ECT] >= 135)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT] - 135) * 0.44) + 20.0;
  else if (ToyotaData[OBD_ECT] >= 82)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT] - 82) * 0.38);
  else if (ToyotaData[OBD_ECT] >= 39)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT] - 39) * 0.47) - 20.0;
  else if (ToyotaData[OBD_ECT] >= 15)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT] - 15) * 0.83) - 40.0;
  else
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT] - 15) * 2.0) - 60.0;   // 6
  //endif OBD_ECT
  OBDDATA[OBD_TPS] = ToyotaData[OBD_TPS] / 1.8;              //7
  OBDDATA[OBD_SPD] = ToyotaData[OBD_SPD];                    //8/
  OBDDATA[OBD_OXSENS] = (float)ToyotaData[OBD_OXSENS] * 0.01953125;  //9
#ifdef SECOND_O2SENS
  OBDDATA[OBD_OXSENS2] = (float)ToyotaData[OBD_OXSENS2] * 0.01953125;   //10
#endif
  OBDFLAG[0] = bitRead(ToyotaData[11], 0); //  Переобогащение после запуска 1-Вкл
  OBDFLAG[1] = bitRead(ToyotaData[11], 1); //Холодный двигатель 1-Да
  OBDFLAG[2] = bitRead(ToyotaData[11], 4); //Детонация 1-Да
  OBDFLAG[3] = bitRead(ToyotaData[11], 5); //Обратная связь по лямбда зонду 1-Да
  OBDFLAG[4] = bitRead(ToyotaData[11], 6); //Дополнительное обогащение 1-Да
  OBDFLAG[5] = bitRead(ToyotaData[12], 0); //Стартер 1-Да
  OBDFLAG[6] = bitRead(ToyotaData[12], 1); //Признак ХХ (Дроссельная заслонка) 1-Да(Закрыта)
  OBDFLAG[7] = bitRead(ToyotaData[12], 2); //Кондиционер 1-Да
  OBDFLAG[8] = bitRead(ToyotaData[12], 3); //Нейтраль 1-Да
  OBDFLAG[9] = bitRead(ToyotaData[12], 4); //Смесь  первой половины 1-Богатая, 0-Бедная
#ifdef SECOND_O2SENS //Вторая лябмда для Vобразных движков
  OBDFLAG[10] = bitRead(ToyotaData[12], 5); //Смесь второй половины 1-Богатая, 0-Бедная
#endif
}


void ChangeState() {
  static uint8_t ID, EData[TOYOTA_MAX_BYTES];
  static boolean InPacket = false;
  static unsigned long StartMS;
  static uint16_t BitCount;
  int state = digitalRead(ENGINE_DATA_PIN);
  //  digitalWrite(LED_PIN, state);
  if (InPacket == false)  {
    if (state == MY_HIGH)   {
      StartMS = millis();
    }   else   { // else  if (state == MY_HIGH)
      if ((millis() - StartMS) > (15 * 8))   {
        StartMS = millis();
        InPacket = true;
        BitCount = 0;
      } // end if  ((millis() - StartMS) > (15 * 8))
    } // end if  (state == MY_HIGH)
  }  else   { // else  if (InPacket == false)
    uint16_t bits = ((millis() - StartMS) + 1 ) / 8; // The +1 is to cope with slight time errors
    StartMS = millis();
    // process bits
    while (bits > 0)  {
      if (BitCount < 4)  {
        if (BitCount == 0)
          ID = 0;
        ID >>= 1;
        if (state == MY_LOW)  // inverse state as we are detecting the change!
          ID |= 0x08;
      }   else    { // else    if (BitCount < 4)
        uint16_t bitpos = (BitCount - 4) % 11;
        uint16_t bytepos = (BitCount - 4) / 11;
        if (bitpos == 0)      {
          // Start bit, should be LOW
          if ((BitCount > 4) && (state != MY_HIGH))  { // inverse state as we are detecting the change!
            ToyotaFailBit = BitCount;
            InPacket = false;
            break;
          } // end if ((BitCount > 4) && (state != MY_HIGH))
        }  else if (bitpos < 9)  { //else TO  if (bitpos == 0)
          EData[bytepos] >>= 1;
          if (state == MY_LOW)  // inverse state as we are detecting the change!
            EData[bytepos] |= 0x80;
        } else { // else if (bitpos == 0)
          // Stop bits, should be HIGH
          if (state != MY_LOW)  { // inverse state as we are detecting the change!
            ToyotaFailBit = BitCount;
            InPacket = false;
            break;
          } // end if (state != MY_LOW)
          if ( (bitpos == 10) && ((bits > 1) || (bytepos == (TOYOTA_MAX_BYTES - 1))) ) {
            ToyotaNumBytes = 0;
            ToyotaID = ID;
            for (uint16_t i = 0; i <= bytepos; i++)
              ToyotaData[i] = EData[i];
            ToyotaNumBytes = bytepos + 1;
            if (bits >= 16)  // Stop bits of last byte were 1's so detect preamble for next packet
              BitCount = 0;
            else  {
              ToyotaFailBit = BitCount;
              InPacket = false;
            }
            break;
          }
        }
      }
      ++BitCount;
      --bits;
    } // end while
  } // end (InPacket == false)
} // end void change



