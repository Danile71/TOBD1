#include <uRTCLib.h>
//#include <DS3232RTC.h>

// ToyotaOBD1_Reader
// In order to read the data from the OBD connector, short E1 + TE2. then to read the data connect to VF1.
// Note the data line output is 12V - connecting it directly to one of the arduino pins might damage (proabably) the board
// This is made for diaply with an OLED display using the U8glib - which allow wide range of display types with minor adjusments.
// Many thanks to GadgetFreak for the greate base code for the reasding of the data.
// If you want to use invert line - note the comments on the MY_HIGH and the INPUT_PULLUP in the SETUP void.

#include "U8glib.h"
#include "SdFat.h"
#include <SPI.h>
//#include <EEPROM.h>
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
#define LED_PIN          12
#define OX_PIN          0 //A0 для сенсора кислорода
#define TT_PIN          1 //A1 для сенсора ТТ АКПП
#define ENGINE_DATA_PIN 2 //D2 VF1 PIN
#define TOGGLE_BTN_PIN 4 // D4 screen change PIN
//#define INJECTOR_PIN 3 // D3 Номер ноги для форсунки
#define SS 5             // D5 Номер ноги SS SD модуля

//DEFINE констант расходомера
#define Ls 0.004020653 //производительсность форсунки литров в секунду // базовый 0.004 или 240cc
#define Ncyl 6 //кол-во цилиндров

//DEFINE модуля записи на SD карту
#define FILE_BASE_NAME "Data"   //шаблон имени файла
#define error(msg) sd.errorHalt(&Serial,F(msg)) //ошибки при работе с SD

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
uRTCLib rtc;
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);

SdFat sd;
SdFile file;

MD_KeySwitch S(TOGGLE_BTN_PIN, HIGH);
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
bool flagNulSpeed = true;
unsigned int OX, TT;

volatile uint8_t ToyotaNumBytes, ToyotaID, ToyotaData[TOYOTA_MAX_BYTES];
volatile uint16_t ToyotaFailBit = 0;
boolean LoggingOn = false; // dfeine connection flag and last success packet - for lost connection function.


void setup() {
  char fileName[13] = FILE_BASE_NAME "00.csv";
  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  rtc.set_rtc_address(0x68);
  rtc.set_ee_address(0x57);
  noInterrupts();
  Serial.begin(115200);
  delay(100);
  //EEPROM.put(200, odometer); запись значения одометра
  total_trip = rtc.eeprom_read(104);
  total_time = rtc.eeprom_read(108);
  odometer = rtc.eeprom_read(200);
  total_inj_dur_ee = rtc.eeprom_read(204);
  //EEPROM.get(104, total_trip);
  //EEPROM.get(108, total_time);
  //EEPROM.get(200, odometer);
  //EEPROM.get(204, total_inj_dur_ee);

  //analogReference(EXTERNAL);

  S.begin();
  S.enableDoublePress(true);
  S.enableLongPress(true);
  S.enableRepeat(false);
  S.enableRepeatResult(false);
  S.setDoublePressTime(300);
  S.setLongPressTime(2000);
  u8g.setFont(u8g_font_profont15r);
  /*if (DEBUG_OUTPUT) {
    Serial.println("system Started");
    Serial.print("Read float from EEPROM: ");
    Serial.println(total_trip, 3);
    Serial.println(total_time, 3);
    Serial.println(odometer, 1);
    Serial.println(total_inj_dur_ee, 3);
    }*/


  if (!sd.begin(SS, SPI_FULL_SPEED )) {
    sd.initErrorHalt(&Serial);
  }
  if (BASE_NAME_SIZE > 6) {
    error("FILE_BASE_NAME too long");
  }
  if (sd.exists("Data99.csv"))                    //очистка SD карты если логов более 99 штук
  {
    u8g.setFont(u8g_font_profont15r);
    u8g.firstPage();
    do {
      u8g.drawStr( 0, 17, "WIPE DATA!" );
    } while ( u8g.nextPage() );

    if (!sd.wipe()) {
      sd.errorHalt("Wipe failed.");
    }
    if (!sd.begin(SS, SPI_FULL_SPEED )) {
      sd.initErrorHalt();
    }
  }

  while (sd.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 1]++;
    } else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    } else {
      error("Can't create file name");
    }
  }

  if (!file.open(fileName, O_CREAT | O_WRITE  | O_EXCL)) {
    error("file.open");
  }
  writeHeader();

  pinMode(ENGINE_DATA_PIN, INPUT); // VF1 PIN
  pinMode(LED_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENGINE_DATA_PIN), ChangeState, CHANGE); //setup Interrupt for data line
  pinMode(TOGGLE_BTN_PIN, INPUT);           // кнопка СЛЕД. ЭКРАН
  CurrentDisplayIDX = 1; // set to display 1
  drawScreenSelector();
  //Расходомер
  t = millis();
  last_log_time = millis();
  interrupts();
  delay(1000);
} // END VOID SETUP

void loop(void) {
  unsigned long new_t;
  unsigned int diff_t;
  switch (S.read())
  {
    case MD_KeySwitch::KS_NULL: break;
    case MD_KeySwitch::KS_PRESS:    ent(); break;
    case MD_KeySwitch::KS_DPRESS:   {
        if (LoggingOn == false) LoggingOn = true; else LoggingOn = false;
      } break;
    case MD_KeySwitch::KS_LONGPRESS: cleardata(); break;
    case MD_KeySwitch::KS_RPTPRESS: break;
  }

  if (ToyotaNumBytes > 0)  {    // if found bytes
    new_t = millis();
    if (new_t > t && getOBDdata(OBD_RPM) > 100 ) {// выполняем только когда на работающем двигателе
      diff_t = new_t - t;
      cycle_obd_inj_dur = getOBDdata(OBD_RPM)  * Ncyl * (float)diff_t  * getOBDdata(OBD_INJ) / 120000; //Время открытых форсунок за 1 такт данных. В МС
      //ОБ/М           ОБ/С
      //форсунка срабатывает раз в 2 оборота КВ
      //6форсунок в с
      //время цикла мс в с. Получаем кол-во срабатываний за время цикла. Умножаем на время открытия форсунки, получаем время открытия 6 форсунок В МИЛЛИСЕКУНДАХ

      trip_inj_dur += cycle_obd_inj_dur;                                                              //Время открытых форсунок за поездку        В МС
      total_inj_dur_ee += cycle_obd_inj_dur;                                                           //Время открытых форсунок за все время. EEPROM    В МС

      trip_fuel_consumption = trip_inj_dur  * Ls / 1000;    //потребление топлива за поездку в литрах
      total_fuel_consumption = total_inj_dur_ee  * Ls / 1000;  //потребление топлива за все время. Из ЕЕПРОМ в литрах


      cycle_trip = (float)diff_t  * getOBDdata(OBD_SPD) / 3600000;  //расстояние пройденное за такт обд данных
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

      if (LoggingOn == true) logData();         //запись в лог данных по двоному нажатию на кнопку

      updateEepromData();   //запись данных при остановке

      if (millis() - last_log_time > 180000) {        //Запись данных в EEPROM каждые 3 минуты. Чтобы не потерять данные при движении на трассе
        rtc.eeprom_write(104, total_trip);
        rtc.eeprom_write(108, total_time);
        rtc.eeprom_write(200, odometer);
        rtc.eeprom_write(204, total_inj_dur_ee);
        last_log_time = millis();
      }
    }
    drawScreenSelector(); // draw screen
    ToyotaNumBytes = 0;     // reset the counter.
  } // end if (ToyotaNumBytes > 0)
  if (millis() % 50 == 0 && LoggingOn == true && CurrentDisplayIDX == 6) { //каждые 50мс, когда включено логирование и выбран экран с флагами(!)
    OX = analogRead(OX_PIN);
    if (OX < 400) { //исключаю ложные показание > ~1.3В
      file.write(';');
      file.print(((float)OX * VREF_MEASURED) / 1024, 3 );
      file.println();
    }
  }
  if (millis() % 500 == 0) {  //каждые пол секунды читаем состояние АКПП
    TT = analogRead(TT_PIN);
    TT_curr = (int)(TT * VREF_MEASURED * TT_PIN_DIVIDER / 1024  + 0.5);
    drawScreenSelector();
    if (TT_last != TT_curr) {
      drawScreenSelector();
      TT_last = TT_curr;
    }
    //Serial.println((float)TT * VREF_MEASURED / 1024 * 3.13, 3);
    // Serial.println((int)(TT * VREF_MEASURED / 1024 * 3.13+0.5));
  }
}

//  if (millis() % 5000 < 50) autoscreenchange();      // ротация экранов

void updateEepromData() {
  if (getOBDdata(OBD_SPD) == 0 && flagNulSpeed == false)  {   //Запись данных в еепром когда остановка авто
    EEPROM.put(104, total_trip);
    EEPROM.put(108, total_time);
    EEPROM.put(200, odometer);
    EEPROM.put(204, total_inj_dur_ee);
    flagNulSpeed = true;                                  //запрет повторной записи
    last_log_time = millis();                             //чтобы не писать лишний раз
  }
  if (getOBDdata(OBD_SPD) != 0) flagNulSpeed = false;     //начали двигаться - разрешаем запись
}

void cleardata() {
  int i;
  for (i = 104; i <= 112; i++) {
    EEPROM.update(i, 0);
  }
  for (i = 204; i <= 208; i++) {
    EEPROM.update(i, 0);
  }
  EEPROM.get(104, total_trip);
  EEPROM.get(108, total_time);
  EEPROM.get(204, total_inj_dur_ee);


}
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

//обнуление данных


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

void drawScreenSelector(void) {
  if (CurrentDisplayIDX == 1) DrawCurrentFuelConsuption();
  else if (CurrentDisplayIDX == 2) DrawTotalFuelConsuption();
  else if (CurrentDisplayIDX == 3) drawTimeDistance();
  else if (CurrentDisplayIDX == 4) drawTripTimeDistance();
  else if (CurrentDisplayIDX == 5) drawAllData();
  else if (CurrentDisplayIDX == 6) drawExtraData();
} // end drawScreenSelector()

void DrawCurrentFuelConsuption(void) {
  u8g.setFont(u8g_font_profont15r);
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_profont15r);
    u8g.drawStr( 0, 15, "TRIP" );
    u8g.drawStr( 74, 15, "L" );
    u8g.setPrintPos(35, 15) ;
    u8g.print(trip_fuel_consumption, 1);
    if (LoggingOn == true)  u8g.drawHLine(20, 24, 88);
    u8g.setPrintPos(90, 20) ;
    u8g.print((float)TT * VREF_MEASURED  * TT_PIN_DIVIDER / 1024, 2);
    /*
        u8g.setFont(u8g_font_profont22r);
        switch (TT_curr) { //для делителя 10k + 4.7k
          case 0: u8g.drawStr( 95, 20, "1" );   break;
          case 2: u8g.drawStr( 95, 20, "2" ); break;
          case 4: u8g.drawStr( 95, 20, "3" ); break;
          case 5: u8g.drawStr( 95, 20, "3L" ); break;
          case 6: u8g.drawStr( 95, 20, "4" ); break;
          case 7: u8g.drawStr( 95, 20, "4L" ); break;
        }*/
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
      u8g.print( 100  * (getOBDdata(OBD_INJ) * getOBDdata(OBD_RPM) / getOBDdata(OBD_SPD)*Ls * 0.18), 1);
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




void drawAllData(void) {
  // graphic commands to redraw the complete screen should be placed here
  u8g.setFont(u8g_font_profont15r);
  u8g.firstPage();
  do {
    u8g.drawStr( 0, 17, "INJ" );
    u8g.setPrintPos(25, 17) ;
    u8g.print(getOBDdata(OBD_INJ));

    u8g.drawStr( 0, 32, "IGN");
    u8g.setPrintPos(25, 32) ;
    u8g.print( int(getOBDdata(OBD_IGN)));

    u8g.drawStr( 0, 47, "IAC");
    u8g.setPrintPos(25, 47) ;
    u8g.print( int(getOBDdata(OBD_IAC)));

    u8g.drawStr( 0, 62, "RPM");
    u8g.setPrintPos(25, 62) ;
    u8g.print( int(getOBDdata(OBD_RPM)));

    u8g.drawStr( 65, 17, "MAP" );
    u8g.setPrintPos(92, 17) ;
    u8g.print( int(getOBDdata(OBD_MAP)));

    u8g.drawStr( 65, 32, "ECT");
    u8g.setPrintPos(92, 32) ;
    u8g.print( int(getOBDdata(OBD_ECT)));

    u8g.drawStr( 65, 47, "TPS");
    u8g.setPrintPos(92, 47) ;
    u8g.print( int(getOBDdata(OBD_TPS)));

    u8g.drawStr( 65, 62, "SPD");
    u8g.setPrintPos(92, 62) ;
    u8g.print( int(getOBDdata(OBD_SPD)));

    u8g.drawVLine(63, 0, 64);
  } while ( u8g.nextPage() ); // end picture loop
} // end void drawalldata

void autoscreenchange() {
  CurrentDisplayIDX++;
  if (CurrentDisplayIDX > 3) CurrentDisplayIDX = 1;
  drawScreenSelector();
}
void ent() {//ПЕРЕКЛЮЧЕНИЕ ЭКРАНОВ
  CurrentDisplayIDX++;
  if (CurrentDisplayIDX > 6) CurrentDisplayIDX = 1;
  drawScreenSelector();
}

float getOBDdata(byte OBDdataIDX) {
  float returnValue;
  switch (OBDdataIDX) {
    case 0:// UNKNOWN
      returnValue = ToyotaData[0];
      break;
    case OBD_INJ: //  Время впрыска форсунок  =X*0.125 (мс)
      returnValue = ToyotaData[OBD_INJ] * 0.125; //Время впрыска форсунок
      break;
    case OBD_IGN: // Угол опережения зажигания X*0.47-30 (град)
      returnValue = ToyotaData[OBD_IGN] * 0.47 - 30;
      break;
    case OBD_IAC: //  Состояние клапана ХХ Для разных типов КХХ разные формулы: X/255*100 (%)
      //  X (шаг)
      returnValue = ToyotaData[OBD_IAC] * 0.39215; ///optimize divide
      break;
    case OBD_RPM: //Частота вращения коленвала X*25(об/мин)
      returnValue = ToyotaData[OBD_RPM] * 25;
      break;
    case OBD_MAP: //Расходомер воздуха (MAP/MAF)
      //  X*0.6515 (кПа)
      //  X*4.886 (мм.ртут.столба)
      //  X*0.97 (кПа) (для турбомоторов)
      //  X*7.732 (мм.рт.ст) (для турбомоторов)
      //  (гр/сек) (данная формула для MAF так и не найдена)
      //  X/255*5 (Вольт) (напряжение на расходомере)
      returnValue = ToyotaData[OBD_MAP] * 2; //MAF
      break;
    case OBD_ECT: // Температура двигателя (ECT)
      // В зависимости от величины Х разные формулы:
      // 0..14:          =(Х-5)*2-60
      // 15..38:        =(Х-15)*0.83-40
      // 39..81:        =(Х-39)*0.47-20
      // 82..134:      =(Х-82)*0.38
      // 135..179:    =(Х-135)*0.44+20
      // 180..209:    =(Х-180)*0.67+40
      // 210..227:    =(Х-210)*1.11+60
      // 228..236:    =(Х-228)*2.11+80
      // 237..242:    =(Х-237)*3.83+99
      // 243..255:    =(Х-243)*9.8+122
      // Температура в градусах цельсия.
      if (ToyotaData[OBD_ECT] >= 243)
        returnValue = ((float)(ToyotaData[OBD_ECT] - 243) * 9.8) + 122;
      else if (ToyotaData[OBD_ECT] >= 237)
        returnValue = ((float)(ToyotaData[OBD_ECT] - 237) * 3.83) + 99;
      else if (ToyotaData[OBD_ECT] >= 228)
        returnValue = ((float)(ToyotaData[OBD_ECT] - 228) * 2.11) + 80.0;
      else if (ToyotaData[OBD_ECT] >= 210)
        returnValue = ((float)(ToyotaData[OBD_ECT] - 210) * 1.11) + 60.0;
      else if (ToyotaData[OBD_ECT] >= 180)
        returnValue = ((float)(ToyotaData[OBD_ECT] - 180) * 0.67) + 40.0;
      else if (ToyotaData[OBD_ECT] >= 135)
        returnValue = ((float)(ToyotaData[OBD_ECT] - 135) * 0.44) + 20.0;
      else if (ToyotaData[OBD_ECT] >= 82)
        returnValue = ((float)(ToyotaData[OBD_ECT] - 82) * 0.38);
      else if (ToyotaData[OBD_ECT] >= 39)
        returnValue = ((float)(ToyotaData[OBD_ECT] - 39) * 0.47) - 20.0;
      else if (ToyotaData[OBD_ECT] >= 15)
        returnValue = ((float)(ToyotaData[OBD_ECT] - 15) * 0.83) - 40.0;
      else
        returnValue = ((float)(ToyotaData[OBD_ECT] - 15) * 2.0) - 60.0;
      break;
    case OBD_TPS: // Положение дроссельной заслонки
      // X/2(градусы)
      // X/1.8(%)
      returnValue = ToyotaData[OBD_TPS] / 1.8;
      break;
    case OBD_SPD: // Скорость автомобиля (км/час)
      returnValue = ToyotaData[OBD_SPD];
      break;
    //  Коррекция для рядных/ коррекция первой половины
    case OBD_OXSENS:
      returnValue = (float)ToyotaData[OBD_OXSENS] * 0.01953125;
      break;

#ifdef SECOND_O2SENS
    case OBD_OXSENS2:// Lambda2 tst
      returnValue = (float)ToyotaData[OBD_OXSENS2] * 0.01953125;
      break;
#endif

    //  читаем Байты флагов побитно
    case 11:
      returnValue = bitRead(ToyotaData[11], 0);  //  Переобогащение после запуска 1-Вкл
      break;
    case 12:
      returnValue = bitRead(ToyotaData[11], 1); //Холодный двигатель 1-Да
      break;
    case 13:
      returnValue = bitRead(ToyotaData[11], 4); //Детонация 1-Да
      break;
    case 14:
      returnValue = bitRead(ToyotaData[11], 5); //Обратная связь по лямбда зонду 1-Да
      break;
    case 15:
      returnValue = bitRead(ToyotaData[11], 6); //Дополнительное обогащение 1-Да
      break;
    case 16:
      returnValue = bitRead(ToyotaData[12], 0); //Стартер 1-Да
      break;
    case 17:
      returnValue = bitRead(ToyotaData[12], 1); //Признак ХХ (Дроссельная заслонка) 1-Да(Закрыта)
      break;
    case 18:
      returnValue = bitRead(ToyotaData[12], 2); //Кондиционер 1-Да
      break;
    case 19:
      returnValue = bitRead(ToyotaData[12], 3); //Нейтраль 1-Да
      break;
    case 20:
      returnValue = bitRead(ToyotaData[12], 4); //Смесь  первой половины 1-Богатая, 0-Бедная
      break;

#ifdef SECOND_O2SENS //Вторая лябмда для Vобразных движков
    case 21:
      returnValue = bitRead(ToyotaData[12], 5); //Смесь второй половины 1-Богатая, 0-Бедная
      break;
#endif

    default: // DEFAULT CASE (in no match to number)
      // send "error" value
      returnValue =  9999.99;
  } // end switch
  // send value back
  return returnValue;
} // end void getOBDdata


void ChangeState() {
  static uint8_t ID, EData[TOYOTA_MAX_BYTES];
  static boolean InPacket = false;
  static unsigned long StartMS;
  static uint16_t BitCount;
  int state = digitalRead(ENGINE_DATA_PIN);
  digitalWrite(LED_PIN, state);
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

void drawExtraData(void) {
  u8g.setFont(u8g_font_profont15r);
  u8g.firstPage();
  do {
    u8g.drawStr( 0, 15, "VF" );
    u8g.setPrintPos(25, 15) ;
    u8g.print(getOBDdata(OBD_OXSENS), 1);
    if (int(getOBDdata(11)) == 1) {
      u8g.drawStr( 0, 30, "ASE");
    }
    if (int(getOBDdata(12)) == 1) {
      u8g.drawStr( 0, 45, "CLD");
    }
    if (int(getOBDdata(13)) == 1) {
      u8g.drawStr( 0, 60, "KNK");
    }
    if (int(getOBDdata(14)) == 1) {
      u8g.drawStr( 40, 30, "OL");
    }
    if (int(getOBDdata(15)) == 1) {
      u8g.drawStr( 40, 45, "AE");
    }
    if (int(getOBDdata(16)) == 1) {
      u8g.drawStr( 40, 60, "STA");
    }
    if (int(getOBDdata(17)) == 1) {
      u8g.drawStr( 70, 30, "IDL");
    }
    if (int(getOBDdata(18)) == 1) {
      u8g.drawStr( 70, 45, "A/C");
    }
    if (int(getOBDdata(19)) == 1) {
      u8g.drawStr( 70, 60, "NSW");
    }
    if (int(getOBDdata(20)) == 0) {
      u8g.drawStr(70, 15, "LEAN");
    } else  {
      u8g.drawStr(70, 15, "RICH");
    }
  }
  while ( u8g.nextPage() );
}
