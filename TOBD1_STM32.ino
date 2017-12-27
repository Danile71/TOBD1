

// ToyotaOBD1_Reader
// In order to read the data from the OBD connector, short E1 + TE2. then to read the data connect to VF1.
// Note the data line output is 12V - connecting it directly to one of the arduino pins might damage (proabably) the board
// This is made for diaply with an OLED display using the U8glib - which allow wide range of display types with minor adjusments.
// Many thanks to GadgetFreak for the greate base code for the reasding of the data.
// If you want to use invert line - note the comments on the MY_HIGH and the INPUT_PULLUP in the SETUP void.
#include <FRAM24CXX.h>
#include <uRTCLib.h>
#include "SPI.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341_STM.h>
#include <URTouch.h>

#include "CPMonoP9pt7b.h"
#include "CPMono_L6pt7b.h"
#include "ticking-timebomb20pt7b.h"
#include "DSEG7ModernMini-Bold30pt7b.h"
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
#define OX_PIN          PB0 //D3-PB0 для сенсора кислорода
#define TT_PIN          PA3 //D8-PA3 для сенсора ТТ АКПП
#define ENGINE_DATA_PIN PB4 //D18-PB5-EXTI5 VF1 PIN
//#define TOGGLE_BTN_PIN  18 // D4 screen change PIN
#define TFT_CS         PA4
#define TFT_DC         PC15

#define BEAM1_COLOUR ILI9341_GREEN
#define BEAM2_COLOUR ILI9341_RED
#define GRATICULE_COLOUR 0x07FF
#define BEAM_OFF_COLOUR ILI9341_BLACK
#define CURSOR_COLOUR ILI9341_GREEN


//DEFINE констант расходомера
#define Ls 0.004020653 //производительсность форсунки литров в секунду // базовый 0.004 или 240cc
#define Ncyl 6.0 //кол-во цилиндров

//DEFINE OBD READER
#define  MY_HIGH  HIGH //LOW    // I have inverted the Eng line using an Opto-Coupler, if yours isn't then reverse these low & high defines.
#define  MY_LOW   LOW //HIGH
#define  TOYOTA_MAX_BYTES  24
#define OBD_INJ 0 //Injector pulse width (INJ)
#define OBD_IGN 1 //Ignition timing angle (IGN)
#define OBD_IAC 2 //Idle Air Control (IAC)
#define OBD_RPM 3 //Engine speed (RPM)
#define OBD_MAP 4 //Manifold Absolute Pressure (MAP)
#define OBD_ECT 5 //Engine Coolant Temperature (ECT)
#define OBD_TPS 6 // Throttle Position Sensor (TPS)
#define OBD_SPD 7 //Speed (SPD)
#define OBD_OXSENS 8 // Лямбда 1
#ifdef SECOND_O2SENS
#define OBD_OXSENS2 9 // Лямбда 2 на V-образных движка. У меня ее нету.
#endif
float OBDDATA[11], OLDOBDDATA[11];
bool OBDFLAG[11], OLDOBDFLAG[11];

#define TOBD_ID_MAX 9
const char* TOBD_ID_NAME[TOBD_ID_MAX] = {"INJ", "IGN", "IAC", "RPM", "MAP", "ECT", "TPS", "SPD", "VF"};

unsigned int pos;

float total_fuel_consumption = 0;
float total_inj_dur_ee = 0;
float total_km = 0;
float total_avg_consumption;
float total_avg_speed;
uint32_t total_time = 0;
boolean _isLPK = false;


float trip_fuel_consumption = 0;
float trip_avg_fuel_consumption;
float trip_inj_dur = 0;
float trip_avg_speed;
float trip_km = 0;

float cycle_obd_inj_dur = 0;
float cycle_trip = 0;

bool SS_ = false;

uint32_t current_time = 0;

uint32_t t;
uint32_t last_log_time = 0;
uint32_t odometer;

bool flagNulSpeed = true, isActive = false;
boolean LoggingOn = false;

int16_t myWidth ;
int16_t myHeight ;
uint32_t touchX, touchY;

float maindata[4], maindataold[4];

byte CurrentDisplayIDX = 1, TT_last = 0, TT_curr = 0;
unsigned int OX, TT;


volatile uint8_t ToyotaNumBytes, ToyotaID, ToyotaData[TOYOTA_MAX_BYTES];
volatile uint16_t ToyotaFailBit = 0;
uint32_t targetTime = 0;
uint8_t hh, mm, ss;
uint8_t omm, ohh;



float sx = 0, sy = 1, mx = 1, my = 0, hx = -1, hy = 0;    // Saved H, M, S x & y multipliers
float sdeg = 0, mdeg = 0, hdeg = 0;
uint16_t osx = 120, osy = 120, omx = 120, omy = 120, ohx = 120, ohy = 120; // Saved H, M, S x & y coords
uint16_t x00 = 0, x11 = 0, y00 = 0, y11 = 0;
boolean initial = 1, clock_init = 0;










//конструкторы


Adafruit_ILI9341_STM tft = Adafruit_ILI9341_STM(TFT_CS, TFT_DC); // Use hardware SPI
//Creating object for FRAM chip
//chipaddress, write protect status, write protect pin, chipDensity
FRAM24CXX FRAM(0x50, 4);
uRTCLib rtc(0x68);
URTouch  Touch( PB13, PB12, PB15, PB14, PA8);
// Color definitions
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0
#define WHITE    0xFFFF

#define BG_COLOR BLACK
void setup() {
  Serial.begin(115200);
  //  while (!Serial) ; //wait until Serial ready
  tft.begin();
  tft.setRotation(3);
  tft.setTextColor(ILI9341_GREEN);
  FRAM.begin();
  Touch.InitTouch(0);
  Touch.setPrecision(PREC_EXTREME);
  delay(50);
  myHeight   = tft.height();
  myWidth  = tft.width() ;
  //tft.drawCircle(140, 120, 180, ILI9341_YELLOW);
  //Serial.println("Writeng to FRAM...");
  //FRAM.write(0, float(260000.10));
  //FRAM.writeLong(4, 13154464);
  //FRAM.writeLong(8, 2600);
  //FRAM.writeFloat(12, 4545454.11);
  FRAM.read(0, total_km);
  FRAM.read(4, total_time);
  FRAM.read(8, odometer);
  FRAM.read(12, total_inj_dur_ee);
  Serial.println(total_km, 3);
  Serial.println(total_time);
  Serial.println(odometer);
  Serial.println(total_inj_dur_ee, 3);
  //second, minute, hour, dayOfWeek, dayOfMonth, month, year
  // rtc.set(0, 04, 20, 3, 27, 12, 17);
  rtc.refresh();
  mm = rtc.minute();
  hh = rtc.hour();
  ss = rtc.second();
  /*if (DEBUG_OUTPUT) {
    Serial.println("system Started");
    Serial.print("Read float from EEPROM: ");
    Serial.println(total_km, 3);
    Serial.println(total_time, 3);
    Serial.println(odometer, 1);
    Serial.println(total_inj_dur_ee, 3);
    }*/


  //  writeHeader();
  pinMode(ENGINE_DATA_PIN, INPUT); // VF1 PIN
  // pinMode(LED_PIN, OUTPUT);
  pinMode(TT_PIN, INPUT_ANALOG);
  pinMode(OX_PIN, INPUT_ANALOG);
  attachInterrupt(ENGINE_DATA_PIN, ChangeState, CHANGE); //setup Interrupt for data line
  CurrentDisplayIDX = 1; // set to display 1
  //Расходомер
  t = millis();
  last_log_time = millis();
  tft.fillScreen(BG_COLOR);
  targetTime = millis() + 1000;
} // END VOID SETUP

void loop(void) {
  unsigned long new_t;
  unsigned int diff_t;
  rtc.refresh();
  readTouch();

  if (targetTime < millis()) {
    targetTime = millis() + 1000;
    ss++;              // Advance second
    if (ss == 60)
      ss = 0;
  }
  if (ss % 2)  // Flash the colon
    SS_ = false;
  else
    SS_ = true;

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
      trip_km += cycle_trip;  //Пройденное расстояние с момента включения. В КМ
      total_km += cycle_trip;    //Полное пройденное расстояние. EEPROM. В КМ
      odometer += cycle_trip;       //электронный одометр. Хранится в еепром и не стирается кнопкой

      current_time += diff_t;             //Время в пути в миллисекундах с момента включения
      total_time += diff_t;                         //полное пройденное время в миллисекундах лимит ~49 суток. EEPROM

      trip_avg_speed = trip_km  * 3600000 / (float)current_time ;       //средняя скорость за поездку
      total_avg_speed = total_km  * 3600000 / (float)total_time;           // средняя скорость за все время. км\ч

      trip_avg_fuel_consumption = 100 * trip_fuel_consumption / trip_km; //средний расход за поездку
      total_avg_consumption = 100 * total_fuel_consumption / total_km;      //среднее потребление за все время - Л на 100км

      t = new_t;//тест

      //  if (LoggingOn == true) logData();         //запись в лог данных по двоному нажатию на кнопку
      SaveToFram();   //запись данных при остановке

      if (millis() - last_log_time > 180000) {        //Запись данных в EEPROM каждые 3 минуты. Чтобы не потерять данные при движении на трассе
        //        rtc.eeprom_write(104, total_km);
        //      rtc.eeprom_write(108, total_time);
        //   rtc.eeprom_write(200, odometer);
        //   rtc.eeprom_write(204, total_inj_dur_ee);
        last_log_time = millis();
      }
    }
    // // draw screen
    drawScreenSelector();
    for (uint8_t i = 0; i <= 11; i++) {
      OLDOBDDATA[i] = OBDDATA[i];
      OLDOBDFLAG[i] = OBDFLAG[i];
    }
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
    TT = analogRead(TT_PIN);
    TT_curr = (int)(TT * VREF_MEASURED * TT_PIN_DIVIDER / 4095.0  + 0.5);
    drawScreenSelector();
    //  Serial.println((float)TT * VREF_MEASURED / 4095.0 * TT_PIN_DIVIDER, 3);
    //  Serial.println((int)(TT * VREF_MEASURED / 4095.0 * TT_PIN_DIVIDER + 0.5));
  }
}


void mainscreen(void) {


  tft.setTextColor(ILI9341_GREEN);
  tft.setFont(&CPMono_v07_Light6pt7b );
  if (isActive == false) {                      //если первый раз экран появился то рисуются подписи
    tft.setCursor(16, 90);
    tft.print("Trip fuel");
    tft.setCursor(16, 146);
    tft.print("Coolant temp");
    tft.setCursor(196, 90);
    tft.print("Instant fuel");
    tft.setCursor(196, 146);
    tft.print("Average trip");
    tft.setFont(&CPMono_v07_Plain9pt7b );
    tft.setCursor(112, 120);
    tft.print("L");
    tft.setCursor(112, 182);
    tft.print("C");
    tft.drawCircle(106, 170, 3, ILI9341_GREEN);
    tft.setCursor(265, 182);
    tft.print("LPK");
    tft.setCursor(265, 120);
    if (OBDDATA[OBD_SPD] > 1)       //если есть движение то литры на 100км
      tft.print("LPK");
    if (OBDDATA[OBD_SPD] < 1)   //если стоим то литры в час
      tft.print("LPH");


    tft.setFont(&ticking_timebomb20pt7b);
    tft.setCursor(16, 120);
    tft.print(trip_fuel_consumption, 1 );

    tft.setCursor(16, 182);
    if (OBDDATA[OBD_ECT] < 50)
      tft.setTextColor(ILI9341_BLUE);
    else if (OBDDATA[OBD_ECT] < 99) tft.setTextColor(ILI9341_GREEN);
    else tft.setTextColor(ILI9341_RED);
    tft.print(OBDDATA[OBD_ECT], 1);

    tft.setTextColor(ILI9341_GREEN);
    tft.setCursor(188, 120);
    if (OBDDATA[OBD_SPD] > 1)        //если есть движение то литры на 100км
      tft.print(maindata[0], 1);
    else tft.print( maindata[1] , 1); //иначе литры в час

    tft.setCursor(188, 182);
    if (trip_avg_fuel_consumption < 100)
      tft.print(trip_avg_fuel_consumption, 1);
    else
      tft.print( "99.9");

    tft.setFont(&DSEG7ModernMini_Bold30pt7b);
    showtime();
    tft.drawRoundRect(8, 75, 128, 55, 12, ILI9341_RED);
    tft.drawRoundRect(8, 134, 128, 55, 12, ILI9341_RED);
    tft.drawRoundRect(184, 75, 130, 55, 12, ILI9341_RED);
    tft.drawRoundRect(184, 134, 130, 55, 12, ILI9341_RED);
    switch (TT_curr) { //для делителя 10k + 4.7k
      case 0:
        {
          tft.fillRoundRect(149, 180, 20, 28, 3, ILI9341_RED);    //первая передача
        } break;
      case 2:
        {
          tft.fillRoundRect(149, 180, 20, 28, 3, ILI9341_RED);    //первая передача
          tft.fillRoundRect(149, 145, 20, 28, 3, ILI9341_RED);   //вторая передача
        } break;
      case 4:
        {
          tft.fillRoundRect(149, 180, 20, 28, 3, ILI9341_RED);    //первая передача
          tft.fillRoundRect(149, 145, 20, 28, 3, ILI9341_RED);   //вторая передача
          tft.fillRoundRect(149, 110, 20, 28, 3, ILI9341_RED);  //третья передача
        } break;
      case 5: {
          tft.fillRoundRect(146, 105, 26, 109, 5, ILI9341_GREEN); //gt lock
          tft.fillRoundRect(149, 180, 20, 28, 3, ILI9341_RED);
          tft.fillRoundRect(149, 145, 20, 28, 3, ILI9341_RED);
          tft.fillRoundRect(149, 110, 20, 28, 3, ILI9341_RED);
        } break;
      case 6:
        {
          tft.fillRoundRect(149, 180, 20, 28, 3, ILI9341_RED);
          tft.fillRoundRect(149, 145, 20, 28, 3, ILI9341_RED);
          tft.fillRoundRect(149, 110, 20, 28, 3, ILI9341_RED);
          tft.fillRoundRect(149, 75, 20, 28, 3, ILI9341_RED);
        } break;
      case 7:
        {
          tft.fillRoundRect(146, 70, 26, 144, 5, ILI9341_GREEN);
          tft.fillRoundRect(149, 180, 20, 28, 3, ILI9341_RED);
          tft.fillRoundRect(149, 145, 20, 28, 3, ILI9341_RED);
          tft.fillRoundRect(149, 110, 20, 28, 3, ILI9341_RED);
          tft.fillRoundRect(149, 75, 20, 28, 3, ILI9341_RED);
        } break;
    }














  }// если экран не меняется то подписи не обновляются


  if (TT_last != TT_curr) {
    switch (TT_curr) { //для делителя 10k + 4.7k
      case 0:
        {
          tft.fillRoundRect(146, 70, 26, 144, 5, ILI9341_BLACK);
          tft.fillRoundRect(149, 180, 20, 28, 3, ILI9341_RED);    //первая передача
        } break;
      case 2:
        {
          tft.fillRoundRect(146, 70, 26, 144, 5, ILI9341_BLACK);
          tft.fillRoundRect(149, 180, 20, 28, 3, ILI9341_RED);    //первая передача
          tft.fillRoundRect(149, 145, 20, 28, 3, ILI9341_RED);   //вторая передача
        } break;
      case 4:
        {
          tft.fillRoundRect(146, 70, 26, 144, 5, ILI9341_BLACK);
          tft.fillRoundRect(149, 180, 20, 28, 3, ILI9341_RED);    //первая передача
          tft.fillRoundRect(149, 145, 20, 28, 3, ILI9341_RED);   //вторая передача
          tft.fillRoundRect(149, 110, 20, 28, 3, ILI9341_RED);  //третья передача
        } break;
      case 5: {
          tft.fillRoundRect(146, 70, 26, 144, 5, ILI9341_BLACK);
          tft.fillRoundRect(146, 105, 26, 109, 5, ILI9341_GREEN); //gt lock
          tft.fillRoundRect(149, 180, 20, 28, 3, ILI9341_RED);
          tft.fillRoundRect(149, 145, 20, 28, 3, ILI9341_RED);
          tft.fillRoundRect(149, 110, 20, 28, 3, ILI9341_RED);
        } break;
      case 6:
        {
          tft.fillRoundRect(146, 70, 26, 144, 5, ILI9341_BLACK);
          tft.fillRoundRect(149, 180, 20, 28, 3, ILI9341_RED);
          tft.fillRoundRect(149, 145, 20, 28, 3, ILI9341_RED);
          tft.fillRoundRect(149, 110, 20, 28, 3, ILI9341_RED);
          tft.fillRoundRect(149, 75, 20, 28, 3, ILI9341_RED);
        } break;
      case 7:
        {
          tft.fillRoundRect(146, 70, 26, 144, 5, ILI9341_GREEN);
          tft.fillRoundRect(149, 180, 20, 28, 3, ILI9341_RED);
          tft.fillRoundRect(149, 145, 20, 28, 3, ILI9341_RED);
          tft.fillRoundRect(149, 110, 20, 28, 3, ILI9341_RED);
          tft.fillRoundRect(149, 75, 20, 28, 3, ILI9341_RED);
        } break;
    }

  }
  //  Serial.println((float)TT * VREF_MEASURED / 4095.0 * TT_PIN_DIVIDER, 3);
  //  Serial.println((int)(TT * VREF_MEASURED / 4095.0 * TT_PIN_DIVIDER + 0.5));
  tft.setFont(&CPMono_v07_Plain9pt7b );
  tft.setCursor(2, 220);
  tft.fillRect(2, 220 - 19, 140, 20, ILI9341_BLACK);
  tft.print("TT=");
  tft.print((float)TT * VREF_MEASURED / 4095.0 * TT_PIN_DIVIDER, 1);
  tft.print("Gear=");
  tft.print(TT_curr);
  TT_last = TT_curr;
  maindata[0] = 100  * (OBDDATA[OBD_INJ] * OBDDATA[OBD_RPM] / OBDDATA[OBD_SPD] * Ls * 0.18);  //LPK
  maindata[1] = OBDDATA[OBD_INJ] * OBDDATA[OBD_RPM] * Ls * 0.18;    //LPH
  maindata[2] = trip_avg_fuel_consumption;
  maindata[3] = trip_fuel_consumption;

  tft.setFont(&ticking_timebomb20pt7b);
  tft.setCursor(16, 120);
  if (maindata[3] != maindataold[3])
    send_tft(String ( trip_fuel_consumption, 1), 20);

  if (OBDDATA[OBD_ECT] < 50)
    tft.setTextColor(ILI9341_BLUE);
  else if (OBDDATA[OBD_ECT] < 99) tft.setTextColor(ILI9341_GREEN);
  else tft.setTextColor(ILI9341_RED);
  tft.setCursor(16, 182);
  if (OBDDATA[OBD_ECT] != OLDOBDDATA[OBD_ECT])
    send_tft(String ( OBDDATA[OBD_ECT], 1), 10);


  tft.setTextColor(ILI9341_GREEN);

  if (OBDDATA[OBD_SPD] > 1)       //если есть движение то литры на 100км
  {
    if (_isLPK == false)
    {
      tft.setCursor(265, 120);
      tft.setFont(&CPMono_v07_Plain9pt7b );
      send_tft("LPK", 0);
      _isLPK = true;
    }
    if (maindata[0] != maindataold[0])
    {
      tft.setCursor(188, 120);
      tft.setFont(&ticking_timebomb20pt7b);
      send_tft(String ( maindata[0], 1), 20);
    }
  }
  if (OBDDATA[OBD_SPD] < 1)   //если стоим то литры в час
  {
    if (_isLPK == true)
    {
      tft.setCursor(265, 120);
      tft.setFont(&CPMono_v07_Plain9pt7b );
      send_tft("LPH", 0);
      _isLPK = false;
    }
    if (maindata[1] != maindataold[1])
    {
      tft.setCursor(188, 120);
      tft.setFont(&ticking_timebomb20pt7b);
      send_tft(String ( maindata[1], 1), 20);
    }
  }

  tft.setCursor(188, 182);
  if (trip_avg_fuel_consumption < 100)
    if (maindata[2] != maindataold[2])
      send_tft(String ( trip_avg_fuel_consumption, 1), 20);
  if (trip_avg_fuel_consumption > 100)
    if (maindata[2] != maindataold[2])
      send_tft("99.9", 20);

  showtime();
  isActive = true;
}


void drawAllData(void) {
  uint8_t i;
  tft.setTextColor(ILI9341_GREEN);
  tft.setFont(&CPMono_v07_Plain9pt7b);
  if (isActive == false) {                    //форсирую перерисовку  после переключения экрана

    for (i = 0; i < TOBD_ID_MAX; i++)
    {
      tft.setCursor(15, 18 * i + 18);
      tft.print(TOBD_ID_NAME[i]);
    }
    tft.setCursor(69, 18);
    tft.print(OBDDATA[OBD_INJ], 2);
    tft.setCursor(69, 36);
    tft.print(OBDDATA[OBD_IGN], 1);
    tft.setCursor(69, 54);
    tft.print(OBDDATA[OBD_IAC], 1);
    tft.setCursor(69, 72);
    tft.print(int(OBDDATA[OBD_RPM]));
    tft.setCursor(69, 90);
    tft.print(int(OBDDATA[OBD_MAP]));
    tft.setCursor(69, 108);
    tft.print(int(OBDDATA[OBD_ECT]));
    tft.setCursor(69, 126);
    tft.print(int(OBDDATA[OBD_TPS]));
    tft.setCursor(69, 144);
    tft.print(int(OBDDATA[OBD_SPD]));
    tft.setCursor(69, 162);
    tft.print(OBDDATA[OBD_OXSENS], 2);

    if (OBDFLAG[0] == true) {
      tft.setCursor(160, 18);
      tft.print("START");
      tft.setCursor(186, 36);
      tft.print("ENRICH");
    }
    if (OBDFLAG[1] == true)  {
      tft.setCursor(160, 54);
      tft.print("COLD");
    }

    if (OBDFLAG[2] == true)  {
      tft.setCursor(160, 72);
      tft.print("KNOCKING");
    }

    if (OBDFLAG[3] == true)  {
      tft.setCursor(160, 90);
      tft.print("OPEN LOOP");
    }

    if (OBDFLAG[4] == true)  {
      tft.setCursor(160, 108);
      tft.print("ACCEL");
      tft.setCursor(186, 126);
      tft.print("ENRICH");
    }

    if (OBDFLAG[5] == true)  {
      tft.setCursor(160, 144);
      tft.print("STARTER");
    }

    if (OBDFLAG[6] == true)  {
      tft.setCursor(160, 162);
      tft.print("IDLE");
    }

    if (OBDFLAG[7] == true) {
      tft.setCursor(160, 180);
      tft.print("AIR COND");
    }

    if (OBDFLAG[8] == true)  {
      tft.setCursor(160, 198);
      tft.print("NEUTRAL");
    }

    tft.setCursor(160, 216);
    if (OBDFLAG[9] == true)  {
      tft.fillRect(160, 203, 160, 19, ILI9341_BLACK);
      tft.print("LEAN");
    } else if (OBDFLAG[9] == false) {
      tft.fillRect(160, 203, 160, 19, ILI9341_BLACK);
      tft.print("RICH");
    }
  }                        //конец перерисовки
  tft.setCursor(69, 18);              //тут данные обновляются только после их изменения
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
  if (OBDFLAG[9] != OLDOBDFLAG[9]) tft.fillRect(160, 203, 160, 19, ILI9341_BLACK);
  if ((OBDFLAG[9] == true) && (OBDFLAG[9] != OLDOBDFLAG[9]))
    tft.print("LEAN");
  else if ((OBDFLAG[9] == false) && (OBDFLAG[9] != OLDOBDFLAG[9]))
    tft.print("RICH");

  isActive = true;        // ставлю флаг активного экрана чтобы не перерисовывать лишнее
} // end void drawalldata


void getOBD() {
  OBDDATA[OBD_INJ] = ToyotaData[OBD_INJ + 1] * 0.125;            //1
  OBDDATA[OBD_IGN] = ToyotaData[OBD_IGN + 1] * 0.47 - 30;        //2
  OBDDATA[OBD_IAC] = ToyotaData[OBD_IAC + 1] * 0.39215;          //3
  OBDDATA[OBD_RPM] = ToyotaData[OBD_RPM + 1] * 25;               //4
  OBDDATA[OBD_MAP] = ToyotaData[OBD_MAP + 1] * 2; //MAF            5
  if (ToyotaData[OBD_ECT] >= 243)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT + 1] - 243) * 9.8) + 122;
  else if (ToyotaData[OBD_ECT] >= 237)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT + 1] - 237) * 3.83) + 99;
  else if (ToyotaData[OBD_ECT] >= 228)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT + 1] - 228) * 2.11) + 80.0;
  else if (ToyotaData[OBD_ECT] >= 210)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT + 1] - 210) * 1.11) + 60.0;
  else if (ToyotaData[OBD_ECT] >= 180)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT + 1] - 180) * 0.67) + 40.0;
  else if (ToyotaData[OBD_ECT] >= 135)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT + 1] - 135) * 0.44) + 20.0;
  else if (ToyotaData[OBD_ECT] >= 82)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT + 1] - 82) * 0.38);
  else if (ToyotaData[OBD_ECT] >= 39)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT + 1] - 39) * 0.47) - 20.0;
  else if (ToyotaData[OBD_ECT] >= 15)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT + 1] - 15) * 0.83) - 40.0;
  else
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT + 1] - 15) * 2.0) - 60.0; // 6
  //endif OBD_ECT
  OBDDATA[OBD_TPS] = ToyotaData[OBD_TPS + 1] / 1.8;            //7
  OBDDATA[OBD_SPD] = ToyotaData[OBD_SPD + 1];                  //8/
  OBDDATA[OBD_OXSENS] = (float)ToyotaData[OBD_OXSENS + 1] * 0.01953125; //9
#ifdef SECOND_O2SENS
  OBDDATA[OBD_OXSENS2] = (float)ToyotaData[OBD_OXSENS2 + 1] * 0.01953125; //10
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

//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
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
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------

void readTouch() {
  if (Touch.dataAvailable())
  {
    Touch.read();
    if ((Touch.getX() > 0) && (Touch.getY() > 0)) {
      touchY = myHeight -  Touch.getX();
      touchX = myWidth - Touch.getY();
    }
    tft.drawPixel(touchX, touchY, BEAM2_COLOUR);
  }

  if ((touchX >  200) && (Touch.dataAvailable()))
    if (CurrentDisplayIDX < 3) {
      CurrentDisplayIDX++;
      tft.fillScreen(ILI9341_BLACK);
      isActive = false;
      Serial.println(isActive);
      drawScreenSelector();
    }
    else CurrentDisplayIDX = 3;

  if ((touchX < 100) && (Touch.dataAvailable()))
    if (CurrentDisplayIDX > 1) {
      CurrentDisplayIDX--;
      tft.fillScreen(ILI9341_BLACK);
      isActive = false;
      Serial.println(isActive);
      drawScreenSelector();
    }
    else CurrentDisplayIDX = 1;
  
}
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
void update_tft_float(uint8_t i) {
  if  (OLDOBDDATA[i] != OBDDATA[i]) {
    tft.fillRect(tft.getCursorX(), tft.getCursorY() - 16, 76, 19, ILI9341_BLACK);
    tft.print(OBDDATA[i], 2);
  }
}

//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
void update_tft_int(uint8_t i) {
  if  (OLDOBDDATA[i] != OBDDATA[i]) {
    tft.fillRect(tft.getCursorX(), tft.getCursorY() - 16, 76, 19, ILI9341_BLACK);
    tft.print(int(OBDDATA[i]));
  }
}
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------

void send_tft(String text, uint8_t advance) {
  int str_len = text.length() + 1;
  char char_array[str_len];
  text.toCharArray(char_array, str_len);
  int16_t x1, y1;
  uint16_t w1, h1;
  tft.getTextBounds(char_array, tft.getCursorX(), tft.getCursorY(), &x1, &y1, &w1, &h1);
  tft.fillRect(x1, y1, w1 + advance, h1, BG_COLOR);
  tft.print(text);
}


//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
void tft_string (char s[]) {
  for (uint8_t i = 0; i < strlen(s); i++) {
    tft.print(s[i]);
    delay (10);
  }
}
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------

void drawScreenSelector(void) {
  if (CurrentDisplayIDX == 1)
    drawAllData();
  else if (CurrentDisplayIDX == 2)
    mainscreen();
} // end drawScreenSelector()


/*
  void writeHeader() {
  #ifdef LOGGING_FULL
  file.print(F(";OX_RAW;INJ TIME;IGN;IAC;RPM;MAP;ECT;TPS;SPD;VF1;OX;ASE;COLD;KNOCK;OPEN LOOP;Acceleration Enrichment;STARTER;IDLE;A/C;NEUTRAL;AVG SPD;LPK_OBD;LPH_OBD;TOTAL_OBD;AVG_OBD;CURR_OBD;CURR_RUN;total_km"));
  #endif
  #ifdef LOGGING_DEBUG
  file.print(F(";OX_RAW;INJ TIME;IGN;IAC;RPM;MAP;ECT;TPS;SPD;VF1;OX;AVG SPD;LPK_OBD;LPH_OBD;TOTAL_OBD;AVG_OBD;CURR_OBD;CURR_RUN;total_km"));
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
  file.print(trip_km);   file.write(';');    //CURR_RUN ok
  file.print(total_km); file.write(';');//RUN_TOTAL      ok
  #endif
  file.println();
  file.sync();
  }
*/

void SaveToFram() {
  if (OBDDATA[OBD_SPD] == 0 && flagNulSpeed == false)  {   //Запись данных в еепром когда остановка авто
    //      FRAM.write(0, 260000.10);
    //      FRAM.write(4, 13154464);
    //      FRAM.write(8, 2600);
    //      FRAM.write(12, 4545454.11);
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
  //  EEPROM.get(104, total_km);
  //  EEPROM.get(108, total_time);
  //  EEPROM.get(204, total_inj_dur_ee);
}

void showtime()
{
  int16_t X1, Y1;
  uint16_t W1, H1;

  tft.setFont(&DSEG7ModernMini_Bold30pt7b  );
  tft.setCursor(57, 60);
  if (omm == rtc.minute())
  {
    if (rtc.hour() < 10) tft.print("0");
    tft.print(rtc.hour());
  }
  else
  {
    if (rtc.hour() < 10) send_tft("0", 0);
    send_tft(String (rtc.hour()), 0);
  }
  if (SS_ == true) {
    tft.getTextBounds(":", tft.getCursorX(), tft.getCursorY(), &X1, &Y1, &W1, &H1);
    tft.fillRect(X1, Y1, W1, H1, BG_COLOR);
    tft.print(" ");
  }
  else
  {
    send_tft(":", 0);
  }
  if (omm == rtc.minute()) {
    if (rtc.minute() < 10) tft.print("0");
    tft.print(rtc.minute());
  }
  else
  {
    tft.getTextBounds("00", tft.getCursorX(), tft.getCursorY(), &X1, &Y1, &W1, &H1);
    tft.fillRect(X1, Y1, W1, H1, BG_COLOR);
    if (rtc.minute() < 10) send_tft("0", 0);
    send_tft(String (rtc.minute()), 0);
  }
  for (uint8_t i = 0; i < 4; i++) {
    maindataold[i] = maindata[i];
  }
  omm = rtc.minute();


}





