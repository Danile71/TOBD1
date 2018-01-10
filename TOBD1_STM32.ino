

// ToyotaOBD1_Reader
// In order to read the data from the OBD connector, short E1 + TE2. then to read the data connect to VF1.
// Note the data line output is 12V - connecting it directly to one of the arduino pins might damage (proabably) the board
// This is made for diaply with an OLED display using the U8glib - which allow wide range of display types with minor adjusments.
// Many thanks to GadgetFreak for the greate base code for the reasding of the data.
// If you want to use invert line - note the comments on the MY_HIGH and the INPUT_PULLUP in the SETUP void.
#include "screens.cpp"
#include <FRAM24CXX.h>
#include <uRTCLib.h>
#include "SPI.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341_STM.h>
#include <URTouch.h>

#include "CPMonoP9pt7b.h"
#include "CPMono_L6pt7b.h"
#include "DSEG7ModernMini-Bold13pt7b.h"
#include "DSEG7ModernMini-Bold30pt7b.h"
#define VREF_MEASURED 3.32              //Измеренное опорное напряжение с 3.3В стабилизатора
#define TT_PIN_DIVIDER 3.13

// выбрать один вариант логирования
//#define LOGGING_MINIMAL //запись минимума данных
#define LOGGING_FULL    //Запись на SD всех данных
//#define SECOND_O2SENS  // Включение 2го сенсора кислорода для V движков
#define DEBUG_OUTPUT false // for debug option - swith output to Serial

//DEFINE пинов под входы-выходы
//#define LED_PIN         33  //встроенный светодиод
#define OX_PIN          PB0 //D3-PB0 для сенсора кислорода
#define TT_PIN          PA3 //D8-PA3 для сенсора ТТ АКПП
#define ENGINE_DATA_PIN PB4 //D18-PB5-EXTI5 VF1 PIN
//#define TOGGLE_BTN_PIN  18 // D4 screen change PIN
#define TFT_CS         PA4
#define TFT_DC         PC15
#define TFT_LED        PA2
#define LIGHT_PIN   PA1
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

#ifdef SECOND_O2SENS    // Лямбда 2 на V-образных движка. У меня ее нету.
#define TOBD_ID_MAX 10
#define OBD_OXSENS2 9
float OBDDATA[TOBD_ID_MAX], OLDOBDDATA[TOBD_ID_MAX];
bool OBDFLAG[TOBD_ID_MAX + 1], OLDOBDFLAG[TOBD_ID_MAX + 1];
#endif

#define TOBD_ID_MAX 9
float OBDDATA[TOBD_ID_MAX], OLDOBDDATA[TOBD_ID_MAX];
bool OBDFLAG[TOBD_ID_MAX + 1], OLDOBDFLAG[TOBD_ID_MAX + 1];
const uint8_t numReadings = 100;

const char* TOBD_ID_NAME[TOBD_ID_MAX] = {"INJ", "IGN", "IAC", "RPM", "MAP", "ECT", "TPS", "SPD", "VF"};
const char* DT_NAME[6] = {"Hour", "Minute", "Week Day", " Month Day", "Month", "Year"};
float total_fuel_consumption = 0;
float total_inj_dur_RO = 0;
struct OXDATA
{
  uint16_t min[10];
  uint16_t max[10];
  uint32_t time_min[10];
  uint32_t time_max[10];
  uint16_t min_avg;
  uint16_t max_avg;
  float freq;
};


struct LIGHT_STRUCT
{
  uint16_t RAW;
  uint16_t current;
  uint16_t target;
  uint16_t last;
  const uint8_t P = 20;
  uint16_t RAW_VAL = 0;
};

OXDATA OX;
LIGHT_STRUCT TFT_BL;

boolean _isLPK = false;


float trip_fuel_consumption = 0;
float trip_avg_fuel_consumption;
float trip_inj_dur = 0;
float trip_avg_speed;
float trip_km = 0;
float odometerRO;
uint16_t old_trip_km = 0;
float cycle_obd_inj_dur = 0;
float cycle_trip = 0;

uint32_t trip_time = 0;
uint32_t total_time_RO = 0;
uint32_t t;
uint32_t last_log_time = 0;
uint32_t press_time;

bool flagNulSpeed = true, isActive = false;
boolean LoggingOn = false;

int16_t myWidth ;
int16_t myHeight ;
uint32_t touchX, touchY;

float maindata[4], maindataold[4];

uint8_t CurrentDisplayIDX = 1, TT_last = 0, TT_curr = 0;
int16_t _OX, TT;
uint8_t ox_id_max = 0;
uint8_t ox_id_min = 0;
uint16_t readIndex = 0;

uint8_t work_id = 0;
volatile uint8_t ToyotaNumBytes, ToyotaID, ToyotaData[TOYOTA_MAX_BYTES];
volatile uint16_t ToyotaFailBit = 0;
uint8_t omm;

//конструкторы
Adafruit_ILI9341_STM tft = Adafruit_ILI9341_STM(TFT_CS, TFT_DC); // Use hardware SPI
FRAM24CXX FRAM(0x50, 4); //chipaddress, chipDensity
uRTCLib rtc(0x68);
Adafruit_GFX_Button buttons[11];
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
uint8_t DT[6];
void setup() {
  for (uint8_t iii = 0; iii < 10; iii++) {
    OX.min[iii] = 2000;
    OX.max[iii] = 0;
  }
  int check = 0;
  Serial.begin(115200);
  Serial1.begin(115200);
  delay (300);
  //while (!Serial) ; //wait until Serial ready
  Wire.begin();
  tft.begin();
  FRAM.begin();
  Touch.InitTouch(0);
  delay(1000); //Wait a second for OpenLog to init
  tft.setRotation(3);
  tft.setTextColor(ILI9341_GREEN);
  tft.fillScreen(BG_COLOR);
  Touch.setPrecision(PREC_EXTREME);
  myHeight   = tft.height();
  myWidth  = tft.width() ;
  //tft.drawCircle(140, 120, 180, ILI9341_YELLOW);
  //FRAM.write(100, int(1986));
  FRAM.read(0, trip_km); //километраж поездки. Обнуляется кнопкой
  FRAM.read(4, trip_time);  //Время поездки. Обнуляется кнопкой
  FRAM.read(8, odometerRO); //одометр. не обнуляется
  FRAM.read(12, total_inj_dur_RO);  //полная длительность впрыска. не обнуляется.
  FRAM.read(16, trip_inj_dur);//длительность впрыска. Обнуляется кнопкой.
  FRAM.read(20, total_time_RO);//длительность впрыска. Обнуляется кнопкой.
  FRAM.read(100, check);
  tft.setFont(&CPMono_v07_Plain9pt7b );
  tft.setCursor(0, 20);
  tft_string("Diagnostic...");
  tft_string("Checking FRAM");
  if (check == 1986)
  {
    tft.println(check);
    tft_string ("Fram work good");
  } else
  {
    tft.println(check);
    tft_string ("Check FRAM");
  }
  //second, minute, hour, dayOfWeek, dayOfMonth, month, year
  //rtc.set(0, 49, 23, 4, 28, 12, 17);
  pinMode(ENGINE_DATA_PIN, INPUT); // VF1 PIN
  pinMode(TT_PIN, INPUT_ANALOG);
  pinMode(OX_PIN, INPUT_ANALOG);
  pinMode(TFT_LED, PWM);
  pinMode(LIGHT_PIN, INPUT_ANALOG);
  attachInterrupt(ENGINE_DATA_PIN, ChangeState, CHANGE); //setup Interrupt for data line
  CurrentDisplayIDX = 2; // set to display 2
  //Расходомер
  t = millis();
  delay (2000);
  last_log_time = millis();
  press_time = millis();
  rtc.refresh();
  omm = rtc.minute();
  writeHeader();
  LoggingOn = true;
  tft.fillScreen(BG_COLOR);
  buttons[0].initButtonUL(&tft, 20, 10, 50, 50, ILI9341_RED, ILI9341_GREEN, ILI9341_BLACK, (char *)"X", 11 ); //экран настроек
  buttons[1].initButtonUL(&tft, 20, 80, 50, 50, ILI9341_RED, ILI9341_GREEN, ILI9341_BLACK, (char *) "-", 11); //экран настроек
  buttons[2].initButtonUL(&tft, 250, 80, 50, 50, ILI9341_RED, ILI9341_GREEN, ILI9341_BLACK, (char *)"+", 11); //экран настроек
  buttons[3].initButtonUL(&tft, 20, 170, 100, 50, ILI9341_RED, ILI9341_GREEN, ILI9341_BLACK, (char *) "Prev", 11); //экран настроек
  buttons[4].initButtonUL(&tft, 200, 170, 100, 50, ILI9341_RED, ILI9341_GREEN, ILI9341_BLACK, (char *) "Next", 11); //экран настроек
  buttons[5].initButtonUL(&tft, 270, 0, 50, 50, ILI9341_RED, ILI9341_GREEN, ILI9341_BLACK, (char *) ">", 11 ); //Главный экран
  buttons[6].initButtonUL(&tft, 0, 0, 50, 50, ILI9341_RED, ILI9341_GREEN, ILI9341_BLACK, (char *) "<", 11 );  //Главный экран
  buttons[7].initButtonUL(&tft, 0, 210, 30, 30, ILI9341_GREEN, ILI9341_RED, ILI9341_BLACK, (char *)"X", 11 ); //главный экран
  buttons[8].initButtonUL(&tft, 270, 0, 50, 50, ILI9341_RED, ILI9341_GREEN, ILI9341_BLACK, (char *) ">", 11 ); //все данные
  buttons[9].initButtonUL(&tft, 270, 0, 50, 50, ILI9341_RED, ILI9341_GREEN, ILI9341_BLACK, (char *)">", 11); //Отладка
  buttons[10].initButtonUL(&tft, 270, 60, 50, 50, ILI9341_RED, ILI9341_GREEN, ILI9341_BLACK, (char *) "<", 11); //отладка
  isActive = false;
} // END VOID SETUP

void loop(void) {
  unsigned long new_t;
  unsigned int diff_t;
  if (millis() % 2 == 0)
  {
    if (readIndex < numReadings)
    {
      TFT_BL.RAW_VAL += 4095 - analogRead(LIGHT_PIN); //набор данных с сенсора света для усреднения
      readIndex++;
    }
    else
    {
      readIndex = 0;
      TFT_BL.RAW = TFT_BL.RAW_VAL / numReadings;   //усредняем
      TFT_BL.target = map(TFT_BL.RAW, 0, 4095, 0, 65535); //масштабируем
      Serial.print("Analog reading = ");
      Serial.println(TFT_BL.RAW);
    }
  }

  if (millis() % 2 == 0) {
    if (TFT_BL.current - TFT_BL.P > TFT_BL.target)            //+- 20 чтобы яркость не прыгала
      TFT_BL.current -= TFT_BL.P;                             //увеличиваем яркость с шагом 20
    if (TFT_BL.current + TFT_BL.P < TFT_BL.target)            // +- 20 чтобы яркость не прыгала
      TFT_BL.current += TFT_BL.P;                             //уменьшаем яркость с шагом 20
    if (TFT_BL.current > 62000) TFT_BL.current = 62000;       //минимальная яркость 62000
    pwmWrite(TFT_LED, TFT_BL.current);
    Serial.print("TFT BRIGHTness = ");
    Serial.println(TFT_BL.current);
  }
  if (millis() % 100 == 0) {
    readTouch();
  }
  if ((ToyotaNumBytes > 0) && (CurrentDisplayIDX != 4))  {  // if found bytes
    getOBD();
    new_t = millis();
    if (new_t > t && OBDDATA[OBD_RPM] > 100 ) {// выполняем только когда на работающем двигателе
      diff_t = new_t - t;
      cycle_obd_inj_dur = OBDDATA[OBD_RPM]  * Ncyl * (float)diff_t  * OBDDATA[OBD_INJ] / 120000.0; //Время открытых форсунок за 1 такт данных. В МС
      //ОБ/М           ОБ/С
      //форсунка срабатывает раз в 2 оборота КВ
      //6форсунок в с
      //время цикла мс в с. Получаем кол-во срабатываний за время цикла. Умножаем на время открытия форсунки, получаем время открытия 6 форсунок В МИЛЛИСЕКУНДАХ

      trip_inj_dur += cycle_obd_inj_dur;                                                              //Время открытых форсунок за поездку        В МС
      total_inj_dur_RO += cycle_obd_inj_dur;                                                           //Время открытых форсунок за все время. EEPROM    В МС

      trip_fuel_consumption = trip_inj_dur  * Ls / 1000.0;    //потребление топлива за поездку в литрах
      total_fuel_consumption = total_inj_dur_RO  * Ls / 1000.0;  //потребление топлива за все время. Из ЕЕПРОМ в литрах

      cycle_trip = (float)diff_t  * OBDDATA[OBD_SPD] / 3600000.0;  //расстояние пройденное за такт обд данных
      trip_km += cycle_trip;  //Полное пройденное расстояние. EEPROM. В КМ
      odometerRO += cycle_trip;       //электронный одометр. Хранится в еепром и не стирается кнопкой

      trip_time += diff_t;             //Время в пути в миллисекундах с момента включения
      total_time_RO += diff_t;
      trip_avg_speed = trip_km  * 3600000.0 / (float)trip_time ;       //средняя скорость за поездку
      trip_avg_fuel_consumption = 100.0 * trip_fuel_consumption / trip_km; //средний расход за поездку
      t = new_t;//тест

      if (LoggingOn == true) logData();         //запись в лог данных по двоному нажатию на кнопку
      SaveToFram();   //запись данных при остановке

      if (millis() - last_log_time > 180000) {        //Запись данных в EEPROM каждые 3 минуты. Чтобы не потерять данные при движении на трассе
        FRAM.write(0, trip_km);
        FRAM.write(4, trip_time);
        FRAM.write(8, odometerRO);
        FRAM.write(12, total_inj_dur_RO);
        FRAM.write(16, trip_inj_dur);
        FRAM.write(20, total_time_RO);
        last_log_time = millis();
      }
    }
    // // draw screen
    drawScreenSelector();
    for (uint8_t i = 0; i < TOBD_ID_MAX; i++)
      OLDOBDDATA[i] = OBDDATA[i];
    for (uint8_t i = 0; i < TOBD_ID_MAX + 1; i++)
      OLDOBDFLAG[i] = OBDFLAG[i];
    ToyotaNumBytes = 0;     // reset the counter.
  } // end if (ToyotaNumBytes > 0)

  if (millis() % 50 == 0) { //каждые 50мс, когда включено логирование и выбран экран с флагами(!)
    if (ox_id_min == 9) {
      for (uint8_t zz = 0; zz < 10; zz++)
      {
        OX.min_avg = OX.min[zz]++;
        OX.min[zz] = 2000;
      }
      OX.min_avg = (int)OX.min_avg / 10;
      ox_id_min = 0;
    }
    if (ox_id_max == 9) {
      for (uint8_t zz = 0; zz < 10; zz++)
      {
        OX.max_avg = OX.max[zz]++;
        OX.max[zz] = 0;
      }
      OX.max_avg = (int)OX.max_avg / 10;
      ox_id_max = 0;
      OX.freq = (float)(OX.time_max[9] - OX.time_max[0]) / 10000.0; //частота в ГЦ
      drawScreenSelector();
    }

    _OX = analogRead(OX_PIN);
    if (_OX < 1240) { //исключаю ложные показание > ~1В
      if (_OX > OX.max[ox_id_max]) {
        OX.max[ox_id_max] = _OX;
        OX.time_max[ox_id_max] = millis();
      }
      if (_OX < OX.min[ox_id_min]) {
        OX.min[ox_id_min] = _OX;
        OX.time_min[ox_id_min] = millis();
      }
      if (_OX < OX.max[ox_id_max] && millis() > OX.time_max[ox_id_max])
        ox_id_max++;
      if (_OX > OX.min[ox_id_min] && millis() > OX.time_min[ox_id_min])
        ox_id_min++;
    }
  }
  if (millis() % 500 == 0) { //каждые пол секунды читаем состояние АКПП. И ничего не делать когда активны настройки
    TT = analogRead(TT_PIN);
    TT_curr = (int)(TT * VREF_MEASURED * TT_PIN_DIVIDER / 4095.0  + 0.5);
  }
}


void mainscreen(void) {
  tft.setTextColor(ILI9341_GREEN);
  tft.setFont(&CPMono_v07_Light6pt7b );
  if (isActive == false) {                      //если первый раз экран появился то рисуются подписи
    tft.fillScreen(ILI9341_BLACK);
    showtime();
    tft.setFont(&CPMono_v07_Light6pt7b );
    tft.setCursor(16, 90);
    tft.print("Trip fuel");
    tft.setCursor(16, 146);
    tft.print("Coolant temp");
    tft.setCursor(196, 90);
    tft.print("Instant fuel");
    tft.setCursor(196, 146);
    tft.print("Average trip");
    tft.setFont(&CPMono_v07_Plain9pt7b );
    tft.setCursor(117, 120);
    tft.print("L");
    tft.setCursor(117, 182);
    tft.print("C");
    tft.drawCircle(111, 170, 3, ILI9341_GREEN);
    tft.setCursor(270, 182);
    tft.print("LPK");
    tft.setCursor(270, 120);
    if (OBDDATA[OBD_SPD] > 1)       //если есть движение то литры на 100км
      tft.print("LPK");
    if (OBDDATA[OBD_SPD] < 1)   //если стоим то литры в час
      tft.print("LPH");
    tft.setCursor(280, 210);
    tft.print("KM");

    tft.setFont(&DSEG7ModernMini_Bold13pt7b );
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
    tft.setCursor(186, 220);
    if (trip_km < 10)
      tft.print("000");
    else if (trip_km < 100)
      tft.print("00");
    else if (trip_km < 1000)
      tft.print("0");
    tft.print( (int)trip_km);
    tft.setFont(&DSEG7ModernMini_Bold30pt7b);
    tft.drawRoundRect(8, 75, 128, 55, 12, ILI9341_RED);
    tft.drawRoundRect(8, 134, 128, 55, 12, ILI9341_RED);
    tft.drawRoundRect(184, 75, 130, 55, 12, ILI9341_RED);
    tft.drawRoundRect(184, 134, 130, 55, 12, ILI9341_RED);
    switch (TT_curr) { //для делителя 10k + 4.7k
      case 0:
        {
          if (OBDDATA[OBD_SPD] > 1) tft.fillRoundRect(149, 180, 20, 28, 3, ILI9341_RED);    //первая передача только при движении
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
    tft.setFont(&CPMono_v07_Plain9pt7b );
    for (uint8_t i = 5; i < 8; i++)
      buttons[i].drawButton();

    isActive = true;
  }// если экран не меняется то подписи не обновляются


  if (TT_last != TT_curr) {
    switch (TT_curr) { //для делителя 10k + 4.7k
      case 0:
        {
          tft.fillRoundRect(146, 70, 26, 144, 5, ILI9341_BLACK);
          if (OBDDATA[OBD_SPD] > 1) tft.fillRoundRect(149, 180, 20, 28, 3, ILI9341_RED);  //первая передача только с начала движения
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
  maindata[0] = 100  * (OBDDATA[OBD_INJ] * OBDDATA[OBD_RPM] / OBDDATA[OBD_SPD] * Ls * 0.18);  //LPK
  maindata[1] = OBDDATA[OBD_INJ] * OBDDATA[OBD_RPM] * Ls * 0.18;    //LPH
  maindata[2] = trip_avg_fuel_consumption;
  maindata[3] = trip_fuel_consumption;

  tft.setFont(&DSEG7ModernMini_Bold13pt7b );
  tft.setCursor(16, 120);
  if (maindata[3] < maindataold[3] - 0.01 || maindata[3] > maindataold[3] + 0.01)
    send_tft(String (trip_fuel_consumption, 2), 20);

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
      tft.setCursor(270, 120);
      tft.setFont(&CPMono_v07_Plain9pt7b );
      send_tft("LPK", 0);
      _isLPK = true;
    }
    if (maindata[0] != maindataold[0])
    {
      tft.setCursor(188, 120);
      tft.setFont(&DSEG7ModernMini_Bold13pt7b );
      send_tft(String ( maindata[0], 1), 20);
    }
  }
  if (OBDDATA[OBD_SPD] < 1)   //если стоим то литры в час
  {
    if (_isLPK == true)
    {
      tft.setCursor(270, 120);
      tft.setFont(&CPMono_v07_Plain9pt7b );
      send_tft("LPH", 0);
      _isLPK = false;
    }
    if (maindata[1] != maindataold[1])
    {
      tft.setCursor(188, 120);
      tft.setFont(&DSEG7ModernMini_Bold13pt7b );
      send_tft(String ( maindata[1], 1), 20);
    }
  }

  tft.setCursor(188, 182);
  if (trip_avg_fuel_consumption < 100)
    if (maindata[2] < maindataold[2] - 0.1 || maindata[2] > maindataold[2] + 0.1)
      send_tft(String ( trip_avg_fuel_consumption, 1), 20);
  if (trip_avg_fuel_consumption > 100)
    if (maindata[2] != maindataold[2])
      send_tft("99.9", 20);
  showtime();

  if (old_trip_km != (int)trip_km) {
    tft.setFont(&DSEG7ModernMini_Bold13pt7b );
    tft.fillRect(184, 193, 90, 30, BG_COLOR);
    tft.setCursor(186, 220);
    if (trip_km < 10)
      tft.print("000");
    else if (trip_km < 100)
      tft.print("00");
    else if (trip_km < 1000)
      tft.print("0");
    tft.print( (int)trip_km);
    old_trip_km = (int)trip_km;
  }

}


void drawAllData(void) {
  uint8_t i;
  tft.setTextColor(ILI9341_GREEN);
  tft.setFont(&CPMono_v07_Plain9pt7b);
  if (isActive == false) {                    //форсирую перерисовку  после переключения экрана
    tft.fillScreen(ILI9341_BLACK);
    isActive = true;        // ставлю флаг активного экрана чтобы не перерисовывать лишнее
    for (i = 0; i < TOBD_ID_MAX; i++)
    {
      tft.setCursor(15, 18 * i + 18);
      tft.print(TOBD_ID_NAME[i]);
    }
    tft.setCursor(69, 18);
    tft.print(OBDDATA[OBD_INJ], 2);
    tft.setCursor(69, 36);
    tft.print(OBDDATA[OBD_IGN]);
    tft.setCursor(69, 54);
    tft.print(OBDDATA[OBD_IAC]);
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

    buttons[8].drawButton();
  }                        //конец перерисовки
  else
  {
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
      tft.print(F("START"));
      tft.setCursor(186, 36);
      tft.print(F("ENRICH"));
    }
    else if ((OBDFLAG[0] == false) && (OBDFLAG[0] != OLDOBDFLAG[0])) tft.fillRect(160, 2, 160, 42, ILI9341_BLACK);

    if ((OBDFLAG[1] == true) && (OBDFLAG[1] != OLDOBDFLAG[1])) {
      tft.setCursor(160, 54);
      tft.print(F("COLD"));
    } else if ((OBDFLAG[1] == false) && (OBDFLAG[1] != OLDOBDFLAG[1])) tft.fillRect(160, 42, 160, 19, ILI9341_BLACK);

    if ((OBDFLAG[2] == true) && (OBDFLAG[2] != OLDOBDFLAG[2])) {
      tft.setCursor(160, 72);
      tft.print(F("KNOCKING"));
    } else if ((OBDFLAG[2] == false) && (OBDFLAG[2] != OLDOBDFLAG[2])) tft.fillRect(160, 59, 160, 19, ILI9341_BLACK);

    if ((OBDFLAG[3] == true) && (OBDFLAG[3] != OLDOBDFLAG[3])) {
      tft.setCursor(160, 90);
      tft.print(F("OPEN LOOP"));
    } else if ((OBDFLAG[3] == false) && (OBDFLAG[3] != OLDOBDFLAG[3]))  tft.fillRect(160, 78, 160, 19, ILI9341_BLACK);

    if ((OBDFLAG[4] == true) && (OBDFLAG[4] != OLDOBDFLAG[4])) {
      tft.setCursor(160, 108);
      tft.print(F("ACCEL"));
      tft.setCursor(186, 126);
      tft.print(F("ENRICH"));
    } else if ((OBDFLAG[4] == false) && (OBDFLAG[4] != OLDOBDFLAG[4])) tft.fillRect(160, 95, 160, 42, ILI9341_BLACK);

    if ((OBDFLAG[5] == true) && (OBDFLAG[5] != OLDOBDFLAG[5])) {
      tft.setCursor(160, 144);
      tft.print(F("STARTER"));
    } else if ((OBDFLAG[5] == false) && (OBDFLAG[5] != OLDOBDFLAG[5])) tft.fillRect(160, 130, 160, 19, ILI9341_BLACK);

    if ((OBDFLAG[6] == true) && (OBDFLAG[6] != OLDOBDFLAG[6])) {
      tft.setCursor(160, 162);
      tft.print(F("IDLE"));
    } else if ((OBDFLAG[6] == false) && (OBDFLAG[6] != OLDOBDFLAG[6])) tft.fillRect(160, 149, 160, 19, ILI9341_BLACK);

    if ((OBDFLAG[7] == true) && (OBDFLAG[7] != OLDOBDFLAG[7])) {
      tft.setCursor(160, 180);
      tft.print(F("AIR COND"));
    } else if ((OBDFLAG[7] == false) && (OBDFLAG[7] != OLDOBDFLAG[7])) tft.fillRect(160, 168, 160, 19, ILI9341_BLACK);

    if ((OBDFLAG[8] == true) && (OBDFLAG[8] != OLDOBDFLAG[8])) {
      tft.setCursor(160, 198);
      tft.print(F("NEUTRAL"));
    } else  if ((OBDFLAG[8] == false) && (OBDFLAG[8] != OLDOBDFLAG[8])) tft.fillRect(160, 185, 160, 19, ILI9341_BLACK);

    tft.setCursor(160, 216);
    if (OBDFLAG[9] != OLDOBDFLAG[9]) tft.fillRect(160, 203, 160, 19, ILI9341_BLACK);
    if ((OBDFLAG[9] == true) && (OBDFLAG[9] != OLDOBDFLAG[9]))
      tft.print(F("LEAN"));
    else if ((OBDFLAG[9] == false) && (OBDFLAG[9] != OLDOBDFLAG[9]))
      tft.print(F("RICH"));
  }
} // end void drawalldata


void getOBD() {
  OBDDATA[OBD_INJ] = ToyotaData[OBD_INJ + 1] * 0.125;            //0
  OBDDATA[OBD_IGN] = ToyotaData[OBD_IGN + 1] * 0.47 - 30;        //1
  OBDDATA[OBD_IAC] = ToyotaData[OBD_IAC + 1] * 0.39215;          //2
  OBDDATA[OBD_RPM] = ToyotaData[OBD_RPM + 1] * 25;               //3
  OBDDATA[OBD_MAP] = ToyotaData[OBD_MAP + 1] * 2.0; //MAF         4
  if (ToyotaData[OBD_ECT + 1] >= 243)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT + 1] - 243.0) * 9.8) + 122.0;
  else if (ToyotaData[OBD_ECT + 1] >= 237)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT + 1] - 237.0) * 3.83) + 99;
  else if (ToyotaData[OBD_ECT + 1] >= 228)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT + 1] - 228.0) * 2.11) + 80.0;
  else if (ToyotaData[OBD_ECT + 1] >= 210)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT + 1] - 210.0) * 1.11) + 60.0;
  else if (ToyotaData[OBD_ECT + 1] >= 180)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT + 1] - 180.0) * 0.67) + 40.0;
  else if (ToyotaData[OBD_ECT + 1] >= 135)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT + 1] - 135.0) * 0.44) + 20.0;
  else if (ToyotaData[OBD_ECT + 1] >= 82)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT + 1] - 82.0) * 0.38);
  else if (ToyotaData[OBD_ECT + 1] >= 39)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT + 1] - 39.0) * 0.47) - 20.0;
  else if (ToyotaData[OBD_ECT + 1] >= 15)
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT + 1] - 15.0) * 0.83) - 40.0;
  else
    OBDDATA[OBD_ECT] = ((float)(ToyotaData[OBD_ECT + 1] - 15.0) * 2.0) - 60.0; // 5
  //endif OBD_ECT
  OBDDATA[OBD_TPS] = ToyotaData[OBD_TPS + 1] / 1.8;            //6
  OBDDATA[OBD_SPD] = ToyotaData[OBD_SPD + 1];                  //7/
  OBDDATA[OBD_OXSENS] = (float)ToyotaData[OBD_OXSENS + 1] * 0.01953125; //8
#ifdef SECOND_O2SENS
  OBDDATA[OBD_OXSENS2] = (float)ToyotaData[OBD_OXSENS2 + 1] * 0.01953125; //9
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
    if (CurrentDisplayIDX == 1) {
      if (buttons[8].contains(touchX, touchY) && millis() - press_time > 300) {
        press_time = millis();
        CurrentDisplayIDX++;
        isActive = false;
        drawScreenSelector();
      }
    }
    if (CurrentDisplayIDX == 2) {
      if (buttons[5].contains(touchX, touchY) && millis() - press_time > 300) {
        press_time = millis();
        CurrentDisplayIDX++;
        isActive = false;
        drawScreenSelector();
      }
      if (buttons[6].contains(touchX, touchY) && millis() - press_time > 300) {
        press_time = millis();
        CurrentDisplayIDX--;
        isActive = false;
        drawScreenSelector();
      }
      if (buttons[7].contains(touchX, touchY) && millis() - press_time > 300) {
        tft.setFont(&CPMono_v07_Plain9pt7b );
        buttons[7].drawButton(true);
        cleardata();
        delay(300);
        buttons[7].drawButton();
        press_time = millis();
      }
    }

    if (CurrentDisplayIDX == 3) {
      if (buttons[9].contains(touchX, touchY) && millis() - press_time > 300) {
        press_time = millis();
        CurrentDisplayIDX++;
        isActive = false;
        drawScreenSelector();
      }
      if (buttons[10].contains(touchX, touchY) && millis() - press_time > 300) {
        press_time = millis();
        CurrentDisplayIDX--;
        isActive = false;
        drawScreenSelector();
      }
    }
    if (CurrentDisplayIDX == 4) {
      if (buttons[0].contains(touchX, touchY) && millis() - press_time > 300) {
        press_time = millis();
        CurrentDisplayIDX = 2;
        isActive = false;
        drawScreenSelector();
      }
      else if (buttons[1].contains(touchX, touchY) || buttons[2].contains(touchX, touchY) || buttons[3].contains(touchX, touchY) || buttons[4].contains(touchX, touchY))
        settings();
    }
  }
  touchY = 500;
  touchX = 500;
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
  tft.fillRect(x1 - 1, y1, w1 + advance, h1, BG_COLOR);
  tft.print(text);
}

//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
void tft_string (const char *s) {
  for (uint8_t i = 0; i < strlen(s); i++) {
    tft.print(s[i]);
    delay (10);
  }
  tft.println();
}
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------

void drawScreenSelector(void) {
  if (CurrentDisplayIDX == 1) {
    drawAllData();
  }
  else if (CurrentDisplayIDX == 2)
  {
    mainscreen();
  }
  else if (CurrentDisplayIDX == 3)
  {
    summary_screen();
  }

  else if (CurrentDisplayIDX == 4)
  {
    settings();
  }
} // end drawScreenSelector()



void writeHeader() {
  Serial1.println(F("TIME;INJ TIME;IGN;IAC;RPM;MAF;ECT;TPS;SPD;VF1;ASE;OPEN LOOP;Acceleration Enrichment;IDLE;NEUTRAL;OX;Trip Avg Fuel"));
}

void logData() {
  uint8_t i;
  Serial1.print(float(millis()) / 60000.0, 3); Serial1.print(F(";"));
  for (i = 0; i < TOBD_ID_MAX; i++)
  {
    Serial1.print(OBDDATA[i], 2);
    Serial1.print(F(";"));
  }
  Serial1.print(OBDFLAG[0]);//ASE
  Serial1.print(F(";"));
  Serial1.print(OBDFLAG[3]); //OPEN LOOP
  Serial1.print(F(";"));
  Serial1.print(OBDFLAG[4]); //Acceleration Enrichment
  Serial1.print(F(";"));
  Serial1.print(OBDFLAG[6]); //IDLE
  Serial1.print(F(";"));
  Serial1.print(OBDFLAG[8]); //NEUTRAL
  Serial1.print(F(";"));
  Serial1.print(OBDFLAG[9]); //OX
  Serial1.print(F(";"));
  Serial1.print(trip_avg_fuel_consumption, 1);
}


void SaveToFram() {
  if (OBDDATA[OBD_SPD] == 0 && flagNulSpeed == false)  {   //Запись данных в еепром когда остановка авто
    FRAM.write(0, trip_km);
    FRAM.write(4, trip_time);
    FRAM.write(8, odometerRO);
    FRAM.write(12, total_inj_dur_RO);
    FRAM.write(16, trip_inj_dur);
    FRAM.write(20, total_time_RO);
    flagNulSpeed = true;                                  //запрет повторной записи
    last_log_time = millis();                             //чтобы не писать лишний раз
  }
  if (OBDDATA[OBD_SPD] != 0) flagNulSpeed = false;     //начали двигаться - разрешаем запись
}

void cleardata() {
  FRAM.write(0, 0);
  FRAM.write(4, 0);
  FRAM.write(16, 0);
  delay(15);
  FRAM.read(0, trip_km); //километраж поездки. Обнуляется кнопкой
  FRAM.read(4, trip_time);  //Время поездки. Обнуляется кнопкой
  FRAM.read(16, trip_inj_dur);//длительность впрыска. Обнуляется кнопкой.
}

void showtime()
{
  int16_t X1, Y1;
  uint16_t W1, H1;
  rtc.refresh();
  tft.setFont(&DSEG7ModernMini_Bold30pt7b);
  tft.setCursor(57, 60);
  if (isActive == false)
  {
    if (rtc.hour() < 10) tft.print("0");
    tft.print(rtc.hour());
    tft.print(F(":"));
    if (rtc.minute() < 10) tft.print("0");
    tft.print(rtc.minute());
  }

  if (omm != rtc.minute())
  {
    tft.getTextBounds((char *) "00:00", tft.getCursorX(), tft.getCursorY(), &X1, &Y1, &W1, &H1);
    tft.fillRect(X1, Y1, W1, H1, BG_COLOR);
    if (rtc.hour() < 10) tft.print("0");
    tft.print(rtc.hour());
    tft.print(F(":"));
    if (rtc.minute() < 10) tft.print("0");
    tft.print(rtc.minute());
    omm = rtc.minute();
  }
  for (uint8_t i = 0; i < 4; i++)
    maindataold[i] = maindata[i];
}

void settings(void) {
  uint8_t _id = 10; //чтобы ничего не происходило по дефолту
  rtc.refresh();
  tft.setCursor(20, 40);
  tft.setFont(&CPMono_v07_Plain9pt7b);
  tft.setTextColor(ILI9341_GREEN);
  if (isActive == false)
  {
    tft.fillScreen(ILI9341_BLACK);
    delay (300); //от случайного нажатия на кнопку.
    work_id = 0;
    DT[0] = rtc.hour();
    DT[1] = rtc.minute();
    DT[2] = rtc.dayOfWeek();
    DT[3] = rtc.day();
    DT[4] = rtc.month();
    DT[5] = rtc.year();
    isActive = true;
    for (uint8_t i = 0; i < 5; i++)
      buttons[i].drawButton();
    tft.setTextColor(ILI9341_GREEN);
    tft.setCursor(125, 110);
    tft.setFont(&DSEG7ModernMini_Bold30pt7b);
    tft.print(DT[work_id]);
    tft.setCursor(120, 145);
    tft.setFont(&CPMono_v07_Plain9pt7b);
    tft.print(DT_NAME[work_id]);
  }
  for (uint8_t b = 0; b < 5; b++) {
    if (buttons[b].contains(touchX, touchY)) {
      _id = b;
      buttons[b].press(true);  // tell the button it is pressed
    } else
      buttons[b].press(false);  // tell the button it is NOT pressed
  }
  // now we can ask the buttons if their state has changed
  for (uint8_t b = 0; b < 5; b++) {
    if (buttons[b].justReleased())
      buttons[b].drawButton();  // draw normal
    if (buttons[b].justPressed())
      buttons[b].drawButton(true);  // draw invert!
  }
  tft.setCursor(125, 110);
  tft.setFont(&DSEG7ModernMini_Bold30pt7b);
  switch (_id) {
    case 1: {
        if (DT[work_id] > 0 && work_id == 0) DT[work_id]--;
        if (DT[work_id] > 0 && work_id == 1) DT[work_id]--;
        if (DT[work_id] > 1 && work_id == 2) DT[work_id]--;
        if (DT[work_id] > 1 && work_id == 3) DT[work_id]--;
        if (DT[work_id] > 1 && work_id == 4) DT[work_id]--;
        if (DT[work_id] > 0 && work_id == 5) DT[work_id]--;
        rtc.set(0, DT[1], DT[0], DT[2] ,  DT[3], DT[4], DT[5]);
        rtc.refresh();
        DT[0] = rtc.hour();
        DT[1] = rtc.minute();
        DT[2] = rtc.dayOfWeek();
        DT[3] = rtc.day();
        DT[4] = rtc.month();
        DT[5] = rtc.year();
        tft.fillRect(108, 37, 113, 74, BG_COLOR);
        tft.setTextColor(ILI9341_GREEN);
        tft.print(DT[work_id]);
        tft.setCursor(120, 145);
        tft.setFont(&CPMono_v07_Plain9pt7b);
        tft.print(DT_NAME[work_id]);
      } break;
    case 2: {
        if (DT[work_id] < 23 && work_id == 0) DT[work_id]++;
        if (DT[work_id] < 59 && work_id == 1) DT[work_id]++;
        if (DT[work_id] < 7 && work_id == 2) DT[work_id]++;
        if (DT[work_id] < 31 && work_id == 3) DT[work_id]++;
        if (DT[work_id] < 11 && work_id == 4) DT[work_id]++;
        if (DT[work_id] < 99 && work_id == 5) DT[work_id]++;
        rtc.set(0, DT[1], DT[0], DT[2] ,  DT[3], DT[4], DT[5]);
        rtc.refresh();
        DT[0] = rtc.hour();
        DT[1] = rtc.minute();
        DT[2] = rtc.dayOfWeek();
        DT[3] = rtc.day();
        DT[4] = rtc.month();
        DT[5] = rtc.year();
        tft.fillRect(108, 37, 113, 74, BG_COLOR);
        tft.setTextColor(ILI9341_GREEN);
        tft.print(DT[work_id]);
        tft.setCursor(120, 145);
        tft.setFont(&CPMono_v07_Plain9pt7b);
        tft.print(DT_NAME[work_id]);
      } break;
    case 3: {
        if (work_id > 0) work_id--;
        tft.fillRect(108, 37, 130, 114, BG_COLOR);
        tft.setTextColor(ILI9341_GREEN);
        tft.print(DT[work_id]);
        tft.setCursor(120, 145);
        tft.setFont(&CPMono_v07_Plain9pt7b);
        tft.print(DT_NAME[work_id]);
      } break;
    case 4: {
        if (work_id < 5) work_id++;
        tft.fillRect(108, 37, 130, 114, BG_COLOR);
        tft.setTextColor(ILI9341_GREEN);
        tft.print(DT[work_id]);
        tft.setCursor(120, 145);
        tft.setFont(&CPMono_v07_Plain9pt7b);
        tft.print(DT_NAME[work_id]);
      } break;
  }
  delay(100) ;
  tft.setFont(&CPMono_v07_Plain9pt7b);
  for (uint8_t b = 0; b < 5; b++) {
    buttons[b].press(false);
    buttons[b].drawButton();
  }
  // touchX = 0;
  // touchY = 0;
}


void summary_screen(void) {
  tft.setTextColor(ILI9341_GREEN);
  tft.setFont(&CPMono_v07_Plain9pt7b);
  if (isActive == false) {                    //форсирую перерисовку  после переключения экрана
    tft.fillScreen(ILI9341_BLACK);
    isActive = true;        // ставлю флаг активного экрана чтобы не перерисовывать лишнее
    tft.setCursor(10, 15);
    tft.print(F("TRIP   L"));
    tft.setCursor(10, 45);
    tft.print(F("-- AVG L"));
    tft.setCursor(10, 75);
    tft.print(F("TRIP  KM"));
    tft.setCursor(10, 105);
    tft.print(F("-AVG SPD"));
    tft.setCursor(10, 135);
    tft.print(F("TOTAL  L"));
    tft.setCursor(10, 165);
    tft.print(F("TOTAL KM"));
    tft.setCursor(10, 195);
    tft.print(F("OX  FREQ"));
    tft.setCursor(10, 225);
    tft.print(F("OX MIN|MAX"));
    tft.setCursor(140, 15);
    tft.print(trip_fuel_consumption, 1);
    tft.setCursor(140, 45);
    tft.print(trip_avg_fuel_consumption, 1);
    tft.setCursor(140, 75);
    tft.print(trip_km, 1);
    tft.setCursor(140, 105);
    tft.print(trip_avg_speed, 1);
    tft.setCursor(140, 135);
    tft.print(total_fuel_consumption, 1);
    tft.setCursor(140, 165);
    tft.print(odometerRO, 1);
    tft.setCursor(140, 195);
    tft.print(OX.freq, 3);
    tft.setCursor(140, 225);
    tft.print((float)OX.min_avg * VREF_MEASURED  / 4095.0 , 2);
    tft.print(F(" | "));
    tft.print((float)OX.min_avg * VREF_MEASURED  / 4095.0 , 2);
    buttons[9].drawButton();
    buttons[10].drawButton();
  }                        //конец перерисовки
  else
  {
    tft.setCursor(140, 15);
    send_tft(String(trip_fuel_consumption, 1), 20);
    tft.setCursor(140, 45);
    send_tft(String(trip_avg_fuel_consumption, 1), 20);
    tft.setCursor(140, 75);
    send_tft(String(trip_km, 1), 20);
    tft.setCursor(140, 105);
    send_tft(String(trip_avg_speed, 1), 20);
    tft.setCursor(140, 135);
    send_tft(String(total_fuel_consumption, 1), 20);
    tft.setCursor(140, 165);
    send_tft(String(odometerRO, 1), 20);
    tft.setCursor(140, 195);
    send_tft(String(OX.freq, 3), 20);
    tft.setCursor(140, 225);
    tft.fillRect(137, 207, 138, 25, BG_COLOR);
    tft.print((float)OX.min_avg * VREF_MEASURED  / 4095.0 , 2);
    tft.print(F(" | "));
    tft.print((float)OX.min_avg * VREF_MEASURED / 4095.0 , 2);
  } // end void drawalldata
}


