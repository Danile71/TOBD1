//todo
//перерисовка 3го экрана кривая
//нету сброса полного километража  -работает
//добавить всетаки отдельный километраж который не будет сбрасываться.  -проверить
//индикатор акпп не показывает нейтраль    -  проверить фикс
//редко обновляется индикатор среднего расхода на главной панели - проверить фикс
//добавить схему белый тест на черном фоне
//добавить экран с датой и временем
//добавить калибровку производительности форсунок


// ToyotaOBD1_Reader
// In order to read the data from the OBD connector, short E1 + TE2. then to read the data connect to VF1.
// Many thanks to GadgetFreak for the greate base code for the reading of the data.
// If you want to use invert line - note the comments on the MY_HIGH and the INPUT_PULLUP in the SETUP void.
#include "extra.h"
#include "src\FRAM24CXX\FRAM24CXX.h"
#include "src\uRTCLib\uRTCLib.h"
#include "SPI.h"
#include "src\MD_KeySwitch\MD_KeySwitch.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341_STM.h>
#include "CPMonoP9pt7b.h"
#include "CPMono_L6pt7b.h"
#include "DSEG7ModernMini-Bold13pt7b.h"
#include "DSEG7ModernMini-Bold30pt7b.h"
#define VREF_MEASURED 3.32              //Измеренное опорное напряжение с 3.3В стабилизатора
#define TT_PIN_DIVIDER 3.13
#define FUEL_PIN_DIVIDER 2
//#define SECOND_O2SENS  // Включение 2го сенсора кислорода для V движков
#define DEBUG_OUTPUT false // for debug option - swith output to Serial

//DEFINE пинов под входы-выходы
//ADC PIN's  3, 4, 5, 6, 7, 8, 9, 10, 11
//#define LED_PIN         33  //встроенный светодиод
#define FUEL_PIN           PB0  //D3-PB0 для датчика уровня топлива - ok
#define TT_PIN             PA3  //D8-PA3 для сенсора ТТ АКПП        - ok
#define ENGINE_DATA_PIN    PB4  //D18-PB5-EXTI5 VF1 PIN             - ok
#define TFT_CS             PA4  //D7                                - bad need remap to D14 PC13
#define TFT_DC             PC15 //D12                               - ok
#define TFT_LED_PIN        PA2  //D9 -PA2 для шим подсветки дисплея - bad    PWM //TIMER2
#define LIGHT_SENSOR_PIN   PA1  //D10-PA1 Для фоторезистора         - ok
#define BUTTON_ARRAY_PIN   PA0  //D11-PA0 для массива из 4х кнопок  - ok
#define RESET_PIN          PB3  //D19-PB3 отдельно для кнопки сброса- ok
//DEFINE констант расходомера
//#define Ls 0.004020653 //производительсность форсунки литров в секунду // базовый 0.004 или 240cc
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
const char* TOBD_ID_NAME[TOBD_ID_MAX] = {"INJ", "IGN", "IAC", "RPM", "MAF", "ECT", "TPS", "SPD", "VF1", "VF2"};
#else
#define TOBD_ID_MAX 9
const char* TOBD_ID_NAME[TOBD_ID_MAX] = {"INJ", "IGN", "IAC", "RPM", "MAF", "ECT", "TPS", "SPD", "VF"};
#endif
const char* DT_NAME[8] = {"Hour", "Minute", "Week Day", " Month Day", "Month", "Year", "Color Scheme", "FORSCAL"};
const char* weekdays[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};  // 0,1,2,3,4,5,6
const char* colorscheme[2] = {"GREEN", "B/W"};
struct TFT_LED_STRUCT
{
  uint16_t in_med;
  uint16_t current;
  uint16_t target;
  const uint8_t P = 5;
  uint16_t in = 0;
};
struct FUEL_STRUCT
{
  uint16_t in = 0;
  uint16_t in_med;
  float volt;
};
struct FLAG_STRUCT
{
  uint8_t f_num = 0;
  boolean TripActive = false;
  boolean isTrip = false;
};
FUEL_STRUCT fuel;
TFT_LED_STRUCT TFT_LED;
FLAG_STRUCT flag;

float OBDDATA[TOBD_ID_MAX], OLDOBDDATA[TOBD_ID_MAX];
bool OBDFLAG[TOBD_ID_MAX + 1], OLDOBDFLAG[TOBD_ID_MAX + 1];
boolean _isLPK = false;
float total_fuel_consumption = 0;
float total_inj_dur = 0, inj_dur_RO, Ls;
float trip_fuel_consumption = 0;
float trip_avg_fuel_consumption, total_avg_fuel_consumption;
float trip_inj_dur = 0;
float trip_avg_speed, total_avg_spd;
float trip_km, total_km, km_RO;

float cycle_obd_inj_dur = 0;
float cycle_trip = 0;
float maindata[4], maindataold[4];
uint32_t trip_time, total_time, time_RO;
uint32_t t, TEXT_COLOR, BG_COLOR, RED_COLOR, BLUE_COLOR, GT_COLOR;
uint32_t last_log_time = 0;
bool flagNulSpeed = true, isActive = false, _fram;
boolean LoggingOn = false;
int16_t TT;
uint16_t old_trip_km = 0;
uint16_t readIndex = 0;
uint8_t CurrentDisplayIDX, TT_last = 0, TT_curr = 0;
uint8_t work_id = 0;
volatile uint8_t ToyotaNumBytes, ToyotaID, ToyotaData[TOYOTA_MAX_BYTES];
volatile uint16_t ToyotaFailBit = 0;
uint8_t omm;
uint8_t keyValue  =  0;
uint8_t newKeyValue;
uint8_t _id = 10; //чтобы ничего не происходило по дефолту
uint8_t DT[7];
//конструкторы
Adafruit_ILI9341_STM tft = Adafruit_ILI9341_STM(TFT_CS, TFT_DC); // Use hardware SPI
FRAM24CXX FRAM(0x50, 4); //chipaddress, chipDensity
uRTCLib rtc(0x68);
MD_KeySwitch S(RESET_PIN, HIGH);
// Color definitions
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0
#define WHITE    0xFFFF
#define ILI9341_GREY 0x8410
//#define BG_COLOR BLACK

void setup() {
  int check = 0;
  Serial.begin(115200);
  Serial1.begin(115200);
  delay (300);
  Wire.begin();
  tft.begin();
  FRAM.begin();
  delay(1000); //Wait a second for OpenLog to init
  tft.setRotation(3);

  //FRAM.write(100, int(1986)); //Запись тестового значения

  FRAM.read(0, trip_km);        //Пробег. Обнуляется кнопкой на главном экране
  FRAM.read(4, trip_time);      //Время поездки. Обнуляется кнопкой на главном экране
  FRAM.read(8, total_km);       //Пробег. Обнуляется кнопкой на третьем экране
  FRAM.read(12, total_inj_dur); //Длительность впрыска. Обнуляется кнопкой на третьем экране
  FRAM.read(16, trip_inj_dur);  //длительность впрыска. Обнуляется кнопкой на главном экране
  FRAM.read(20, total_time);    //Время работы двигателя. Обнуляется кнопкой на третьем экране
  FRAM.read(24, time_RO);       //Время работы двигателя. Не обнуляется никак
  FRAM.read(28, km_RO);         //Пробег. Не обнуляется никак
  FRAM.read(32, inj_dur_RO);    //Длительность впрыска. Не обнуляется никак
  FRAM.write(36, 0.004020653f);
  FRAM.read(36, Ls);

  FRAM.read(40, DT[6]);
  FRAM.read(100, check);
  if (DT[6] == 0) { //Зеленая схема
    TEXT_COLOR = ILI9341_GREEN;
    BG_COLOR = ILI9341_BLACK;
    BLUE_COLOR = ILI9341_BLUE;
    RED_COLOR = ILI9341_RED;
    GT_COLOR = ILI9341_GREEN;
  }
  if (DT[6] == 1) { //ЧБ схема
    TEXT_COLOR = ILI9341_WHITE;
    BG_COLOR = ILI9341_BLACK;
    BLUE_COLOR = ILI9341_WHITE;
    RED_COLOR = ILI9341_WHITE;
    GT_COLOR = ILI9341_GREY;
  }
  tft.setTextColor(TEXT_COLOR);
  tft.fillScreen(BG_COLOR);
  tft.setFont(&CPMono_v07_Plain9pt7b );
  tft.setCursor(0, 20);
  tft_string("Diagnostic...");
  tft_string("Checking FRAM");
  if (check == 1986)
  {
    tft.println(check);
    tft_string ("Fram work good");
    Serial.println("test");
    _fram = true;
  } else
  {
    tft.println(check);
    tft_string ("Check FRAM");
    _fram = false; //отключаю запись в память если возник какой то косяк
  }
  //second, minute, hour, dayOfWeek, dayOfMonth, month, year
  //rtc.set(0, 49, 23, 4, 28, 12, 17);
  pinMode(ENGINE_DATA_PIN, INPUT); // VF1 PIN
  pinMode(TT_PIN, INPUT_ANALOG);
  pinMode(FUEL_PIN, INPUT_ANALOG);
  pinMode(BUTTON_ARRAY_PIN, INPUT_ANALOG);
  pinMode(TFT_LED_PIN, PWM);
  pinMode(LIGHT_SENSOR_PIN, INPUT_ANALOG);
  attachInterrupt(ENGINE_DATA_PIN, ChangeState, CHANGE); //setup Interrupt for data line
  S.begin();
  S.enableDoublePress(false);
  S.enableLongPress(true);
  S.enableRepeat(false);
  S.enableRepeatResult(false);
  S.setDoublePressTime(300);
  S.setLongPressTime(2000);
  CurrentDisplayIDX = 2; // set to display 2


  last_log_time = 0;
  t = millis();
  rtc.refresh();
  omm = rtc.minute();
  writeHeader();    //запись заголовка лога
  LoggingOn = true; //Запуск логирования
  isActive = false;
} // END VOID SETUP

void loop(void) {
  uint32_t new_t;
  uint16_t diff_t;
  if (millis() % 2 == 0) {

    readbuttons();     //каждые 2мс происходит чтение состояния кнопок
    backlightadjust(); //каждые 2мс оценивается яркость и корректируется
  }
  if (millis() % 500 == 0) { //каждые пол секунды читаем состояние АКПП и напряжение с ДУТ
    TT = analogRead(TT_PIN);
    TT_curr = (int)(TT * VREF_MEASURED * TT_PIN_DIVIDER / 4095.0  + 0.5);
    fuel.in = analogRead(FUEL_PIN);
    fuel.volt = (float)fuel.in * VREF_MEASURED * FUEL_PIN_DIVIDER / 4095.0;
    //fuel.volt = (float)fuel.in_med * VREF_MEASURED * FUEL_PIN_DIVIDER / 4095.0;
  }
  if ((ToyotaNumBytes > 0) && (CurrentDisplayIDX != 4))  {  // if found bytes


    if (flag.f_num == 3) {        //    ротация
      flag.isTrip = !flag.isTrip; //    данных
      flag.TripActive = false;    //     на
      flag.f_num = 0;             //    главном экране
    }                             //    раз в три новых
    flag.f_num++;                 //    пакета OBD данных
    getOBD();   //конвертация данных + оформление их вмассив
    new_t = millis();
    if (new_t > t && OBDDATA[OBD_RPM] > 100 ) {// выполняем только  на работающем двигателе
      diff_t = new_t - t;
      cycle_obd_inj_dur = OBDDATA[OBD_RPM]  * Ncyl * (float)diff_t  * OBDDATA[OBD_INJ] / 120000.0; //Время открытых форсунок за 1 такт данных. В МС
      //ОБ/М           ОБ/С
      //форсунка срабатывает раз в 2 оборота КВ
      //6форсунок в с
      //время цикла мс в с. Получаем кол-во срабатываний за время цикла. Умножаем на время открытия форсунки, получаем время открытия 6 форсунок В МИЛЛИСЕКУНДАХ
      trip_inj_dur += cycle_obd_inj_dur;                                                              //Время открытых форсунок за поездку        В МС
      total_inj_dur += cycle_obd_inj_dur;                                                           //Время открытых форсунок за все время. EEPROM    В МС
      inj_dur_RO += cycle_obd_inj_dur;
      cycle_trip = (float)diff_t  * OBDDATA[OBD_SPD] / 3600000.0;  //расстояние пройденное за такт обд данных
      trip_km += cycle_trip;  //Полное пройденное расстояние. EEPROM. В КМ
      total_km += cycle_trip;       //электронный одометр. Хранится в еепром и не стирается кнопкой
      km_RO += cycle_trip;
      trip_time += diff_t;             //Время в пути в миллисекундах с момента включения
      total_time += diff_t;
      time_RO += diff_t;
      t = new_t;//тест
      if (LoggingOn == true) logData();         //запись в лог данных по двоному нажатию на кнопку
      SaveToFram();   //запись данных при остановке
    }
    trip_fuel_consumption = trip_inj_dur  * Ls / 1000.0;    //потребление топлива за поездку в литрах
    total_fuel_consumption = total_inj_dur  * Ls / 1000.0;  //потребление топлива за все время. Из ЕЕПРОМ в литрах
    trip_avg_fuel_consumption = 100.0 * trip_fuel_consumption / trip_km; //средний расход за поездку
    total_avg_fuel_consumption = 100.0 * total_fuel_consumption / total_km;
    trip_avg_speed = trip_km  * 3600000.0 / (float)trip_time ;       //средняя скорость за поездку
    total_avg_spd = total_km * 3600000.0 / (float)total_time ;
    drawScreenSelector();
    for (uint8_t i = 0; i < TOBD_ID_MAX; i++)
      OLDOBDDATA[i] = OBDDATA[i];
    for (uint8_t i = 0; i < TOBD_ID_MAX + 1; i++)
      OLDOBDFLAG[i] = OBDFLAG[i];
    for (uint8_t i = 0; i < 4; i++)
      maindataold[i] = maindata[i];
    ToyotaNumBytes = 0;     // reset the counter.
  } // end if (ToyotaNumBytes > 0)
}

void mainscreen(void) {
  tft.setTextColor(TEXT_COLOR);
  if (isActive == false) {                      //если первый раз экран появился то рисуются подписи
    tft.fillScreen(BG_COLOR);
    showtime();
    tft.setFont(&CPMono_v07_Light6pt7b );
    printpos(F("TRIP FUEL"), 16, 90);
    printpos(F("COOLANT TEMP"), 16, 146);
    printpos(F("INSTANT FUEL"), 196, 90);
    printpos(F("AVERAGE TRIP"), 196, 146);
    printpos(F("TRIP"), 48, 209);
    tft.setFont(&CPMono_v07_Plain9pt7b );
    printpos(F("L"), 117, 120);
    printpos(F("C"), 117, 182);
    tft.drawCircle(111, 170, 3, TEXT_COLOR);
    printpos(F("LPK"), 270, 182);
    if (OBDDATA[OBD_SPD] > 1)       //если есть движение то литры на 100км
      printpos(F("LPK"), 270, 120);
    if (OBDDATA[OBD_SPD] < 1)   //если стоим то литры в час
      printpos(F("LPH"), 270, 120);
    tft.setFont(&DSEG7ModernMini_Bold13pt7b );
    printpos(trip_fuel_consumption, 1, 16, 120);
    if (OBDDATA[OBD_ECT] < 50)
      tft.setTextColor(BLUE_COLOR);
    else if (OBDDATA[OBD_ECT] < 99) tft.setTextColor(TEXT_COLOR);
    else tft.setTextColor(RED_COLOR);
    printpos(OBDDATA[OBD_ECT], 1, 16, 182);
    tft.setTextColor(TEXT_COLOR);
    if (OBDDATA[OBD_SPD] > 1)        //если есть движение то литры на 100км
      printpos(maindata[0], 1, 188, 120);
    else printpos(maindata[1], 1, 188, 120); //иначе литры в час
    if (trip_avg_fuel_consumption < 100)
      printpos(trip_avg_fuel_consumption, 1, 188, 182);
    else
      printpos(F("99.9"), 188, 182);
    tft.drawRoundRect(8, 75, 128, 55, 12, RED_COLOR);
    tft.drawRoundRect(8, 134, 128, 55, 12, RED_COLOR);
    tft.drawRoundRect(184, 75, 130, 55, 12, RED_COLOR);
    tft.drawRoundRect(184, 134, 130, 55, 12, RED_COLOR);
    switch (TT_curr) { //для делителя 10k + 4.7k
      case 0:
        firstgear();
        break;
      case 2:
        secondgear();
        break;
      case 4:
        thirdgear(false);
        break;
      case 5:
        thirdgear(true);
        break;
      case 6:
        fourthgear(false);
        break;
      case 7:
        fourthgear(true);
        break;
    }
    isActive = true;
  }// если экран не меняется то подписи не обновляются
  if (TT_last != TT_curr && OBDDATA[OBD_SPD] > 1) { //только при движении
    switch (TT_curr) { //для делителя 10k + 4.7k
      case 0:
        firstgear();
        break;
      case 2:
        secondgear();
        break;
      case 4:
        thirdgear(false);
        break;
      case 5:
        thirdgear(true);
        break;
      case 6:
        fourthgear(false);
        break;
      case 7:
        fourthgear(true);
        break;
    }
  }
  if (OBDDATA[OBD_SPD] == 0) {
    neutral();
  }
  maindata[0] = 100  * (OBDDATA[OBD_INJ] * OBDDATA[OBD_RPM] / OBDDATA[OBD_SPD] * Ls * 0.18);  //LPK
  maindata[1] = OBDDATA[OBD_INJ] * OBDDATA[OBD_RPM] * Ls * 0.18;    //LPH
  maindata[2] = trip_avg_fuel_consumption;
  maindata[3] = trip_fuel_consumption;
  tft.setTextColor(TEXT_COLOR);
  if (flag.isTrip) {
    if (!flag.TripActive) {
      tft.setFont(&CPMono_v07_Light6pt7b );
      tft.fillRect(16, 82, 100, 12, BG_COLOR);
      printpos(F("TRIP FUEL"), 16, 90);
      tft.fillRect(231, 196, 40, 19, BG_COLOR);
      printpos(F("KM"), 234, 209);
      flag.TripActive = true;
      tft.setFont(&DSEG7ModernMini_Bold13pt7b );
      tft.fillRect(16, 95, 90, 27, BG_COLOR);
      printpos(trip_fuel_consumption, 2, 16, 120);
      tft.fillRect(85, 196, 145, 37, BG_COLOR);
      printpos(F("   "), 90, 225);
      if (trip_km < 10)
        printpos(F("000"), 90, 225);
      else if (trip_km < 100)
        printpos(F("00"), 90, 225);
      else if (trip_km < 1000)
        printpos(F("0"), 90, 225);
      tft.print(trip_km, 1);
    }
    if (fabs(maindata[3] - maindataold[3]) > 0.09)
    {
      tft.setFont(&DSEG7ModernMini_Bold13pt7b );
      tft.fillRect(16, 95, 90, 27, BG_COLOR);
      printpos(trip_fuel_consumption, 2, 16, 120);
    }
  }
  if (!flag.isTrip) {
    if (!flag.TripActive) {
      tft.setFont(&CPMono_v07_Light6pt7b );
      tft.fillRect(16, 82, 100, 12, BG_COLOR);
      printpos(F("FUEL REMAINS"), 16, 90);
      tft.fillRect(231, 196, 40, 19, BG_COLOR);
      printpos(F("TIME"), 234, 209);
      tft.setFont(&DSEG7ModernMini_Bold13pt7b );
      tft.fillRect(16, 95, 90, 27, BG_COLOR);
      printpos(fuel.volt, 2, 16, 120);
      tft.fillRect(85, 196, 145, 37, BG_COLOR);
      printpos(TimeToString(trip_time), 90, 225);
      flag.TripActive = true;
    }
    tft.setFont(&DSEG7ModernMini_Bold13pt7b );
    tft.fillRect(16, 95, 90, 27, BG_COLOR);
    printpos(fuel.volt, 2, 16, 120);
  }
  if (OBDDATA[OBD_ECT] < 50)
    tft.setTextColor(BLUE_COLOR);
  else if (OBDDATA[OBD_ECT] < 99) tft.setTextColor(TEXT_COLOR);
  else tft.setTextColor(RED_COLOR);

  if (fabs(OBDDATA[OBD_ECT] - OLDOBDDATA[OBD_ECT]) > 0.09)
  {
    tft.setFont(&DSEG7ModernMini_Bold13pt7b );
    tft.fillRect(16, 157, 90, 27, BG_COLOR);
    printpos(OBDDATA[OBD_ECT], 1, 16, 182);
  }
  tft.setTextColor(TEXT_COLOR);
  if (OBDDATA[OBD_SPD] > 1)       //если есть движение то литры на 100км
  {
    if (_isLPK == false)
    {
      tft.setFont(&CPMono_v07_Plain9pt7b );
      send_tft("LPK", 0, 270, 120);
      _isLPK = true;
    }
    if (fabs(maindata[0] - maindataold[0]) > 0.09)
    {
      tft.setFont(&DSEG7ModernMini_Bold13pt7b );
      tft.fillRect(188, 95, 82, 27, BG_COLOR);
      printpos(maindata[0], 1, 188, 120);
    }
  }
  if (OBDDATA[OBD_SPD] < 1)   //если стоим то литры в час
  {
    if (_isLPK == true)
    {
      tft.setFont(&CPMono_v07_Plain9pt7b );
      send_tft("LPH", 0, 270, 120);
      _isLPK = false;
    }
    if (fabs(maindata[1] - maindataold[1]) > 0.09)
    {
      tft.setFont(&DSEG7ModernMini_Bold13pt7b );
      tft.fillRect(188, 95, 82, 27, BG_COLOR);
      printpos(maindata[1], 1, 188, 120);
    }
  }
  if (trip_avg_fuel_consumption < 100)
    if (fabs(maindata[2] - maindataold[2]) > 0.09)
    {
      tft.fillRect(188, 157, 82, 27, BG_COLOR);
      printpos(trip_avg_fuel_consumption, 1, 188, 182);
    }
  if (trip_avg_fuel_consumption > 100)
    if (fabs(maindata[2] - maindataold[2]) > 0.09)
    {
      tft.fillRect(188, 157, 82, 27, BG_COLOR);
      printpos(F("99.9"), 188, 182);
    }
  showtime();
}


void drawAllData(void) {
  uint8_t i;
  tft.setTextColor(TEXT_COLOR);
  tft.setFont(&CPMono_v07_Plain9pt7b);
  if (isActive == false) {                    //форсирую перерисовку  после переключения экрана
    tft.fillScreen(BG_COLOR);
    isActive = true;        // ставлю флаг активного экрана чтобы не перерисовывать лишнее
    for (i = 0; i < TOBD_ID_MAX; i++)
      printpos(TOBD_ID_NAME[i], 15, 18 * i + 18);
    printpos(OBDDATA[OBD_INJ], 2, 69, 18);
    printpos(OBDDATA[OBD_IGN], 2, 69, 36);
    printpos(OBDDATA[OBD_IAC], 2, 69, 54);
    printpos(OBDDATA[OBD_RPM], 2, 69, 72);
    printpos(OBDDATA[OBD_MAP], 0, 69, 90);
    printpos(OBDDATA[OBD_ECT], 0, 69, 108);
    printpos(OBDDATA[OBD_TPS], 0, 69, 126);
    printpos(OBDDATA[OBD_SPD], 0, 69, 144);
    printpos(OBDDATA[OBD_OXSENS], 2, 69, 162);
    if (OBDFLAG[0] == true) {
      printpos(F("START"), 160, 18);
      printpos(F("ENRICH"), 160, 36);
    }
    if (OBDFLAG[1] == true)
      printpos(F("COLD"), 160, 54);
    if (OBDFLAG[2] == true)
      printpos(F("KNOCKING"), 160, 72);
    if (OBDFLAG[3] == true)
      printpos(F("CLOSED LOOP"), 160, 90);
    if (OBDFLAG[4] == true)  {
      printpos(F("ACCEL"), 160, 108);
      printpos(F("ENRICH"), 160, 126);
    }
    if (OBDFLAG[5] == true)
      printpos(F("STARTER"), 160, 144);
    if (OBDFLAG[6] == true)
      printpos(F("IDLE"), 160, 162);
    if (OBDFLAG[7] == true)
      printpos(F("AIR COND"), 160, 180);
    if (OBDFLAG[8] == true)
      printpos(F("NEUTRAL"), 160, 198);
    if (OBDFLAG[9] == true)  {
      tft.fillRect(160, 203, 160, 19, BG_COLOR);
      printpos(F("LEAN"), 160, 216);
    } else if (OBDFLAG[9] == false) {
      tft.fillRect(160, 203, 160, 19, BG_COLOR);
      printpos(F("RICH"), 160, 216);
    }
  }                        //конец перерисовки
  else
  { //тут данные обновляются только после их изменения
    update_tft_float(OBD_INJ, 69, 18);
    update_tft_int(OBD_IGN, 69, 36);
    update_tft_int(OBD_IAC, 69, 54);
    update_tft_int(OBD_RPM, 69, 72);
    update_tft_int(OBD_MAP, 69, 90);
    update_tft_int(OBD_ECT, 69, 108);
    update_tft_int(OBD_TPS, 69, 126);
    update_tft_int(OBD_SPD, 69, 144);
    update_tft_float(OBD_OXSENS, 69, 162);
    if ((OBDFLAG[0] == true) && (OBDFLAG[0] != OLDOBDFLAG[0])) {
      printpos(F("START"), 160, 18);
      printpos(F("ENRICH"), 160, 36);
    }
    else if ((OBDFLAG[0] == false) && (OBDFLAG[0] != OLDOBDFLAG[0])) tft.fillRect(160, 2, 160, 42, BG_COLOR);
    if ((OBDFLAG[1] == true) && (OBDFLAG[1] != OLDOBDFLAG[1]))
      printpos(F("COLD"), 160, 54);
    else if ((OBDFLAG[1] == false) && (OBDFLAG[1] != OLDOBDFLAG[1])) tft.fillRect(160, 42, 160, 19, BG_COLOR);
    if ((OBDFLAG[2] == true) && (OBDFLAG[2] != OLDOBDFLAG[2]))
      printpos(F("KNOCKING"), 160, 72);
    else if ((OBDFLAG[2] == false) && (OBDFLAG[2] != OLDOBDFLAG[2])) tft.fillRect(160, 59, 160, 19, BG_COLOR);
    if ((OBDFLAG[3] == true) && (OBDFLAG[3] != OLDOBDFLAG[3]))
      printpos(F("CLOSED LOOP"), 160, 90);
    else if ((OBDFLAG[3] == false) && (OBDFLAG[3] != OLDOBDFLAG[3]))  tft.fillRect(160, 78, 160, 19, BG_COLOR);
    if ((OBDFLAG[4] == true) && (OBDFLAG[4] != OLDOBDFLAG[4])) {
      printpos(F("ACCEL"), 160, 108);
      printpos(F("ENRICH"), 160, 126);
    } else if ((OBDFLAG[4] == false) && (OBDFLAG[4] != OLDOBDFLAG[4])) tft.fillRect(160, 95, 160, 42, BG_COLOR);
    if ((OBDFLAG[5] == true) && (OBDFLAG[5] != OLDOBDFLAG[5]))
      printpos(F("STARTER"), 160, 144);
    else if ((OBDFLAG[5] == false) && (OBDFLAG[5] != OLDOBDFLAG[5])) tft.fillRect(160, 130, 160, 19, BG_COLOR);
    if ((OBDFLAG[6] == true) && (OBDFLAG[6] != OLDOBDFLAG[6])) {
      printpos(F("IDLE"), 160, 162);
    } else if ((OBDFLAG[6] == false) && (OBDFLAG[6] != OLDOBDFLAG[6])) tft.fillRect(160, 149, 160, 19, BG_COLOR);
    if ((OBDFLAG[7] == true) && (OBDFLAG[7] != OLDOBDFLAG[7])) {
      printpos(F("AIR COND"), 160, 180);
    } else if ((OBDFLAG[7] == false) && (OBDFLAG[7] != OLDOBDFLAG[7])) tft.fillRect(160, 168, 160, 19, BG_COLOR);
    if ((OBDFLAG[8] == true) && (OBDFLAG[8] != OLDOBDFLAG[8])) {
      printpos(F("NEUTRAL"), 160, 198);
    } else  if ((OBDFLAG[8] == false) && (OBDFLAG[8] != OLDOBDFLAG[8])) tft.fillRect(160, 185, 160, 19, BG_COLOR);
    if (OBDFLAG[9] != OLDOBDFLAG[9]) tft.fillRect(160, 203, 160, 19, BG_COLOR);
    if ((OBDFLAG[9] == true) && (OBDFLAG[9] != OLDOBDFLAG[9]))
      printpos(F("LEAN"), 160, 216);
    else if ((OBDFLAG[9] == false) && (OBDFLAG[9] != OLDOBDFLAG[9]))
      printpos(F("RICH"), 160, 216);
  }
} // end void drawalldata


void getOBD() {
  OBDDATA[OBD_INJ] = ToyotaData[OBD_INJ + 1] * 0.125;            //0
  OBDDATA[OBD_IGN] = ToyotaData[OBD_IGN + 1] * 0.47 - 30;        //1
  OBDDATA[OBD_IAC] = ToyotaData[OBD_IAC + 1] * 0.39215;          //2
  OBDDATA[OBD_RPM] = ToyotaData[OBD_RPM + 1] * 25;               //3
  OBDDATA[OBD_MAP] = ToyotaData[OBD_MAP + 1]; //MAF         4
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

void drawScreenSelector(void) {
  if (CurrentDisplayIDX == 1)
    drawAllData();
  else if (CurrentDisplayIDX == 2)
    mainscreen();
  else if (CurrentDisplayIDX == 3)
    summary_screen();
  else if (CurrentDisplayIDX == 4)
    settings();
} // end drawScreenSelector()

void writeHeader() {
  Serial1.println(F("Init complete! Launch date:"));
  Serial1.print(F("Launch date: ;"));
  Serial1.print(rtc.day());
  Serial1.print(F("."));
  Serial1.print(rtc.month());
  Serial1.print(F("."));
  Serial1.println(rtc.year());
  Serial1.print(F("Launch time: ;"));
  Serial1.print(rtc.hour());
  Serial1.print(F(":"));
  Serial1.print(rtc.minute());
  Serial1.print(F(":"));
  Serial1.println(rtc.second());
  Serial1.print(F("TOTAL KM: ;"));
  Serial1.print(total_km, 0);
  Serial1.print(F("; TOTAL AVG SPD: ;"));
  Serial1.print(total_avg_spd, 2);
  Serial1.print(F("; TOTAL AVG FUEL: ;"));
  Serial1.print(total_avg_fuel_consumption, 2);
  Serial1.print(F("; TOTAL  FUEL: ;"));
  Serial1.println(total_fuel_consumption, 2);
  Serial1.println(F("TIME;INJ TIME;IGN;IAC;RPM;MAF;ECT;TPS;SPD;VF1;ASE;CLOSED LOOP;Acceleration Enrichment;IDLE;NEUTRAL;OX;Trip Avg Fuel; Trip L; Total L; Fuel Volt"));
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
  Serial1.print(OBDFLAG[3]); //CLOSED LOOP
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
  Serial1.print(F(";"));
  Serial1.print(trip_fuel_consumption, 1);
  Serial1.print(F(";"));
  Serial1.print(total_fuel_consumption, 1);
  Serial1.print(F(";"));
  Serial1.print(fuel.volt, 2);
  Serial1.println();
}

void SaveToFram() {
  uint32_t check;
  if (millis() - last_log_time > 60000) // Запись данных в FRAM при остановке или
    //if (flagNulSpeed == false)                                      // каждые 3 минуты. Чтобы не потерять данные при движении на трассе
  {
    //if (_fram) //если FRAM инициализирована нормально
    { //Запись данных в еепром когда остановка авто
      Serial.println("SAVE FRAM");
      FRAM.write(0, trip_km);
      FRAM.write(4, trip_time);
      FRAM.read(4, check);                              //примитивная проверка корректности записи. Мало ли
      if (check != trip_time) {
        tft.fillScreen(BG_COLOR);
        tft.setFont(&DSEG7ModernMini_Bold13pt7b );
        tft.setCursor(16, 120);
        _fram = false;
        tft.print(F("FRAM ERROR!!"));
      }
      FRAM.write(8, total_km);
      FRAM.write(12, total_inj_dur);
      FRAM.write(16, trip_inj_dur);
      FRAM.write(20, total_time);
      FRAM.read(20, check);
      if (check != total_time) {
        tft.fillScreen(BG_COLOR);
        tft.setFont(&DSEG7ModernMini_Bold13pt7b );
        tft.setCursor(16, 120);
        _fram = false;
        tft.print(F("FRAM ERROR!!"));
      }
      // flagNulSpeed = true;                                  //запрет повторной записи
      last_log_time = millis();                             //чтобы не писать лишний раз
    }
    // if (OBDDATA[OBD_SPD] != 0) flagNulSpeed = false;     //начали двигаться - разрешаем запись
  }
}
void cleardata() {
  FRAM.write(0, 0.0f);
  FRAM.write(4, 0l);
  FRAM.write(16, 0.0f);
  if (CurrentDisplayIDX == 3) {
    FRAM.write(8, 0.0f);
    FRAM.write(12, 0.0f);
    FRAM.write(20, 0l);

  }
  delay(10);
  FRAM.read(0, trip_km);            //километраж поездки. Обнуляется кнопкой при активном 2м экране
  FRAM.read(4, trip_time);          //Время поездки. Обнуляется кнопкой при активном 2м экране
  FRAM.read(8, total_km);           //полный пробег. Обнуляется кнопкой при активном 3м экране
  FRAM.read(12, total_inj_dur);     //полная длительность впрыска. Обнуляется кнопкой при активном 3м экране
  FRAM.read(16, trip_inj_dur);      //длительность впрыска. Обнуляется кнопкой. Обнуляется кнопкой при активном 2м экране
  FRAM.read(20, total_time);        //полное время работы двигателя. Обнуляется кнопкой при активном 3м экране
}

void showtime()
{
  rtc.refresh();
  tft.setFont(&DSEG7ModernMini_Bold30pt7b);
  if (isActive == false)
  {
    tft.setCursor(57, 60);
    if (rtc.hour() < 10) tft.print(F("0"));
    tft.print(rtc.hour());
    tft.print(F(":"));
    if (rtc.minute() < 10) tft.print(F("0"));
    tft.print(rtc.minute());
  }
  if (omm != rtc.minute())
  {
    tft.setCursor(57, 60);
    tft.fillRect(56, 0, 205, 70, BG_COLOR);
    if (rtc.hour() < 10) tft.print(F("0"));
    tft.print(rtc.hour());
    tft.print(F(":"));
    if (rtc.minute() < 10) tft.print(F("0"));
    tft.print(rtc.minute());
    omm = rtc.minute();
  }
}


////////////////////////////////////////////////////////////////////////
/////SETTINGS //////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
void showcurrentsetting() {
  tft.fillRect(50, 37, 260, 114, BG_COLOR);
  tft.setTextColor(TEXT_COLOR);
  tft.setFont(&DSEG7ModernMini_Bold30pt7b);
  if (work_id == 2) {


    tft.setFont(&CPMono_v07_Plain9pt7b);
    printpos(weekdays[rtc.dayOfWeek()], 125, 110);        //вывод названия дней
  }
  else if (work_id == 6) {
    tft.setFont(&CPMono_v07_Plain9pt7b);
    printpos(colorscheme[DT[work_id]], 125, 110);
  }
  else if (work_id == 7)
  {
    tft.setFont(&CPMono_v07_Plain9pt7b);
    tft.setCursor(125,110);
    tft.print(Ls,7);
    //(Ls, 7, 125, 110);         //вывод коэфф форсунок 
    printpos(trip_inj_dur  * Ls / 1000.0 , 2, 125, 70);
  }
  else
    printpos(DT[work_id], 125, 110);          //вывод данных текущей настрйки
  tft.setFont(&CPMono_v07_Plain9pt7b);
  printpos(DT_NAME[work_id], 120, 145);     //описание настройки
}
/////////////////////////////////////////////////////////////////////////
void DTsync() {
  DT[0] = rtc.hour();
  DT[1] = rtc.minute();
  DT[2] = rtc.dayOfWeek();
  DT[3] = rtc.day();
  DT[4] = rtc.month();
  DT[5] = rtc.year();
}
////////////////////////////////////////////////////////////////////////
void settings(void) {
  rtc.refresh();
  if (isActive == false)
  {
    tft.fillScreen(BG_COLOR);
    work_id = 0;
    DTsync();
    isActive = true;
    showcurrentsetting();
    return;
  }
  switch (_id) {
    case 1: {
        if (DT[work_id] > 0 && work_id == 0) DT[work_id]--;   //часы минус
        if (DT[work_id] > 0 && work_id == 1) DT[work_id]--;   //минуты минус
        if (DT[work_id] > 1 && work_id == 2) DT[work_id]--;   //дни недели минус
        if (DT[work_id] > 1 && work_id == 3) DT[work_id]--;   //дни месяца минус
        if (DT[work_id] > 1 && work_id == 4) DT[work_id]--;   //месяц минус
        if (DT[work_id] > 0 && work_id == 5) DT[work_id]--;   //год минус
        if (DT[work_id] > 0 && work_id == 6) {
          DT[work_id]--;  //цветовая схема минус
          FRAM.write(40, DT[6]);
        }
        if (Ls > 0.0040000 && work_id == 7) {
          Ls -= 0.000010;  //калибровка форсунок минус
          FRAM.write(36, Ls);
        }


        rtc.set(0, DT[1], DT[0], DT[2] ,  DT[3], DT[4], DT[5]);
        rtc.refresh();
        DTsync();
        showcurrentsetting();
      } break;
    case 2: {
        if (DT[work_id] < 23 && work_id == 0) DT[work_id]++;    //часы плюс
        if (DT[work_id] < 59 && work_id == 1) DT[work_id]++;    //минуты плюс
        if (DT[work_id] < 7 && work_id == 2) DT[work_id]++;     //дни недели плюс
        if (DT[work_id] < 31 && work_id == 3) DT[work_id]++;    //дни месяца плюс
        if (DT[work_id] < 11 && work_id == 4) DT[work_id]++;    //месяц плюс
        if (DT[work_id] < 99 && work_id == 5) DT[work_id]++;    //год плюс
        if (DT[work_id] < 1 && work_id == 6) {
          DT[work_id]++;  //цветовая схема плюс
          FRAM.write(40, DT[6]);
        }
        if (Ls < 0.0049000 && work_id == 7) {
          Ls += 0.000010;  //калибровка форсунок плюс
          FRAM.write(36, Ls);
        }


        rtc.set(0, DT[1], DT[0], DT[2] ,  DT[3], DT[4], DT[5]);
        rtc.refresh();
        DTsync();
        showcurrentsetting();
      } break;
    case 3: {
        if (work_id > 0) work_id--;
        showcurrentsetting();
      } break;
    case 4: {
        if (work_id < 7) work_id++;
        showcurrentsetting();
      } break;
  }
  _id = 10;  //чтобы в новом цикле ничего само не выполнялось
  // delay(100) ;
}


void summary_screen(void) {
  tft.setTextColor(TEXT_COLOR);
  tft.setFont(&CPMono_v07_Plain9pt7b);
  if (isActive == false) {                    //форсирую перерисовку  после переключения экрана
    tft.fillScreen(BG_COLOR);
    isActive = true;        // ставлю флаг активного экрана чтобы не перерисовывать лишнее
    printpos(F("TRIP FUEL  L"), 10, 15);
    printpos(F("TRIP AVG   L"), 10, 35);
    printpos(F("TRIP      KM"), 10, 55);
    printpos(F("TRIP AVG SPD"), 10, 75);
    printpos(F("TOTAL FUEL L"), 10, 95);
    printpos(F("TOTAL AVG  L"), 10, 115);
    printpos(F("TOTAL     KM"), 10, 135);
    printpos(F("TOTAL AVGSPD"), 10, 155);
    printpos(F("FUEL TANK  V"), 10, 175);
    printpos(F("TOTAL   TIME"), 10, 195);
    printpos(trip_fuel_consumption, 2, 160, 15);
    printpos(trip_avg_fuel_consumption, 2, 160, 35);
    printpos(trip_km, 2, 160, 55);
    printpos(trip_avg_speed, 2, 160, 75);
    printpos(total_fuel_consumption, 2, 160, 95);
    printpos(total_avg_fuel_consumption, 2, 160, 115);
    printpos(total_km, 2, 160, 135);
    printpos(total_avg_spd, 2, 160, 155);
    printpos(fuel.volt, 2, 160, 175);
    printpos(TimeToString(total_time), 160, 195);
  }                        //конец перерисовки
  else
  {
    //данные, число разрядов, высота символов, длина стираемой строки, позицияХ, позицияУ
    send_tft2(trip_fuel_consumption, 2, 12, 160, 160, 15);
    send_tft2(trip_avg_fuel_consumption, 2, 12, 160, 160, 35);
    send_tft2(trip_km, 2, 12, 160, 160, 55);
    send_tft2(trip_avg_speed, 2, 12, 160, 160, 75);
    send_tft2(total_fuel_consumption, 2, 12, 160, 160, 95);
    send_tft2(total_avg_fuel_consumption, 2, 12, 160, 160, 115);
    send_tft2(total_km, 2, 12, 160, 160, 135);
    send_tft2(total_avg_spd, 2, 12, 160, 160, 155);
    send_tft2(fuel.volt, 2, 12, 160, 160, 175);
    tft.fillRect(157, 177, 138, 25, BG_COLOR);
    printpos(TimeToString(total_time), 160, 195);
  } // end void drawalldata
}

void backlightadjust() {
  {
    if (readIndex < 10)
    {
      TFT_LED.in += 4095 - analogRead(LIGHT_SENSOR_PIN); //набор данных с сенсора света для усреднения
      //fuel.in += analogRead(FUEL_PIN);
      readIndex++;
    }
    else
    {
      readIndex = 0;
      TFT_LED.in_med = TFT_LED.in / 10;   //усредняем
      //fuel.in_med = fuel.in / 10;
      TFT_LED.target = map(TFT_LED.in_med, 0, 4095, 0, 65535); //масштабируем
      //fuel.in = 0;
      TFT_LED.in = 0;
    }
    if (TFT_LED.current - TFT_LED.P > TFT_LED.target)            //+- 20 чтобы яркость не прыгала
      TFT_LED.current -= TFT_LED.P;                             //увеличиваем яркость с шагом 20
    if (TFT_LED.current + TFT_LED.P < TFT_LED.target)            // +- 20 чтобы яркость не прыгала
      TFT_LED.current += TFT_LED.P;                             //уменьшаем яркость с шагом 20
    if (TFT_LED.current > 62000) TFT_LED.current = 62000;       //минимальная яркость 62000
    if (TFT_LED.in_med < 400) TFT_LED.current = 200;       //минимальная яркость 62000
    pwmWrite(TFT_LED_PIN, TFT_LED.current);
  }
}

void readbuttons() {
  switch (S.read())
  {
    case MD_KeySwitch::KS_NULL:       break;
    case MD_KeySwitch::KS_PRESS:
      {
        if (CurrentDisplayIDX == 4) {     //если экран с настройками то
          isActive = false;
          CurrentDisplayIDX = 2;          //выход из экрана с настройками
          if (DT[6] == 0) { //Зеленая схема
            TEXT_COLOR = ILI9341_GREEN;
            BG_COLOR = ILI9341_BLACK;
            BLUE_COLOR = ILI9341_BLUE;
            RED_COLOR = ILI9341_RED;
            GT_COLOR = ILI9341_GREEN;
          }
          if (DT[6] == 1) { //ЧБ схема
            TEXT_COLOR = ILI9341_WHITE;
            BG_COLOR = ILI9341_BLACK;
            BLUE_COLOR = ILI9341_WHITE;
            RED_COLOR = ILI9341_WHITE;
            GT_COLOR = ILI9341_GREY;
          }
          drawScreenSelector();           //перерисовка экрана
        }
        break;
      case MD_KeySwitch::KS_DPRESS:     break;
      case MD_KeySwitch::KS_LONGPRESS:    //долгое нажание
        {
          cleardata();                    //очистка расхода и пробега
          drawScreenSelector();           //перерисовка экрана
        }
        break;
      case MD_KeySwitch::KS_RPTPRESS:   break;
      }
  }
  newKeyValue = GetKeyValue(BUTTON_ARRAY_PIN);
  if (keyValue != newKeyValue) {  // Если новое значение не совпадает со старым - реагируем на него
    keyValue = newKeyValue;       // Актуализируем переменную хранения состояния
    switch (keyValue) {
      case 2: {
          if (CurrentDisplayIDX < 4) {   //если не экран с настройками то
            CurrentDisplayIDX++;          //переходим на следующий экран
            isActive = false;             //флаг перерисовки всего экрана
            drawScreenSelector();         //функция вызова экрана
            break;
          }
          if (CurrentDisplayIDX == 4) {   //если экран с настройками то
            if (isActive == true)
              _id = 2;                      //настраиваем следующий параметр
            drawScreenSelector();         //перерисовка экрана настроек
          }
        } break;
      case 1: {
          if (CurrentDisplayIDX == 4) {   //если экран с настройками то
            _id = 1;                      //настраиваем предыдущий параметр
            drawScreenSelector();         //перерисовка экрана настроек
            break;
          }
          if (CurrentDisplayIDX > 1 ) {  //если не экран с настройками то
            CurrentDisplayIDX--;          //переходим на предыдущий экран
            isActive = false;             //флаг перерисовки всего экрана
            drawScreenSelector();         //функция вызова экрана
          }

        } break;
      case 3: {
          if (CurrentDisplayIDX == 4) {     //если экран с настройками то
            _id = 3;                        //увеличение текущего параметра
            drawScreenSelector();           //перерисовка экрана настроек
          }
        } break;
      case 4: {
          if (CurrentDisplayIDX == 4) {     //если экран с настройками то
            _id = 4;                        //уменьшение текущего параметра
            drawScreenSelector();           //перерисовка экрана настроек
          }
        } break;
    }
  }
}

void neutral()
{
  tft.fillRoundRect(146, 71, 26, 111, 5, BG_COLOR);
}
void firstgear()
{
  tft.fillRoundRect(146, 71, 26, 111, 5, BG_COLOR);
  tft.fillRoundRect(149, 155, 20, 20, 3, RED_COLOR);  //первая передача только с начала движения
}

void secondgear() {
  tft.fillRoundRect(146, 71, 26, 111, 5, BG_COLOR);
  tft.fillRoundRect(149, 155, 20, 20, 3, RED_COLOR);   //первая передача
  tft.fillRoundRect(149, 129, 20, 20, 3, RED_COLOR);   //вторая передача
}
void thirdgear(bool lock) {
  tft.fillRoundRect(146, 71, 26, 111, 5, BG_COLOR);
  if (lock == true)
    tft.fillRoundRect(146, 97, 26, 85, 5, GT_COLOR); //gt lock
  tft.fillRoundRect(149, 155, 20, 20, 3, RED_COLOR);   //первая передача
  tft.fillRoundRect(149, 129, 20, 20, 3, RED_COLOR);   //вторая передача
  tft.fillRoundRect(149, 103, 20, 20, 3, RED_COLOR);  //третья передача
}
void fourthgear(bool lock) {
  tft.fillRoundRect(146, 71, 26, 111, 5, BG_COLOR);
  if (lock == true)
    tft.fillRoundRect(146, 71, 26, 111, 5, GT_COLOR);
  tft.fillRoundRect(149, 155, 20, 20, 3, RED_COLOR);
  tft.fillRoundRect(149, 129, 20, 20, 3, RED_COLOR);
  tft.fillRoundRect(149, 103, 20, 20, 3, RED_COLOR);
  tft.fillRoundRect(149, 77, 20, 20, 3, RED_COLOR);
}

void printpos(const __FlashStringHelper * str, uint16_t posx, uint16_t posy) {
  tft.setCursor(posx, posy);
  tft.print(str);
}

void printpos(char *str, uint16_t posx, uint16_t posy) {
  tft.setCursor(posx, posy);
  tft.print(str);
}

void printpos(const char *str, uint16_t posx, uint16_t posy) {
  tft.setCursor(posx, posy);
  tft.print(str);
}

void printpos(float data, uint8_t num, uint16_t posx, uint16_t posy) {

  if (data < 0.01) data = 0.01;
  if (data > 999999) data = 999999;
  tft.setCursor(posx, posy);
  tft.print(data, num);
}
void printpos(uint32_t data, uint16_t posx, uint16_t posy) {
  tft.setCursor(posx, posy);
  tft.print(data);
}


void send_tft(String str, uint8_t advance, uint16_t posx, uint16_t posy) {
  int str_len = str.length() + 1;
  char char_array[str_len];
  str.toCharArray(char_array, str_len);
  int16_t x1, y1;
  uint16_t w1, h1;
  tft.setCursor(posx, posy);
  tft.getTextBounds(char_array, posx, posy, &x1, &y1, &w1, &h1);
  tft.fillRect(x1 - 1, y1, w1 + advance, h1, BG_COLOR);
  tft.print(str);
  str = "";
}

void send_tft2(float data, uint8_t num, uint8_t h, uint8_t w, uint16_t posx, uint16_t posy) {
  if (data < 0.01) data = 0.01;
  if (data > 999999) data = 999999;
  tft.setCursor(posx, posy);
  tft.fillRect(posx - 1, posy - h - 2, w, h + 4, BG_COLOR);
  tft.print(data, num);
}


//-------------------------------------------------------------------------------------------
void update_tft_float(uint8_t i, uint16_t XX, uint16_t YY) {
  if  (OLDOBDDATA[i] != OBDDATA[i]) {
    tft.setCursor(XX, YY);
    tft.fillRect(tft.getCursorX(), tft.getCursorY() - 16, 76, 19, BG_COLOR);
    tft.print(OBDDATA[i], 2);
  }
}

//-------------------------------------------------------------------------------------------
void update_tft_int(uint8_t i, uint16_t XX, uint16_t YY) {
  if  (OLDOBDDATA[i] != OBDDATA[i]) {
    tft.setCursor(XX, YY);
    tft.fillRect(tft.getCursorX(), tft.getCursorY() - 16, 76, 19, BG_COLOR);
    tft.print(int(OBDDATA[i]));
  }
}
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

