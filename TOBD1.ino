// ToyotaOBD1_Reader
// In order to read the data from the OBD connector, short E1 + TE2. then to read the data connect to VF1.
// Note the data line output is 12V - connecting it directly to one of the arduino pins might damage (proabably) the board
// This is made for diaply with an OLED display using the U8glib - which allow wide range of display types with minor adjusments.
// Many thanks to GadgetFreak for the greate base code for the reasding of the data.
// If you want to use invert line - note the comments on the MY_HIGH and the INPUT_PULLUP in the SETUP void.

#include "U8glib.h"
#include "SdFat.h"
#include <SPI.h>
#include <EEPROM.h>

#include <Bounce2.h>

//#define LOGGING_FULL    //Запись на SD всех данных
#define DEBUG_OUTPUT true // for debug option - swith output to Serial

//DEFINE пинов под входы-выходы
#define LED_PIN          13
#define ENGINE_DATA_PIN 2 //VF1 PIN
#define TOGGLE_BTN_PIN 4 //screen change PIN
#define INJECTOR_PIN 3 // Номер ноги для форсунки

//DEFINE констант расходомера
#define Ls 0.004 //производительсность форсунки литров в секунду
#define Lmks 0.000000004 //производительсность форсунки литров в микросекунду
#define Ncyl 6 //кол-во цилиндров

//DEFINE модуля записи на SD карту
#define FILE_BASE_NAME "Data"   //шаблон имени файла
#define error(msg) sd.errorHalt(F(msg)) //ошибки при работе с SD


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
#define OBD_OXSENS2 10 // Лямбда 2 


U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);
SdFat sd;
SdFile file;

Bounce BTN_SCREEN = Bounce(); //создаем экземпляр класса Bounce
byte CurrentDisplayIDX;

float total_consumption = 0;
float current_consumption = 0;
float current_run = 0;
float total_run = 0;
float total_avg_consumption;
float avg_consumption;
float total_avg_consumption_inj;
float avg_consumption_inj;
float total_avg_speed;
float avg_speed;
unsigned long current_time = 0;
unsigned long total_time = 0;
unsigned long t;
bool flagNulSpeed = true;
volatile unsigned long Injector_Open_Duration = 0;
volatile unsigned long INJ_TIME;
volatile unsigned long InjectorTime1;
volatile unsigned long InjectorTime2;

float total_consumption_inj, current_consumption_inj;
volatile uint8_t ToyotaNumBytes, ToyotaID, ToyotaData[TOYOTA_MAX_BYTES];
volatile uint16_t ToyotaFailBit = 0;

// "names" for the OBD data to make life easier
boolean OBDConnected; // dfeine connection flag and last success packet - for lost connection function.
unsigned long OBDLastSuccessPacket;


void setup() {
  char fileName[13] = FILE_BASE_NAME "00.csv";
  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  noInterrupts();
  Serial.begin(115200);
  if (DEBUG_OUTPUT) {
    Serial.println("system Started");
  }
  if (!sd.begin(SS, SD_SCK_MHZ(50))) {
    sd.initErrorHalt();
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


  if (!file.open(fileName, O_CREAT | O_WRITE )) {
    error("file.open");
  }
  writeHeader();
  //displayNoConnection(); // Display no connection

  // set and initialize the TIMER1
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // set entire TCCR1B register to 0
  TCCR1B |= (1 << CS12);//prescaler 256 --> Increment time = 256/16.000.000= 16us
  TIMSK1 |= (1 << TOIE1); // enable Timer1 overflow interrupt
  TCNT1 = 3036; // counter initial value so as to overflow every 1sec: 65536 - 3036 = 62500 * 16us = 1sec (65536 maximum value of the timer)



  pinMode(ENGINE_DATA_PIN, INPUT); // VF1 PIN
  pinMode(INJECTOR_PIN, INPUT); // Injector PIN
  pinMode(LED_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENGINE_DATA_PIN), ChangeState, CHANGE); //setup Interrupt for data line
  attachInterrupt(digitalPinToInterrupt(INJECTOR_PIN), InjectorTime, CHANGE); //setup Interrupt for data line

  pinMode(TOGGLE_BTN_PIN, INPUT);           // кнопка СЛЕД. ЭКРАН
  BTN_SCREEN.attach(TOGGLE_BTN_PIN); // устанавливаем кнопку
  BTN_SCREEN.interval(50); // устанавливаем параметр stable interval = 50 мс

  // Set OBD to not connected
  //OBDConnected = false;
  CurrentDisplayIDX = 1; // set to display 1
  //Расходомер
  EEPROM.get(104, total_run);
  EEPROM.get(108, total_time);
  EEPROM.get(200, total_consumption_inj);
  EEPROM.get(204, total_consumption);
  t = millis();
  interrupts();
  delay(10);
  //Расходомер
} // END VOID SETUP





void loop(void) {
  unsigned long new_t;
  unsigned int diff_t;
  ent();
  if (ToyotaNumBytes > 0)  {    // if found bytes
    new_t = millis();
    if (new_t > t) {
      diff_t = new_t - t;
      current_consumption = getOBDdata(OBD_RPM) / 60 / 2 * Ncyl * (float)diff_t / 1000 * getOBDdata(OBD_INJ) / 1000 * Ls; // Потребление 6 форсунок за текущий такт ОБД данных (в литрах) с текущими оборотами c момента включения
      total_consumption += current_consumption;         //потребление бензина за все время в литрах. EEPROM
      //ОБ/М           ОБ/С
      //форсунка срабатывает раз в 2 оборота КВ
      //6форсунок в с
      //время цикла мс в с. Получаем кол-во срабатываний за время цикла. Умножаем на время открытия форсунки, получаем время открытия 6 форсунок В МИЛЛИСЕКУНДАХ

      current_run += (float)diff_t / 3600000 * getOBDdata(OBD_SPD);  //Пройденное расстояние с момента включения. В КМ
      total_run += (float)diff_t / 3600000 * getOBDdata(OBD_SPD);    //Полное пройденное расстояние. EEPROM. В КМ

      total_time += diff_t;                         //полное пройденное время в миллисекундах лимит ~49 суток. EEPROM
      current_time += diff_t;             //Время в пути в миллисекундах с момента включения

      total_avg_consumption = total_consumption * 100 / total_run;  //    среднее потребление за все время - Л на 100км
      total_avg_speed = total_run / (float)total_time * 3600000;           // средняя скорость за все время. км\ч

      avg_speed = (current_run / (float)current_time * 3600000) ;       //average speed
      avg_consumption_inj = 100 * current_consumption_inj / current_run; //average l/100km for unleaded fuel
      avg_consumption = 100 * current_consumption / current_run;
      t = millis();
    }
    drawScreenSelector(); // draw screen
    logData();
    OBDLastSuccessPacket = millis(); // set last success
    OBDConnected = true; // set connected to true
    ToyotaNumBytes = 0;     // reset the counter.
  } // end if (ToyotaNumBytes > 0)

  /*
    if (OBDLastSuccessPacket + 3500 < millis() && OBDConnected) {   //check for lost connection
      displayNoConnection();  // show no connection
      OBDConnected = false;  // set OBDConnected to false.
    } // end if loas conntcion
  */
  if (getOBDdata(OBD_SPD) == 0 && flagNulSpeed == false)  {
    EEPROM.put(104, total_run);
    EEPROM.put(108, total_time);
    EEPROM.put(200, total_consumption_inj);
    EEPROM.put(204, total_consumption);
    flagNulSpeed = true;
  }
  if (millis() % 250 < 50) logData();
  if (getOBDdata(OBD_SPD) != 0) flagNulSpeed = false;
} // end void loop

void drawScreenSelector(void) {
  if (CurrentDisplayIDX == 1)  drawAllData();
  else if (CurrentDisplayIDX == 2) drawExtraData();
  else if (CurrentDisplayIDX == 3) drawFuelConsuption();
  else if (CurrentDisplayIDX == 4) drawExtraFlags();
} // end drawScreenSelector()

void drawFuelConsuption(void) {
  u8g.setFont(u8g_font_unifont);
  u8g.firstPage();
  do {
    u8g.drawStr( 0, 17, "Ft1" );
    u8g.setPrintPos(25, 17) ;
    u8g.print(total_consumption_inj, 1);

    //u8g.drawStr( 0, 17, "SPD" );
    //u8g.setPrintPos(25, 17) ;
    //u8g.print(total_avg_speed, 2);


    u8g.drawStr( 0, 32, "Ft2");
    u8g.setPrintPos(25, 32) ;
    u8g.print(total_consumption, 1);

    u8g.drawStr( 0, 47, "KMt");
    u8g.setPrintPos(25, 47) ;
    u8g.print(total_run);

    u8g.drawStr( 0, 62, "Hto");
    u8g.setPrintPos(25, 62) ;
    u8g.print(float(total_time) / 3600000, 1);

    u8g.drawStr( 65, 17, "Fav" );
    u8g.setPrintPos(92, 17) ;
    u8g.print(total_avg_consumption, 1);

    u8g.drawStr( 65, 32, "LPK");
    u8g.setPrintPos(92, 32) ;
    u8g.print(100 / getOBDdata(OBD_SPD) * (getOBDdata(OBD_INJ) * getOBDdata(OBD_RPM)*Ls * 0.18) );

    u8g.drawStr( 65, 47, "LPH");
    u8g.setPrintPos(92, 47) ;
    u8g.print( getOBDdata(OBD_INJ) * getOBDdata(OBD_RPM)*Ls * 0.18);

    u8g.drawStr( 65, 62, "Cdr");
    u8g.setPrintPos(92, 62) ;
    u8g.print( current_consumption, 1);

    u8g.drawVLine(63, 0, 64);
  }
  while ( u8g.nextPage() );
}

void drawExtraData(void) {
  u8g.setFont(u8g_font_unifont);
  u8g.firstPage();
  do {
    u8g.drawStr( 0, 17, "O2s" );
    u8g.setPrintPos(25, 17) ;
    u8g.print(int(getOBDdata(OBD_OXSENS)));

    //u8g.drawStr( 0, 32, "OX2");
    //u8g.setPrintPos(25, 32) ;
    //u8g.print( );

    u8g.drawStr( 0, 47, "SEN");
    u8g.setPrintPos(25, 47) ;
    u8g.print( int(getOBDdata(11)));

    u8g.drawStr( 0, 62, "CLD");
    u8g.setPrintPos(25, 62) ;
    u8g.print( int(getOBDdata(12)));

    u8g.drawStr( 65, 17, "DET" );
    u8g.setPrintPos(92, 17) ;
    u8g.print( int(getOBDdata(13)));

    u8g.drawStr( 65, 32, "FE1");
    u8g.setPrintPos(92, 32) ;
    u8g.print( int(getOBDdata(14)));

    u8g.drawStr( 65, 47, "FE2");
    u8g.setPrintPos(92, 47) ;
    u8g.print( int(getOBDdata(15)));

    u8g.drawStr( 65, 62, "STR");
    u8g.setPrintPos(92, 62) ;
    u8g.print( int(getOBDdata(16)));
    u8g.drawVLine(63, 0, 64);
  }
  while ( u8g.nextPage() );
}


void drawExtraFlags(void) {
  u8g.setFont(u8g_font_unifont);
  u8g.firstPage();
  do {
    u8g.drawStr( 0, 17, "IDL" );
    u8g.setPrintPos(25, 17) ;
    u8g.print(int(getOBDdata(17)));

    u8g.drawStr( 0, 32, "CND");
    u8g.setPrintPos(25, 32) ;
    u8g.print( int(getOBDdata(18)));

    u8g.drawStr( 0, 47, "NEU");
    u8g.setPrintPos(25, 47) ;
    u8g.print( int(getOBDdata(19)));

    u8g.drawStr( 0, 62, "EN1");
    u8g.setPrintPos(25, 62) ;
    u8g.print( int(getOBDdata(20)));

    u8g.drawStr( 65, 17, "EN2" );
    u8g.setPrintPos(92, 17) ;
    u8g.print( int(getOBDdata(21)));

    u8g.drawVLine(63, 0, 64);
  }
  while ( u8g.nextPage() );
}



void drawAllData(void) {
  // graphic commands to redraw the complete screen should be placed here
  u8g.setFont(u8g_font_unifont);
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

void displayNoConnection() {
  u8g.setFont(u8g_font_unifont);
  u8g.setFontRefHeightExtendedText();
  u8g.setDefaultForegroundColor();
  u8g.setFontPosTop();
  // picture loop
  u8g.firstPage();
  do {
    // draw box
    u8g.drawRFrame(0, 0, 127, 63, 7);
    u8g.drawStr(15, 25, "NO SIGNAL");
  } while ( u8g.nextPage() );

} // end void

float getOBDdata(byte OBDdataIDX) {
  // define return value
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
      returnValue = ToyotaData[OBD_MAP]; //Сырые данные
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
      returnValue = ToyotaData[OBD_TPS] / 2;
      break;
    case OBD_SPD: // Скорость автомобиля (км/час)
      returnValue = ToyotaData[OBD_SPD];
      break;

    //  Коррекция для рядных/ коррекция первой половины
    case OBD_OXSENS:
      returnValue = (float)ToyotaData[OBD_OXSENS] * 0.01953125;
      break;
    //Коррекция второй половины
    case OBD_OXSENS2:// Lambda2 tst
      returnValue = (float)ToyotaData[OBD_OXSENS2] * 0.01953125;
      break;
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
      returnValue = bitRead(ToyotaData[13], 3); //Нейтраль 1-Да
      break;
    case 20:
      returnValue = bitRead(ToyotaData[14], 4); //Смесь  первой половины 1-Богатая, 0-Бедная
      break;
    case 21:
      returnValue = bitRead(ToyotaData[14], 5); //Смесь второй половины 1-Богатая, 0-Бедная
      break;
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

void ent() {
  //ПЕРЕКЛЮЧЕНИЕ ЭКРАНОВ
  if (BTN_SCREEN.update())
  { if (BTN_SCREEN.read() == HIGH) //если отпускаем
      CurrentDisplayIDX++;
    if (CurrentDisplayIDX > 4) CurrentDisplayIDX = 1;
    drawScreenSelector();
  }
}


void writeHeader() {
#ifdef LOGGING_FULL
  file.print(F("INJ;INJ_TEST;IGN;IAC;RPM;MAP;ECT;TPS;SPD;OXSENS;ASE;COLD;DET;OXf;Re-ENRICHMENT;STARTER;IDLE;A/C;NEUTRAL;Ex1;Ex2;AVG SPD;LPK_OBD;LPH_OBD;LPH_INJ;TOTAL_OBD;TOTAL_INJ;AVG_OBD;AVG_INJ"));
#else
  file.print(F("INJ;INJ_TEST;IGN;IAC;RPM;MAP;ECT;TPS;SPD;O2SENS;AVG SPD;LPK_OBD;LPH_OBD;LPH_INJ;TOTAL_OBD;TOTAL_INJ;AVG_OBD;AVG_INJ"));
#endif
  file.println();
}

void logData() {
  file.print(getOBDdata(OBD_INJ)); file.write(';'); file.print(float(INJ_TIME) / 1000); file.write(';'); file.print(getOBDdata(OBD_IGN));  file.write(';');  file.print(getOBDdata(OBD_IAC));  file.write(';');
  file.print(getOBDdata(OBD_RPM)); file.write(';'); file.print(getOBDdata(OBD_MAP));  file.write(';'); file.print(getOBDdata(OBD_ECT));  file.write(';');
  file.print(getOBDdata(OBD_TPS)); file.write(';');  file.print(getOBDdata(OBD_SPD));  file.write(';'); file.print(getOBDdata(OBD_OXSENS)); file.write(';');
#ifdef LOGGING_FULL
  file.print(getOBDdata(11)); file.write(';'); file.print(getOBDdata(12)); file.write(';'); file.print(getOBDdata(13)); file.write(';'); file.print(getOBDdata(14)); file.write(';');
  file.print(getOBDdata(15)); file.write(';'); file.print(getOBDdata(16)); file.write(';'); file.print(getOBDdata(17)); file.write(';'); file.print(getOBDdata(18)); file.write(';');
  file.print(getOBDdata(19)); file.write(';'); file.print(getOBDdata(20)); file.write(';'); file.print(getOBDdata(21)); file.write(';');
#endif
  file.print(total_avg_speed); file.write(';');                                                                   //!AVG_SPD
  file.print(100 / getOBDdata(OBD_SPD) * (getOBDdata(OBD_INJ) * getOBDdata(OBD_RPM)*Ls * 0.18)); file.write(';');  //LPK_OBD      ok
  file.print(getOBDdata(OBD_INJ) * getOBDdata(OBD_RPM)*Ls * 0.18); file.write(';');                                //LPH_OBD    ok
  file.print(float(INJ_TIME) / 1000 * getOBDdata(OBD_RPM)*Ls * 0.18); file.write(';');                          //LPH_INJ     ok
  file.print(total_consumption); file.write(';');   //TOTAL_OBD     ok
  file.print(total_consumption_inj); file.write(';'); //TOTAL_INJ   ok
  file.print(avg_consumption); file.write(';');   //!AVG_OBD
  file.print(avg_consumption_inj);      //!AVG_INJ
  file.println();

  if (!file.sync() || file.getWriteError())  error("write error");
}

void InjectorTime() { // it is called every time a change occurs at the gasoline injector signal and calculates gasoline injector opening time, during the 1sec interval
  if (digitalRead(INJECTOR_PIN) == LOW) {
    InjectorTime1 = micros();
  }
  if (digitalRead(INJECTOR_PIN) == HIGH) {
    InjectorTime2 = micros();
  }
  if (InjectorTime2 > InjectorTime1 && InjectorTime2 - InjectorTime1 > 15000) {  // Замечено что при резком сбросе оборотов всплески > 15мс=15000мкс. Вангую что это торможение двигателем и форсунки отключается. 
    INJ_TIME = 0;     // в этом случае длительность впрыска равно нулю
  }
  else                                                            // в противном случае
  {
    Injector_Open_Duration += (InjectorTime2 - InjectorTime1);      //Суммирование времени открытия форсунки для подсчета суммарного расхода. в микросекундах.
    INJ_TIME = InjectorTime2 - InjectorTime1;                     //длительность впрыска
  }
}

ISR(TIMER1_OVF_vect) { //TIMER1 overflow interrupt -- occurs every 1sec -- it holds the time (in seconds) and also prevents the overflowing of some variables
  total_consumption_inj += Injector_Open_Duration * Lmks;    // Полный расход бензина в Литрах. Хранится в EEPROM.
  current_consumption_inj += Injector_Open_Duration * Lmks;   // Расход бензина за поездку в литрах
  Injector_Open_Duration = 0;
  TCNT1 = 3036;
}
