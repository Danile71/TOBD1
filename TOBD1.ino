// ToyotaOBD1_Reader
// In order to read the data from the OBD connector, short E1 + TE2. then to read the data connect to VF1.
// Note the data line output is 12V - connecting it directly to one of the arduino pins might damage (proabably) the board
// This is made for diaply with an OLED display using the U8glib - which allow wide range of display types with minor adjusments.
// Many thanks to GadgetFreak for the greate base code for the reasding of the data.
// If you want to use invert line - note the comments on the MY_HIGH and the INPUT_PULLUP in the SETUP void.

// display
#include "U8glib.h"
#include <EEPROM.h>
#include <Bounce2.h>
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);
#define DEBUG_OUTPUT true // for debug option - swith output to Serial
#define ENGINE_DATA_PIN 2 //VF1 PIN
#define TOGGLE_BTN_PIN 3 //screen change PIN
Bounce BTN_SCREEN = Bounce(); //создаем экземпляр класса Bounce
byte CurrentDisplayIDX;

//Расходомер
//#define SPEED_PIN A6 // Номер ноги для датчика скорости
//#define INJECTOR_PIN A7 // Номер ноги для форсунки
#define Ls 0.004 //производительсность форсунки литров в секунду


unsigned long t;

float total_duration;
float total_consumption;
float current_duration;

float current_run;
float total_run;

unsigned long current_time;
unsigned long total_time;

float total_avg_consumption;
float total_avg_speed;

//float avg_consumption;
//float avg_speed;

bool flagNulSpeed = true;
//Расходомер


#define LED_PIN          13
// I have inverted the Eng line using an Opto-Coupler, if yours isn't then reverse these low & high defines.
#define  MY_HIGH  HIGH //LOW
#define  MY_LOW   LOW //HIGH
#define  TOYOTA_MAX_BYTES  24
volatile uint8_t ToyotaNumBytes, ToyotaID;
volatile uint8_t ToyotaData[TOYOTA_MAX_BYTES];
volatile uint16_t ToyotaFailBit = 0;

// "names" for the OND data to make life easier
#define OBD_INJ 1 //Injector pulse width (INJ)
#define OBD_IGN 2 //Ignition timing angle (IGN)
#define OBD_IAC 3 //Idle Air Control (IAC)
#define OBD_RPM 4 //Engine speed (RPM)
#define OBD_MAP 5 //Manifold Absolute Pressure (MAP)
#define OBD_ECT 6 //Engine Coolant Temperature (ECT)
#define OBD_TPS 7 // Throttle Position Sensor (TPS)
#define OBD_SPD 8 //Speed (SPD)
#define OBD_OXSENS 9 // Лямбда 1
#define OBD_OXSENS2 10 // Лямбда 2 tst
boolean OBDConnected; // dfeine connection flag and last success packet - for lost connection function.
unsigned long OBDLastSuccessPacket;


void setup() {
  //float f = 0.00f;
  Serial.begin(115200);
  if (DEBUG_OUTPUT) {
    Serial.println("system Started");
  }
  //Serial.print("Read float from EEPROM: ");

  //Get the float data from the EEPROM at position 'eeAddress'
  //EEPROM.get(104, f);
  //Serial.println(f, 3);    //This may print 'ovf, nan' if the data inside the EEPROM is not a valid float.
  //EEPROM.get(200, f);
  //Serial.println(f, 3);    //This may print 'ovf, nan' if the data inside the EEPROM is not a valid float.
  //EEPROM.get(204, f);
  //Serial.println(f, 3);    //This may print 'ovf, nan' if the data inside the EEPROM is not a valid float.
  displayNoConnection(); // Display no connection

  pinMode(ENGINE_DATA_PIN, INPUT); // VF1 PIN
  pinMode(LED_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENGINE_DATA_PIN), ChangeState, CHANGE); //setup Interrupt for data line

  pinMode(TOGGLE_BTN_PIN, INPUT);           // кнопка СЛЕД. ЭКРАН
  BTN_SCREEN.attach(TOGGLE_BTN_PIN); // устанавливаем кнопку
  BTN_SCREEN.interval(50); // устанавливаем параметр stable interval = 50 мс

  // Set OBD to not connected
  OBDConnected = false;
  CurrentDisplayIDX = 1; // set to display 1
  //Расходомер
  EEPROM.get(104, total_run);
  EEPROM.get(108, total_time);
  EEPROM.get(200, total_duration);
  EEPROM.get(204, total_consumption);
  t = millis();
  //Расходомер
} // END VOID SETUP

void loop(void) {
  unsigned long new_t;
  unsigned int diff_t;
  if (ToyotaNumBytes > 0)  {    // if found bytes
    if (DEBUG_OUTPUT) debugdataoutput();
    new_t = millis();
    if (new_t > t) {
      diff_t = new_t - t;
      current_duration = getOBDdata(OBD_RPM) / 60 / 2 * 6 * diff_t / 1000 * getOBDdata(OBD_INJ); // Время открытия 6 форсунок за текущий такт ОБД данных (в миллисекундах) с текущими оборотами
      //ОБ/М           ОБ/С
      //форсунка срабатывает раз в 2 оборота КВ
      //6форсунок в с
      //время цикла мс в с. Получаем кол-во срабатываний за время цикла. Умножаем на время открытия форсунки, получаем время открытия 6 форсунок В МИЛЛИСЕКУНДАХ
      total_duration += current_duration; //Суммарное время, когда форсунка была открыта. EEPROM В МИЛЛИСЕКУНДАХ
      total_consumption = total_duration / 1000 * Ls; // Суммарный расход бензина по суммарному времени открытия форсунок. EEPROM. В ЛИТРАХ

      current_run = diff_t / 3600000 * getOBDdata(OBD_SPD); // Пройденное расстояние за текущий такт ОБД данных. В КМ
      total_run += current_run;                                 //Полное пройденное расстояние. EEPROM. В КМ

      current_time = diff_t;                              // текущее время цикла (миллисекунды)
      total_time += current_time;                              //полное пройденное время в миллисекундах лимит ~49 суток

      total_avg_consumption = total_consumption * 100 / total_run;  //    среднее потребление за все время - Л на 100км
      total_avg_speed = total_run / total_time * 3600000;           // средняя скорость за все время. км\ч

      Serial.println(total_duration, 3);
      Serial.println(total_consumption, 3);
      Serial.println(current_run, 3);
      Serial.println(current_time, 3);
      t = millis();
    }

    drawScreenSelector(); // draw screen
    OBDLastSuccessPacket = millis(); // set last success
    OBDConnected = true; // set connected to true
    ToyotaNumBytes = 0;     // reset the counter.
  } // end if (ToyotaNumBytes > 0)


  if (ToyotaFailBit > 0 && DEBUG_OUTPUT )  {
    debugfaildataoutput();  // if found FAILBIT and dbug
  }


  if (OBDLastSuccessPacket + 3500 < millis() && OBDConnected) {   //check for lost connection
    displayNoConnection();  // show no connection
    OBDConnected = false;  // set OBDConnected to false.
  } // end if loas conntcion


  if (getOBDdata(OBD_SPD) == 0 && flagNulSpeed == false)  {
    EEPROM.put(104, total_run);
    EEPROM.put(108, total_time);
    EEPROM.put(200, total_duration);
    EEPROM.put(204, total_consumption);
    flagNulSpeed = true;
  }

  if (getOBDdata(OBD_SPD) != 0) flagNulSpeed = false;

  ent();

} // end void loop

void drawScreenSelector(void) {
  if (CurrentDisplayIDX == 1) {
    drawSpeedRpm();
  } else if (CurrentDisplayIDX == 2) {
    drawAllData();
  } else if (CurrentDisplayIDX == 3) {
    drawExtraData();
  } else if (CurrentDisplayIDX == 4) {
    drawFuelConsuption();
  }  else if (CurrentDisplayIDX == 5) {
    drawExtraFlags();
  }

} // end drawScreenSelector()

void drawFuelConsuption(void) {
  u8g.setFont(u8g_font_unifont);
  u8g.firstPage();
  do {
    u8g.drawStr( 0, 17, "SPD" );
    u8g.setPrintPos(25, 17) ;
    u8g.print(total_avg_speed, 2);

    u8g.drawStr( 0, 32, "Fto");
    u8g.setPrintPos(25, 32) ;
    u8g.print(total_consumption, 2);

    u8g.drawStr( 0, 47, "KMt");
    u8g.setPrintPos(25, 47) ;
    u8g.print(total_run);

    u8g.drawStr( 0, 62, "Hto");
    u8g.setPrintPos(25, 62) ;
    u8g.print(total_time / 3600000, 2);

    u8g.drawStr( 65, 17, "Fav" );
    u8g.setPrintPos(92, 17) ;
    u8g.print(total_avg_consumption, 2);

    u8g.drawStr( 65, 32, "LPK");
    u8g.setPrintPos(92, 32) ;
    u8g.print(100 / getOBDdata(OBD_SPD) * (getOBDdata(OBD_INJ) * getOBDdata(OBD_RPM)*Ls * 0.18) );

    u8g.drawStr( 65, 47, "LPH");
    u8g.setPrintPos(92, 47) ;
    u8g.print( getOBDdata(OBD_INJ) * getOBDdata(OBD_RPM)*Ls * 0.18);

    u8g.drawStr( 65, 62, "Cdr");
    u8g.setPrintPos(92, 62) ;
    u8g.print( current_duration, 2);

    u8g.drawVLine(63, 0, 64);
  }
  while ( u8g.nextPage() );
}

void drawExtraData(void) {
  u8g.setFont(u8g_font_unifont);
  u8g.firstPage();
  do {
    u8g.drawStr( 0, 17, "OX1" );
    u8g.setPrintPos(25, 17) ;
    u8g.print(int(getOBDdata(OBD_OXSENS)));

    u8g.drawStr( 0, 32, "OX2");
    u8g.setPrintPos(25, 32) ;
    u8g.print( int(getOBDdata(OBD_OXSENS2)));

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

void drawSpeedRpm() {

  // define setytings for screen
  u8g.setFont(u8g_font_osb35n);
  u8g.setFontRefHeightExtendedText();
  u8g.setDefaultForegroundColor();
  u8g.setFontPosTop();

  // convert into to char so we can get the width
  char buf[4];
  itoa (int(getOBDdata(OBD_SPD)), buf, 10);

  // "calc" rpm bars to diaply - MAX 14
  int rpmToDisplay = int (float(getOBDdata(OBD_RPM)) / 500); // calc the rpm bars    then check to add +1 if pased the +250
  if (double(double(getOBDdata(OBD_RPM)) / 500.0) - int(getOBDdata(OBD_RPM) / 500) > 0.5) {
    rpmToDisplay += 1;
  } // end if */

  // picture loop
  u8g.firstPage();
  do {
    // draw box
    u8g.drawRFrame(20, 4, 90, 48, 7);
    // print speed
    u8g.drawStr(int(108 - u8g.getStrWidth(buf)), 10, buf);

    // draw RPM
    for (int i = 0; i < rpmToDisplay; i++) {
      u8g.drawBox(20 + ((4 + 1)*i) + i, 55, 4, 8);
    } // end for
  } while ( u8g.nextPage() ); // end picture loop

} //end void drawSpeedRpm()


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
      returnValue = ToyotaData[OBD_MAP] * 0.256; //(Вольт) (напряжение на расходомере)
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
      returnValue = (float)ToyotaData[OBD_OXSENS] * 5 / 256;
      break;
    //Коррекция второй половины
    case OBD_OXSENS2:// Lambda2 tst
      returnValue = (float)ToyotaData[OBD_OXSENS2] * 5 / 256;
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
            for (int i = 0; i <= bytepos; i++)
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

void debugdataoutput() {
  // output to Serial.
  Serial.print("ID=");
  Serial.print(ToyotaID);
  for (int i = 0; i < ToyotaNumBytes; i++)
  {
    Serial.print(", ");
    Serial.print(ToyotaData[i]);
  }
  Serial.println(".");
} // end void

void debugfaildataoutput() {
  Serial.print("FAIL = ");
  Serial.print(ToyotaFailBit);
  if (((ToyotaFailBit - 4) % 11) == 0)
    Serial.print(" (StartBit)");
  else if (((ToyotaFailBit - 4) % 11) > 8)
    Serial.print(" (StopBit)");
  Serial.println(".");
  ToyotaFailBit = 0;
} // end void

void ent() {
  //ПЕРЕКЛЮЧЕНИЕ ЭКРАНОВ
  if (BTN_SCREEN.update())
  { if (BTN_SCREEN.read() == HIGH) //если отпускаем
    {
      CurrentDisplayIDX++;
      //EEPROM.write(0,numScreen);
    }
    if (CurrentDisplayIDX > 4)
    {
      CurrentDisplayIDX = 1;
    }
    drawScreenSelector();
  }
}

