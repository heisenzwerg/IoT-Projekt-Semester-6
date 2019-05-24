#define BLYNK_PRINT Serial
#define EspSerial Serial1

//Relay for pump
#define RELAY_ON 1      // Define relay on pin state
#define RELAY_OFF 0     // Define relay off pin state
#define Relay  25       // Arduino pin where the relay connects

#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>
#include "SR04.h"
#include <TroykaDHT.h>
#include <Wire.h>
#include "RTClib.h"
#include <Servo.h>
#include <SPI.h>
#include <MFRC522.h>
//#include <LiquidCrystal.h>     //für 2 Zeilen LCD
#include <LiquidCrystal_I2C.h>

//RFID
#define SS_PIN 53
#define RST_PIN 49
MFRC522 mfrc522(SS_PIN, RST_PIN);

//Servo
Servo servoGarage;
Servo servoFenster;
bool position_garage;
bool position_fenster;
bool pos_garage;
bool pos_fenster;
int angleGarage;
int angleFenster;
int garageOffen = 11;
int fensterOffen = 10;
int fensterZu = 60;
int garageZu = 112;

//Blynk-------------------------------------------
char auth[] = "86e2c32fecea494e8fd6eef75665699a";
char ssid[] = "iPhone";
char pass[] = "10201020";

//#define BLYNK_PRINT Serial
//#define EspSerial Serial1
#define ESP8266_BAUD 115200
ESP8266 wifi(&EspSerial);

BlynkTimer timer;


//Display 2 zeilig--------------------------------
//const int rs = 40, en = 41, d4 = 44, d5 = 45, d6 = 46, d7 = 47;
//LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
// Nutzt PIN's 40, 41, 44 bis 47 zur Kommunikation. KEIN I2C!

//Display 4 zeilig--------------------------------
LiquidCrystal_I2C lcd(0x27, 20, 4);
//Nutzt PIN's 20 (SDA) und 21 (SCL) für I2C. Funktioniert gleichzeitig mit RTC per I2C auf gleichen PIN's.

//Temperatur_Feuchtigkeit-------------------------
DHT dht(2, DHT11);
static unsigned long millis_temp;
int s_temperatur;
int s_feuchtigkeit;


//Wind--------------------------------------------              /////////////////////////////////Hier s_regen
String s_wind;
bool s_regen;
String s_windrichtung;

//Füllstand---------------------------------------
#define TRIG_PIN 26
#define ECHO_PIN 27

SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);
float s_fuellstand;
static unsigned long millis_ultrasonic;


//RTC---------------------------------------------
RTC_DS3231 rtc;
DateTime now;

//Alarm
bool check_ob_alarm_noetig = 0;
bool alarmanlage_an_aus = 0;
bool s_rfid;
bool ist_alarm_aktiv = 0;

//Power
double  s_wattstunden = 0;
float s_wattstunden1 = 0;


//Reed
bool s_reed_1 = 1;
bool s_reed_2 = 1;
bool s_reed_3 = 1;

//Smarte Logik
bool o_smart = 1;  //Ist an (1), oder aus

//Pumpe
bool a_pumpe = 0;
long dauer_pumpe = 0;
int s_durchfluss = 0;
bool pumpe_on_off = 0;

//Bewegungsmelder
int bewegungsmelder1 = 22;
int bewegungsmelder2 = 23;
bool s_beweg_1 = 0;
bool s_beweg_2 = 0;


//LEDs
int led1 = 11;
int led2 = 12;
int led3 = 13;
int s_helligkeit_led1 = 0;
int s_helligkeit_led2 = 0;
int s_helligkeit_led3 = 0;

//Zeit
unsigned long currentMillis;
unsigned long timer_1_5000ms;
unsigned long timer_2_5000ms;
unsigned long millis_pumpe;

//Helligkeit
int s_helligkeit;

//Gas Rauch
bool s_rauch;

void setup()
{
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  Serial.begin(9600);
  delay(10);

  servoGarage.attach(14);
  servoFenster.attach(15);
  servoGarage.write(garageZu);
  servoFenster.write(fensterZu);

  //RTC---------------------------------------------
  rtc.begin();
  SPI.begin();
  mfrc522.PCD_Init();

  //rtc.adjust(DateTime(2019, 5, 20, 16, 20, 0));

  pinMode(29, OUTPUT);
  digitalWrite(29, LOW);
  pinMode(32, OUTPUT);
  digitalWrite(32, LOW);
  pinMode(33, OUTPUT);
  digitalWrite(33, LOW);

  //Display 2 zeilig--------------------------------
  //lcd.begin(16, 2);

  //Display 4 zeilig--------------------------------
  lcd.init();
  lcd.backlight();
  lcd.print("Bootvorgang");
  delay(500);
  lcd.setCursor(0, 0);
  lcd.print("Bootvorgang.");
  delay(500);
  lcd.setCursor(0, 0);
  lcd.print("Bootvorgang..");
  delay(500);
  lcd.setCursor(0, 0);
  lcd.print("Bootvorgang...");


  //Füllstand---------------------------------------
  millis_ultrasonic = millis( );


  //Temperatur_Feuchtigkeit-------------------------
  dht.begin();
  millis_temp = millis( );


  //Blynk-------------------------------------------
  EspSerial.begin(ESP8266_BAUD);
  delay(10);
  Blynk.begin(auth, wifi, ssid, pass);
  timer.setInterval(2000L, pushDataToCloud);    //HIER INTERVALL FÜR DATEN-UPLOAD ANGEBEN. Z.Z. ALLE 2 SEKUNDEN


  //Display-----------------------------------------
  lcd.setCursor(0, 1);
  lcd.print("Verbunden mit WiFi:");
  lcd.setCursor(0, 2);
  lcd.print(ssid);
  delay(2000);
  lcd.clear();
  Blynk.run();
  set_alarm(0);

  Blynk.virtualWrite(V7, 0);
  Blynk.virtualWrite(V8, 0);


  //Pumpe
  digitalWrite(Relay, RELAY_OFF);      // initialise the relay to off
  pinMode(Relay, OUTPUT);

  //Bewegungsmelder
  pinMode (bewegungsmelder1, INPUT);
  pinMode (bewegungsmelder2, INPUT);

  //LED
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);

  analogWrite(led1, LOW);
  analogWrite(led2, LOW);
  analogWrite(led3, LOW);

  Blynk.virtualWrite(V9, 0);

}

void loop()
{
  update_time();      //aktualisiert die Zeit
  write_time();       //schreibt Zeit auf LCD
  s_wind = get_wind();         //gibt String mit Beschreibung der Windstärke zurück
  s_windrichtung = get_windrichtung();
  s_regen = get_rain();         //gibt boolschen Wert zurück, 1 ist Regen, 0 ist kein Regen
  s_rfid = RFID();
  s_rauch = get_rauch();
  s_durchfluss = get_durchfluss();
  s_wattstunden = get_power();
  //s_wattstunden1 = get_leistung();

  s_helligkeit = get_helligkeit();
  s_reed_1 = digitalRead(3);
  o_smart = 1;
  check_ob_alarm_noetig = get_alarm();

  if (check_ob_alarm_noetig == 1 && ist_alarm_aktiv == 0) {
    alarm_aktiv(1);
    Blynk.notify("Verdächtige Aktivitäten in ihrem Heim!");
  }

  if (s_rauch == 1 && ist_alarm_aktiv == 0) {
    alarm_aktiv(1);
    Blynk.notify("Gas/Rauchalarm in ihrem Heim!");
  }

  if (s_regen == 1 && position_fenster == 1) {
    fenster(0);
  }

  if (s_helligkeit == 0) {
    set_LED(255);
  }

  if (digitalRead(37) == HIGH) {
    if (s_helligkeit_led3 != 255) {
      set_LED(255);
    }
    else if (s_helligkeit_led3 == 255) {
      set_LED(0);
    }
    delay(1000);
  }

  if (s_rfid == 1 && ist_alarm_aktiv == 1) {
    alarm_aktiv(0);
    digitalWrite(32, HIGH);
    delay(1100);
    digitalWrite(32, LOW);
  }
  if (millis() - millis_ultrasonic > 4000)
  {
    int temp_fuell = get_fuellstand();
    if (temp_fuell < 500) {
      s_fuellstand = temp_fuell;
    }
    if (s_fuellstand > 330) {
      PUMPEN(1);
      delay(1000);
      PUMPEN(0);
    }
  }

  if (millis() - millis_temp > 6000)
  {
    s_temperatur = get_temperatur();          //immer nur beide zusammen und in dieser Reihenfolge rufen!
    s_feuchtigkeit = get_feuchtigkeit();      //immer nur beide zusammen und in dieser Reihenfolge rufen!
  }

  if (millis() - millis_pumpe > 4000 && pumpe_on_off == 1)
  {
    PUMPEN(0);
  }

  //Aufrufen Smarte Logik
  if (o_smart == 1)
  {
    SmartLogic();
  }

  lcd.setCursor(0,1);
  lcd.print("Temperatur: ");
  lcd.print(s_temperatur);
  lcd.print("\337C");

  lcd.setCursor(0,2);
  lcd.print("Feuchtigkeit: ");
  lcd.print(s_feuchtigkeit);
  lcd.print("%");

  lcd.setCursor(0,3);
  lcd.print("WiFi: ");
  lcd.print(ssid);


  //Systenzeit messen
  currentMillis = millis();

  //Blynk-------------------------------------------
  Blynk.run();
  timer.run();
}

BLYNK_WRITE(V3)
{
  int pinValue = param.asInt(); // pinValue kann hier beispielsweise deine globale Variable für die Alarmanlage sein (on/off)

  if (alarmanlage_an_aus == 0) {
    set_alarm(1);
  }
  else if (alarmanlage_an_aus == 1) {
    set_alarm(0);
  }
}

BLYNK_WRITE(V6)
{
  int pinValue = param.asInt(); // pinValue kann hier beispielsweise deine globale Variable für die Alarmanlage sein (on/off)
  if (pinValue == 1)            //Bei binären Zuständen
  {
    PUMPEN(1);
  }
  if (pinValue == 0)
  {
    PUMPEN(0);
  }
}

BLYNK_WRITE(V7)
{
  int pinValue = param.asInt(); // pinValue kann hier beispielsweise deine globale Variable für die Alarmanlage sein (on/off)
  if (pinValue == 1)            //Bei binären Zuständen
  {
    garage(1);
  }
  if (pinValue == 0)
  {
    garage(0);

  }
}

BLYNK_WRITE(V8)
{
  int pinValue = param.asInt(); // pinValue kann hier beispielsweise deine globale Variable für die Alarmanlage sein (on/off)
  if (pinValue == 1)            //Bei binären Zuständen
  {
    fenster(1);
  }
  if (pinValue == 0)
  {
    fenster(0);

  }
}

BLYNK_WRITE(V9)
{
  int pinValue = param.asInt(); // pinValue kann hier beispielsweise deine globale Variable für die Alarmanlage sein (on/off)

  set_LED(pinValue);
}

void pushDataToCloud()
{
  //  char buf_feuchtigkeit[10];
  //  sprintf(buf_feuchtigkeit, "%d%%", s_feuchtigkeit);
  //
  //  char buf_temperatur[10];
  //  sprintf(buf_temperatur, "%d°C", s_temperatur);

  char buf_uptime[10];
  if (millis() < 120000) {
    sprintf(buf_uptime, "%ds", millis() / 1000);
  }
  else {
    sprintf(buf_uptime, "%d min", millis() / 60000);
  }

  Blynk.virtualWrite(V0, s_temperatur);
  Blynk.virtualWrite(V2, s_feuchtigkeit);
  Blynk.virtualWrite(V1, buf_uptime);
  Blynk.virtualWrite(V4, s_fuellstand);
  Blynk.virtualWrite(V5, s_wind);
  Blynk.virtualWrite(V11, s_windrichtung);
  Blynk.virtualWrite(V12, s_wattstunden1);
  Blynk.virtualWrite(V13, s_durchfluss);
  if (s_regen == 0) {
    Blynk.virtualWrite(V14, "Nein");
  }
  else if (s_regen == 1) {
    Blynk.virtualWrite(V14, "Ja");

  }
}

void set_alarm(int alarm) {
  if (alarm == 1) {
    alarmanlage_an_aus = 1;
    Blynk.virtualWrite(V15, "Alarmanlage an");
    Blynk.virtualWrite(V3, 1);
    digitalWrite(32, HIGH);
    delay(250);
    digitalWrite(32, LOW);
    garage(0);
    fenster(0);
  }

  if (alarm == 0) {
    alarmanlage_an_aus = 0;
    alarm_aktiv(0);
    Blynk.virtualWrite(V15, "Alarmanlage aus");
    Blynk.virtualWrite(V3, 0);
    digitalWrite(33, HIGH);
    delay(800);
    digitalWrite(33, LOW);
  }

}

bool get_alarm() {

  if (check_ob_alarm_noetig == 1) {
    return 1;
  }

  if (alarmanlage_an_aus == 0) {
    return 0;
  }

  if (s_reed_1 == 0) {
    return 1;
  }

  //  if (s_reed_2 == 0) {
  //    return 1;
  //  }

  return 0;
}

void alarm_aktiv(bool aktiv) {
  if (aktiv == 1) {
    digitalWrite(29, HIGH);
    ist_alarm_aktiv = 1;
  }
  else if (aktiv == 0) {
    check_ob_alarm_noetig = 0;
    ist_alarm_aktiv = 0;
    digitalWrite(29, LOW);
  }
}

void garage(bool pos) {

  if (pos == 0) {

    if (position_garage == 0) {
      return;
    }
    else {
      for (angleGarage = garageOffen; angleGarage < garageZu; angleGarage = angleGarage + 2)
      {
        servoGarage.write(angleGarage);
        delay(15);
      }
      position_garage = 0;
    }
  }

  if (pos == 1) {
    if (position_garage == 1) {
      return;
    }
    else {
      for (angleGarage = garageZu; angleGarage > garageOffen; angleGarage = angleGarage - 2)
      {
        servoGarage.write(angleGarage);
        delay(15);
      }
      position_garage = 1;
    }
  }

  Blynk.virtualWrite(V7, position_garage);

}

void fenster(bool pos) {

  if (pos == 0) {

    if (position_fenster == 0) {
      return;
    }
    else {
      for (angleFenster = fensterOffen; angleFenster < fensterZu; angleFenster = angleFenster + 2)
      {
        servoFenster.write(angleFenster);
        delay(15);
      }
      position_fenster = 0;
    }
  }

  if (pos == 1) {
    if (position_fenster == 1) {
      return;
    }
    else {
      for (angleFenster = fensterZu; angleFenster > fensterOffen; angleFenster = angleFenster - 2)
      {
        servoFenster.write(angleFenster);
        delay(15);
      }
      position_fenster = 1;
    }
  }

  Blynk.virtualWrite(V8, position_fenster);
}

float get_fuellstand()
{
  millis_ultrasonic = millis();

  int distand_zu_wasser = sr04.Distance();
  int volumen = (378 - ((distand_zu_wasser - 2) * 42));

  if (volumen < 0) {
    return 0;
  }

  else if (volumen > 378) {
    return 500;
  }
  else {
    return volumen;
  }
}

bool get_rain()
{
  if (analogRead(A1) > 230) {
    return 1;
  }
  else {
    return 0;
  }
}

void update_time()
{
  now = rtc.now();
}

void write_time()
{
  lcd.setCursor(0, 0);
  char days[3];         //char days[] und sprintf(days,....) stellen sicher das immer 2 Stellen angezeigt werden. Zum ändern "%02u" anpassen.
  sprintf(days, "%02u", now.day());
  lcd.print(days);
  lcd.print("/");
  char months[3];
  sprintf(months, "%02u", now.month());
  lcd.print(months);
  lcd.print("/");
  char years[3];
  sprintf(years, "%04u", now.year());
  lcd.print(years);
  lcd.print(" ");
  char hours[3];
  sprintf(hours, "%02u", now.hour());
  lcd.print(hours);
  lcd.print(":");
  char minutes[3];
  sprintf(minutes, "%02u", now.minute());
  lcd.print(minutes);
  lcd.print(" Uhr");
}

int get_temperatur()
{
  dht.read();
  return dht.getTemperatureC();
}

int get_feuchtigkeit()
{
  millis_temp = millis( );
  return dht.getHumidity();
}

String get_wind()
{
  int volt_wind;
  volt_wind = analogRead(A0);

  if (volt_wind < 100)
  {
    return "Windstille";
  }
  else if (volt_wind < 300)
  {
    return "Leichte Brise";
  }
  else if (volt_wind < 700)
  {
    if (position_fenster == 1) {
      fenster(0);
    }
    return "Starker Wind";
  }
  else if (volt_wind > 700)
  {
    if (position_fenster == 1) {
      fenster(0);
    }
    Blynk.notify("Achtung, Sturm zieht auf");
    return "Sturm";
  }
}

bool RFID()
{
  for (int i = 0; i < 10; i++)  {

    //    Serial.println("RFID");
    if ( ! mfrc522.PICC_IsNewCardPresent())
    {
      break;
    }
    if ( ! mfrc522.PICC_ReadCardSerial())
    {
      break;
    }

    long code = 0;
    for (byte i = 0; i < mfrc522.uid.size; i++)
    {
      code = ((code + mfrc522.uid.uidByte[i]) * 10);
    }
    //    Serial.print("Die Kartennummer lautet::");
    //Serial.println(code);

    if (code == 1149410)
    {
      if (digitalRead(34) == HIGH) {
        if (alarmanlage_an_aus == 0) {
          set_alarm(1);
          delay(1000);
          return 0;
        }
        else {
          set_alarm(0);
          delay(1000);
          return 0;
        }
      }
      if (digitalRead(35) == HIGH) {
        if (position_garage == 1) {
          garage(0);
          delay(500);
          return 0;
        }
        else {
          garage(1);
          delay(500);
          return 0;
        }
      }

      if (digitalRead(36) == HIGH) {
        if (position_fenster == 1) {
          fenster(0);
          delay(500);
          return 0;
        }
        else {
          fenster(1);
          delay(500);
          return 0;
        }
      }

      return 1;
    }

    else if (code == 2602310)
    {
      if (digitalRead(34) == HIGH) {
        if (alarmanlage_an_aus == 0) {
          set_alarm(1);
          delay(1000);
          return 0;
        }
        else {
          set_alarm(0);
          delay(1000);
          return 0;
        }
      }
      if (digitalRead(35) == HIGH) {
        if (position_garage == 1) {
          garage(0);
          delay(500);
          return 0;
        }
        else {
          garage(1);
          delay(500);
          return 0;
        }
      }

      if (digitalRead(36) == HIGH) {
        if (position_fenster == 1) {
          fenster(0);
          delay(500);
          return 0;
        }
        else {
          fenster(1);
          delay(500);
          return 0;
        }
      }

      return 1;
    }
    else {
      break;
    }
  }
  return 0;
}

bool get_rauch()
{
  int smokeA0 = A5;
  pinMode(smokeA0, INPUT);
  int analogSensor = analogRead(smokeA0);
  //Serial.println(analogSensor);
  if (analogSensor < 400)
  {
    return 0;
  }
  else
  {
    fenster(1);
    return 1;
  }
}

double get_power()
{
  const int analogIn = A6;
  int mVperAmp = 185; // use 100 for 20A Module and 66 for 30A Module
  int RawValue = 0;
  int ACSoffset = 2500;
  double Voltage = 0;
  double Amps = 0;
  double Power = 0;

  //Berechnung Strom
  RawValue = analogRead(analogIn);
  Voltage = (RawValue / 1024.0) * 5000; // Gets you mV
  Amps = ((Voltage - ACSoffset) / mVperAmp);

  //Bewerhcnung Leistung
  Power = Amps * 5.0 * 1.0;

  return Power;
}



String get_windrichtung()
{
  float Quellspannung = 5.0;
  int AnalogPin = A7;
  int R1 = 290.0; //Zwei Kohleschichtwiderstände in Reihe, pro port max 40mA (R1 geg. Widerstandsgröße)
  long Messwert;
  float SpannungR2; //Spannung über dem zu messenden Widerstand
  float Widerstand;


  //5 Messungen machen und Mittelwert bilden
  Messwert = 0;
  for (int i = 0; i < 5; i++)
  {
    Messwert += analogRead(AnalogPin);
  }
  Messwert = trunc(Messwert / 5);

  //Spannung berechnen
  SpannungR2 = (Quellspannung / 1023.0) * Messwert;
  //Berechnung: (R2 = R1 * (U2/U1))
  Widerstand = R1 * (SpannungR2 / (Quellspannung - SpannungR2));

  if (Widerstand < 3000)
  {
    return "Nord";
  }
  else if (Widerstand < 5764)
  {
    return "Ost";
  }
  else if (Widerstand < 6945)
  {
    return "Süd";
  }
  else
  {
    return "West";
  }
}


void SmartLogic()
{
  //Licht ein bei Bewegung
  s_beweg_1 = digitalRead(bewegungsmelder1);
  s_beweg_2 = digitalRead(bewegungsmelder2);

  if (s_beweg_1 == 1)
  {
    analogWrite(led1, 255);
    timer_1_5000ms = currentMillis + 4000;
  }
  if (s_beweg_2 == 1)
  {
    analogWrite(led2, 255);
    timer_2_5000ms = currentMillis + 4000;
  }

  //Licht aus nach Zeit, passiert nur wenn Licht automatisch über Bewegungsmelder an ging
  if (currentMillis > timer_1_5000ms)
  {
    analogWrite(led1, s_helligkeit_led1);
  }

  if (currentMillis > timer_2_5000ms)
  {
    analogWrite(led2, s_helligkeit_led2);
  }

}

int get_durchfluss() {
  return 60 * dauer_pumpe;
}

void set_LED(int helligkeit) {

  analogWrite(led1, helligkeit);
  analogWrite(led2, helligkeit);
  analogWrite(led3, helligkeit);

  s_helligkeit_led1 = helligkeit;
  s_helligkeit_led2 = helligkeit;
  s_helligkeit_led3 = helligkeit;
  Blynk.virtualWrite(V9, helligkeit);

}

bool get_helligkeit()
{
  const int sensorPin = A2;
  int lightVal;
  lightVal = analogRead(sensorPin);
  if (lightVal > 70)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void PUMPEN(bool on_off)
{
  if (on_off == 1) {
    millis_pumpe = millis();
    digitalWrite(Relay, RELAY_ON);
    pumpe_on_off = 1;
  }

  if (on_off == 0) {
    digitalWrite(Relay, RELAY_OFF);
    dauer_pumpe = dauer_pumpe + ((millis() - millis_pumpe) / 1000);
    pumpe_on_off = 0;
    Blynk.virtualWrite(V6, 0);
  }
}
