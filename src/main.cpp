// https : // github.com/matrix00660066/RFID_card_reader.git

#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>

/**
 * Odczyt karty za pomocą biblioteki mfrc522 na interfejsie SPI
 * Rozkład wyprowadzeń do podłączenia (Arduino Uno):
 * MOSI: Pin 11 / ICSP-4
 * MISO: Pin 12 / ICSP-1
 * SCK: Pin 13 / ISCP-3
 * SS: Pin 7
 * RST: Pin 6
 */

#define SS_PIN 7
#define RST_PIN 6
#define dysk1 16

MFRC522 mfrc522(SS_PIN, RST_PIN); // Create MFRC522 instance.

RFID rfid(SS_PIN, RST_PIN);
// byte ssPins[] = {SS_1_PIN, SS_2_PIN};
MFRC522 mfrc522[NR_OF_READERS]; // tworzenie instancji MFRC522.

LiquidCrystal_I2C lcd(0x27, 16, 2);

//*******************************************//

//*********** Przypisanie pinów *************//
//*******************************************//
// int dysk1 = A2;                              //tranzystor dysku 1 jako pin A2
int dysk2 = A3;              // tranzystor dysku 2 jako pin A3
int kontrolka = 10;          // kontrolka sygnalizacji wgrania karty
int wejscie = 2;             // wejście odczytujące czy zapisujemy kartę jako pin D2
int napiecie5V = A1;         // wejscie do pomiaru napięcia 5V
int napiecie12V = A0;        // wejście do pomiaruu napięcia 12V
int digitalAnalogLedPin = 9; // wejscie rozpoznawania czy sterujemy diodami analogowymi czy cyfrowymi
int naukaPin = 0;            // wejście prozpoznawania czy chcę wejść w tryb uczenia pilota
//*******************************************//

//********** ustawienia zmiennych: **********//
//*******************************************//
boolean stanWejscia; // stan wejścia programowania karty
boolean nauka;       // zmienna do wpisywania czy wchodzę do funkcji nauki pilota
byte serNum0;        // zwykła zmienna do wpisania do niej wartości
byte serNum1;        // odczytanej z karty
byte serNum2;
byte serNum3;
byte serNum4;

byte epromSerNum0 = 0; // zmienna w pamięci EEPROM do przechowywania
byte epromSerNum1 = 1; // zapisanej karty
byte epromSerNum2 = 2;
byte epromSerNum3 = 3;
byte epromSerNum4 = 4;

int intervalOdczytu = 2500; // czas co jaki wykonywany jest skok do funkcji odczytu napięć, tu 2,5sek
boolean windowsCzyLinux;    // zmienna dla rozpoznawania czy włączony windows czy linux

//*******************************************//

void pomiarNapiecia();
void sterowanieDiodami();               //test
void naukaPilota();
void odczytKarty();
void zapisKarty();


//*********** Ustawienia główne *************//
//*******************************************//
void setup()
{

  Serial.begin(9600); // start seriala
  SPI.begin();        // start SPI
  rfid.init();        // inicjalizacja biblioteki RFID

  pinMode(dysk1, OUTPUT); // tu chyba nie trzeba tłumaczyć ?
  digitalWrite(dysk1, LOW);

  pinMode(dysk2, OUTPUT); // tak samo chyba tu ?
  digitalWrite(dysk2, LOW);

  pinMode(kontrolka, OUTPUT);
  digitalWrite(kontrolka, LOW);

  pinMode(wejscie, INPUT_PULLUP);
  stanWejscia = digitalRead(wejscie); // czytam czy nie chcę zapisać karty
  if (stanWejscia == 0)
  {               // jeśli tak to
    zapisKarty(); // obsługuję procedurę zapisu karty do EEPROMu
  }
  else
  {                // a jeśli nie to
    odczytKarty(); // odczytuję kartę
  }
  byte zmienna;                        // zmienna lokalna do odczytu i zaoszczędzenia pamięci
  zmienna = EEPROM.read(epromSerNum0); // do zmiennej wpisuję wartość pierwszego parametru karty
  if (zmienna == serNum0)
  {                                      // jeśli parametr zgadza się z pierwszym który był wpisany do EEPROM i odczytany to
    zmienna = EEPROM.read(epromSerNum1); // sprawqdzam to dalej cyklicznie tak samo
    if (zmienna == serNum1)
    {
      zmienna = EEPROM.read(epromSerNum2);
      if (zmienna == serNum2)
      {
        zmienna = EEPROM.read(epromSerNum3);
        if (zmienna == serNum3)
        {
          zmienna = EEPROM.read(epromSerNum4);
          if (zmienna == serNum4)
          {
            digitalWrite(dysk1, HIGH); // jeśli wszystko się zgadza to włączam jeden dysk
            digitalWrite(dysk2, LOW);
            windowsCzyLinux = 0;
          }
        }
      }
    }
  }
  else
  {
    digitalWrite(dysk1, LOW); // a jeśli się nie zgadza to włączam drugi
    digitalWrite(dysk2, HIGH);
    windowsCzyLinux = 1;
  }
  while ((stanWejscia = digitalRead(wejscie)) == 0)
  {                                // taka pętla sprawi że kontrolka będzie migać
    digitalWrite(kontrolka, HIGH); // po tym jak zostaną ustawione odpowiednie wyjścia
    delay(100);                    // i tylko wówczas kiedy nie zostanie jeszcze zdjęta
    digitalWrite(kontrolka, LOW);  // zworka programowania karty
    delay(100);
  }
  lcd.init(); // zainicjuj lcd
  lcd.clear();
  lcd.backlight();

  pinMode(digitalAnalogLedPin, INPUT_PULLUP);
  nauka = digitalRead(naukaPin);
  if (naukaPin == 0)
  {
    naukaPilota();
  }
}
//*******************************************//

//************* Pętla główna ****************//
//*******************************************//
void loop()
{                          // na razie pusta pętla główna
  unsigned long staryCzas; // zmienna do wpisywania wartości millis
  boolean flagaOdczytuNapiecia;
  if (flagaOdczytuNapiecia == 0)
  {                           // zmienna która sprawdza czy można odczytać czas
    staryCzas = millis();     // do zmiennej staryCzas wp[isujemy aktualną wartość millis
    flagaOdczytuNapiecia = 1; // flagę ustawiamy na 1 żeby wartość millis nie była wpisywana cały czas
  }
  if (millis() - staryCzas > intervalOdczytu)
  {                           // jeśli od wartości millis odejmiemy stary czas i wyjdzie więcej jak intervalOdczytu (2,5sek)
    pomiarNapiecia();         // to wywołaj funkcję pomiaru napięcia
    flagaOdczytuNapiecia = 0; // zeruję flagę żeby można było wpisać nową wartość millis do zmiennej
  }                           //-------------------------------koniec warunku odczytu napięcia

  sterowanieDiodami(); // skok do funkcji sterownia ledami
}
//*******************************************//

void sterowanieDiodami()
{
  boolean digitalAnalog;
  digitalAnalog = digitalRead(digitalAnalogLedPin);
  if (digitalAnalog == 0)
  {
  }
  else
  {
  }
}

void naukaPilota()
{
}

//********* Funckja odczytu karty ***********//
//*******************************************//
void odczytKarty()
{
  Serial.println("Procedura odczytu karty");
  if (rfid.isCard())
  { // tym zajmuje się biblioteka
    if (rfid.readCardSerial())
    {
      Serial.println("Card found");
      serNum0 = rfid.serNum[0]; // wpisanie wartości odczytanej z rfid na pozycji 0
      serNum1 = rfid.serNum[1]; // do danej zmiennej, no i kolejnych
      serNum2 = rfid.serNum[2];
      serNum3 = rfid.serNum[3];
      serNum4 = rfid.serNum[4];

      Serial.println(serNum0); // to drukuje tylko na serialu
      Serial.println(serNum1);
      Serial.println(serNum2);
      Serial.println(serNum3);
      Serial.println(serNum4);
    }
  }
}
//*******************************************//

//********** Funkcja zapisu karty ***********//
//*******************************************//
void zapisKarty()
{
  Serial.println("Procedura zapisu karty");
  odczytKarty();                       // najpierw odczytujemy kartę którą chcemy zapisać
  EEPROM.write(epromSerNum0, serNum0); // tu mamy zapisywanie odczytanej karty do pamięci EEPROM
  EEPROM.write(epromSerNum1, serNum1);
  EEPROM.write(epromSerNum2, serNum2);
  EEPROM.write(epromSerNum3, serNum3);
  EEPROM.write(epromSerNum4, serNum4);
}
//*******************************************//

//******** Funkcja pomiaru napięcia *********//
//*******************************************//
void pomiarNapiecia()
{

  int pomiar5V = 0; // zmienne do przechowywania pomiaróœ napięć 5V i 12V
  int pomiar12V = 0;
  float napiecie = 0;
  // lcd.clear();
  pomiar5V = analogRead(napiecie5V);
  pomiar12V = analogRead(napiecie12V);
  napiecie = pomiar5V * (5.0 / 1024.0);
  lcd.setCursor(0, 0);
  lcd.print("Linia  5V= ");
  lcd.print(napiecie);
  lcd.print("V   ");
  napiecie = pomiar12V * (5.0 / 1024.0) * 3;
  lcd.setCursor(0, 1);
  lcd.print("Linia 12V=");
  lcd.print(napiecie);
  lcd.print("V   ");
}
//*******************************************//
