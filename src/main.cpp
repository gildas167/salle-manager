/**
 *   Gesttion automatique d'une salle de 12 places
 *
 *  @author TIA (Tech It All Engeneering and Services)
 *  @version 1.1
 *  @copyright 2022
 *
 */

#include <Arduino.h>

#include <Arduino.h>
#include <Servo.h> // on inclut la bibliothèque
#include "DHT.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <HCSR04.h>

#define VITESSE 340   // vitesse du son 340 m/s
#define DHTPIN 2      // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22 // DHT 22 (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);

const byte buzzer = 10;
const byte triggerPin = 13;
const byte echoPin = 12;
const byte led_r = 15;
const byte led_v = 4;
const byte ventilo = 3;
unsigned long debut;
unsigned long fin;
unsigned long duree;

unsigned long debut1;
unsigned long fin1;
unsigned long duree1;

unsigned long debut3;
unsigned long fin3;
unsigned long duree3;

UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);

int place = 12; // Enter Total number of parking places
long temps;     // variable qui stocke la mesure du temps
int ferme1 = 0;
int ferme2 = 0;
int enter = 0;
unsigned long currentTime = 0;
unsigned long previousTime = 0;

const int declencheur = 9; // la broche servant à déclencher la mesure
const int capteur = 6;     // la broche qui va lire la mesure
LiquidCrystal_I2C lcd(0x27, 16, 2);
const int pinD5 = 5;
Servo servoMoteur; // on crée un objet servo appelé servoMoteur
int compt = 0;
void setup()
{

  pinMode(ventilo, OUTPUT);
  pinMode(pinD5, OUTPUT);
  pinMode(led_r, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(led_v, OUTPUT);
  digitalWrite(pinD5, HIGH);
  pinMode(declencheur, OUTPUT);
  pinMode(capteur, INPUT);
  servoMoteur.attach(11);
  lcd.init();
  lcd.backlight();
  lcd.init(); // initialize the lcd
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initialisation!!...");
  lcd.setCursor(0, 1);
  lcd.print("Salle Auto Steamy");
  dht.begin();
  delay(2000);
  Serial.begin(9600);
  // on déplace le servo à la position 90º
  servoMoteur.write(360);
  delay(1000);
  // on déplace le servo à la position 180º
  servoMoteur.write(180);
  delay(1000);
  // on déplace le servo à la position 90º
  servoMoteur.write(340);
  // on associe le servo à la broche 11 d'Arduino
  digitalWrite(led_v, HIGH);
  lcd.clear();
}

void loop()
{
  // Wait a few seconds between measurements.
  // delay(2000);
  // Reading temperature or humidity takes about 250 milliseconds!

  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t))
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Le capteur a un problème.");
    lcd.setCursor(0, 1);
    lcd.print("probleme.");
    delay(500);
    lcd.clear();
    return;
  }

  float distance2 = distanceSensor.measureDistanceCm();
  digitalWrite(declencheur, HIGH);
  delayMicroseconds(10); // on attend 10 µs
  digitalWrite(declencheur, LOW);

  // puis on récupère la mesure
  unsigned long duree = pulseIn(capteur, HIGH);
  // l'onde est revenue ! on peut faire le calcul
  // on divise par 2 pour n'avoir qu'un trajet (plutôt que l'aller-retour)
  duree = duree / 2;
  float temps = duree / 1000000.0;           // on met en secondes
  float distance1 = (temps * VITESSE) * 100; // on multiplie par lavitesse, d=t*v

  if (place == 0)
  {
    digitalWrite(led_r, HIGH);
    digitalWrite(led_v, LOW);
  }

  if (distance1 < 10 && ferme1 == 0)
  {
    debut = millis();

    if (place > 0)
    {

      ferme1 = 1;
      if (ferme2 == 0)
      {

        // servoMoteur.write(180);
        servoMoteur.detach();
        digitalWrite(led_r, HIGH);
        digitalWrite(led_v, LOW);
        place = place - 1;
      }
    }
    else
    {
      lcd.setCursor(0, 0);
      lcd.print("   DESOLE!!    ");
      lcd.setCursor(0, 1);
      lcd.print(" Salle Pleine...");
      delay(3000);
      lcd.clear();
    }
  }
  fin = millis();
  duree = fin - debut;
  //  Serial.println(duree);
  if (duree >= 20000 && ferme1 == 1)
  {
    debut = fin;
    place = place + 1;
    if (place > 12)
    {
      place = 12;
    }

    // fin == debut;
    ferme1 = 0;
    servoMoteur.attach(11);
    servoMoteur.write(servoMoteur.read());
    digitalWrite(led_r, LOW);
    digitalWrite(led_v, HIGH);
  }

  if (distance2 < 10 && ferme2 == 0 && place != 12 && distance2 != -1)
  {
    debut1 = millis();
    ferme2 = 1;
    if (ferme1 == 0)
    {
      // servoMoteur.write(180);
      servoMoteur.detach();
      digitalWrite(led_r, HIGH);
      digitalWrite(led_v, LOW);
      place = place + 1;
    }
  }
  fin1 = millis();
  duree1 = fin1 - debut1;
  //  Serial.println(duree);
  if (duree1 >= 20000 && ferme2 == 1)
  {
    debut1 = fin1;
    place = place - 1;
    if (place > 12)
    {
      place = 12;
    }

    // fin == debut;
    ferme2 = 0;
    servoMoteur.attach(11);
    servoMoteur.write(90);
    digitalWrite(led_r, LOW);
    digitalWrite(led_v, HIGH);
  }

  if (ferme1 == 1 && ferme2 == 1)
  {
    digitalWrite(buzzer, HIGH);
    delay(500);
    servoMoteur.attach(11);
    servoMoteur.write(servoMoteur.read());
    digitalWrite(led_r, LOW);
    digitalWrite(led_v, HIGH);
    ferme1 = 0, ferme2 = 0;
  }
  else
  {
    digitalWrite(buzzer, LOW);
  }

  if (t > 29 && place < 12)
  {
    digitalWrite(ventilo, HIGH);
  }
  else
  {
    digitalWrite(ventilo, LOW);
  }

  lcd.setCursor(0, 0);
  lcd.print("Place dispo ");

  if (place >= 10)
  {
    lcd.setCursor(14, 0);
    lcd.print(place);
  }
  if (place < 10)
  {
    lcd.setCursor(14, 0);
    lcd.print(0);
    lcd.setCursor(15, 0);
    lcd.print(place);
  }

  lcd.setCursor(0, 1);
  lcd.print(F("Temp: "));
  lcd.setCursor(7, 1);
  lcd.print(t);
  lcd.setCursor(12, 1);
  lcd.print(F(" C"));
}