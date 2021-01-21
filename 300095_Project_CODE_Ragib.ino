#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include "Wire.h"
#include <Adafruit_MLX90614.h>
#include <MPU6050_light.h>
#include <LiquidCrystal_I2C.h>

Servo myservo;
int pos = 40;

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
int HumanTemp;

MPU6050 mpu6050(Wire);
int X_axis, Y_axis;

LiquidCrystal_I2C lcd(0x27, 16, 2);

SoftwareSerial gsmSerial(9, 10);  // RX | TX

SoftwareSerial gpsSerial(11, 12); // RX | TX
TinyGPSPlus gps;
float lati, longi;
String googlemapURL = "https://www.google.com/maps/@";

#define SprayLed 6
#define Buzzer 8
#define IRpin 3
#define MQ_3 A0

int isHuman, alchohol_out;
String Emergency_Messeage = "";

int Baund = 9600;
int wait = 1000;

void setup() {  
  Serial.begin(115200);
  gpsSerial.begin(Baund);
  gsmSerial.begin(Baund);

  lcd.init();
  lcd.backlight();
  myservo.attach(5);
  myservo.write(pos);

  pinMode(IRpin, INPUT);
  pinMode(MQ_3, INPUT);
  pinMode(SprayLed, OUTPUT);
  pinMode(Buzzer, OUTPUT);

  mlx.begin();
  Wire.begin();

  mpu6050.begin();
  mpu6050.calcGyroOffsets();

  lcd.setCursor(0,0);
  lcd.print("|Safer Shielded|");
  lcd.setCursor(0,1);
  lcd.print("|Transportation|");
}

void(*resetFunc) (void) = 0; //declare reset function @ address 0

void loop() {
  mpu6050.update();

  //GPS_System();
  corona_virus_system();
  car_emergency_system();
  //drunk_driver_alert();
}

void GPS_System() {
  gpsSerial.listen();

  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {
    lati = gps.location.lat();
    longi = gps.location.lng();
  }
}

void drunk_driver_alert() {
  alchohol_out = analogRead(MQ_3);
  int threshold = 490;

  if (alchohol_out >= threshold) {
    delay(wait);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Vehicle Driver");
    lcd.setCursor(0,1);
    lcd.print("is Drunk!!");
    BuzzerTone();

    Emergency_Messeage = "Vehicle Driver is Drunk Alert!";
    send_SMS();
  }
}

void car_emergency_system() {
  X_axis = mpu6050.getAngleX();
  Y_axis = mpu6050.getAngleY();
 
 if (X_axis <= -35 || X_axis >= 35 || Y_axis <= -35 || Y_axis >= 35) {
   lcd.clear();
   delay(wait);
   lcd.setCursor(0,0);
   lcd.print("Car Accident");
   lcd.setCursor(0,1);
   lcd.print("Emergency!!");

   Emergency_Messeage = "Vehicle Accident Emergency Alert!";
   send_SMS();

  for(;;) {
   BuzzerTone();
  }
 }
}

void corona_virus_system(){
  isHuman = digitalRead(IRpin);

  if (isHuman == LOW) {
    lcd.clear();
    checkTemp();
  }
}

void checkTemp() {
  delay(wait);
  HumanTemp = mlx.readObjectTempF()+6;

  lcd.setCursor(0,0);
  lcd.print("BodyTemp: ");
  lcd.print(HumanTemp);
  lcd.println("*F  ");

  if (HumanTemp >= 100) {
    delay(1000);
    lcd.setCursor(0,1);
    lcd.print("Temp. is High!");
    BuzzerTone();

    Emergency_Messeage = "Hanta Virus Patient Suspect Alert!";
    send_SMS();
  }
  else{
    delay(1000);
    lcd.setCursor(0,1);
    lcd.print("Temp. is Okay.");
    BuzzerTone();
    disinfectSpray();
  }
}

void disinfectSpray() {
  myservo.write(180);
  delay(2500);
  myservo.write(pos);
  digitalWrite(SprayLed, HIGH);
  delay(5000);
  digitalWrite(SprayLed, LOW);
  delay(200);

  resetFunc();
}

void send_SMS() {

  lati =  23.7706257;
  longi = 90.3620139;

  gsmSerial.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();
  gsmSerial.println("AT+CMGS=\"+8801718613791\""); // Change ZZ with country code and xxxxxxxxxxx with phone number to sms
  updateSerial();
  
  gsmSerial.println(Emergency_Messeage); //text content
  updateSerial();
  gsmSerial.print("\n");
  updateSerial();
  gsmSerial.println("Google Map Link: ");
  updateSerial();
  gsmSerial.print(googlemapURL);
  updateSerial();
  gsmSerial.print(lati,6);
  updateSerial();
  gsmSerial.print(",");
  updateSerial();
  gsmSerial.print(longi,6);
  updateSerial();
  gsmSerial.print(",15z");
  updateSerial();
  gsmSerial.write(26);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Emergency SMS");
  lcd.setCursor(0,1);
  lcd.println("has sent. OKAY    ");
  //exit(0);
}

void BuzzerTone() {
  digitalWrite(Buzzer, HIGH);
  delay(1000);
  digitalWrite(Buzzer, LOW);
  delay(500);
}

void updateSerial() {
  gsmSerial.listen();
  delay(200);

  while (Serial.available()) {
    gsmSerial.write(Serial.read());  //Forward what Serial received to Software Serial Port
  }
  while (gsmSerial.available()) {
    Serial.write(gsmSerial.read());  //Forward what Software Serial received to Serial Port
  }
}

/*

 "If you want your children to be intelligent, read them fairy tales. 
  If you want them to be more intelligent, read them more fairy tales."

                                                   ---Albert Einstein
              .-----.
            ,'(0) (0)`,
           /     ^     \
    ____ _ \ \_______/ /
   /_||   ||`-._____.-`||   ||-\
  / _||===||    ! !    ||===|| _\
 |- _||===||===========||===||- _|
 \___||___||___________||___||___/
  \\|///   \_:_:_:_:_:_/   \\\|//
  |   _|    |_________|    |   _|
  |   _|   /( ======= )\   |   _|
  \\||//  /\ `-.___.-' /\  \\||//
   (o )  /_ '._______.' _\  ( o)
  /__/ \ |    _|   |_   _| / \__\
  ///\_/ |_   _|   |    _| \_/\\\
 ///\\_\ \    _/   \    _/ /_//\\\
 \\|//_/ ///|\\\   ///|\\\ \_\\|//
         \\\|///   \\\|///
         /-  _\\   //   _\
         |   _||   ||-  _|
       ,/\____||   || ___/\,
      /|\___`\,|   |,/'___/|\
      |||`.\\ \\   // //,'|||
      \\\\_//_//   \\_\\_////
*/
