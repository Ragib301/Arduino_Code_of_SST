/* <<< ARDUINO CODE FOR - SST  >>>

  Modified 23 December 2020
  by Ragib Yasar Rahman

  © Ragib Yasar Rahman

  Backup: https://pastebin.com/NDz70heX
*/

#include <TinyGPS++.h>
#include <Servo.h>
#include "Wire.h"
#include <include/twi.h>
#include <MPU6050_light.h>
#include <NewPing.h>
#include <LiquidCrystal_I2C.h>

Servo myservo;
const int pos = 90;
Servo emergencyDoor;

MPU6050 mpu6050(Wire);
int X_axis, Y_axis;

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define BTSerial Serial1  // HC-06 Bluetooth Module Port
char command;             // Bluetooth Serial Command
String Emergency_Messeage;

#define gpsSerial Serial2  // Neo-6M GPS Port
TinyGPSPlus gps;
float lati, longi;
String googlemapURL = "https://www.google.com/maps/@";

#define TRIGGER_PIN A3
#define ECHO_PIN    A2
#define MAX_DISTANCE 300

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
int distance;

#define SprayLed 14
int Buzzer = 15;
#define IRpin 3
#define MQ_3 A0
#define Emer_Button A1

const int wait = 2000;

//Left motor pins
#define IN1 8
#define IN2 9

//Right motor pins
#define IN3 10
#define IN4 11

/* GY-906 Temperarture Sensor Variables */
#define ADDR      0x5A
#define TO_MAX    0x00
#define TO_MIN    0x01
#define PWM_CTRL  0x02
#define RAW_IR_1  0x04
#define RAW_IR_2  0x05
#define TA        0x06
#define TOBJ_1    0x07
#define TOBJ_2    0x08
#define SYNC_PIN  2
static const uint32_t TWI_CLOCK = 100000;
static const uint32_t RECV_TIMEOUT = 100000;
static const uint32_t XMIT_TIMEOUT = 100000;
Twi *pTwi = WIRE_INTERFACE;

static void Wire_Init(void) {
  pmc_enable_periph_clk(WIRE_INTERFACE_ID);
  PIO_Configure(
  g_APinDescription[PIN_WIRE_SDA].pPort,
  g_APinDescription[PIN_WIRE_SDA].ulPinType,
  g_APinDescription[PIN_WIRE_SDA].ulPin,
  g_APinDescription[PIN_WIRE_SDA].ulPinConfiguration);
  PIO_Configure(
  g_APinDescription[PIN_WIRE_SCL].pPort,
  g_APinDescription[PIN_WIRE_SCL].ulPinType,
  g_APinDescription[PIN_WIRE_SCL].ulPin,
  g_APinDescription[PIN_WIRE_SCL].ulPinConfiguration);
  NVIC_DisableIRQ(TWI1_IRQn);
  NVIC_ClearPendingIRQ(TWI1_IRQn);
  NVIC_SetPriority(TWI1_IRQn, 0);
  NVIC_EnableIRQ(TWI1_IRQn);
}

static void Wire1_Init(void) {
    pmc_enable_periph_clk(WIRE1_INTERFACE_ID);
  PIO_Configure(
      g_APinDescription[PIN_WIRE1_SDA].pPort,
      g_APinDescription[PIN_WIRE1_SDA].ulPinType,
      g_APinDescription[PIN_WIRE1_SDA].ulPin,
      g_APinDescription[PIN_WIRE1_SDA].ulPinConfiguration);
  PIO_Configure(
      g_APinDescription[PIN_WIRE1_SCL].pPort,
      g_APinDescription[PIN_WIRE1_SCL].ulPinType,
      g_APinDescription[PIN_WIRE1_SCL].ulPin,
      g_APinDescription[PIN_WIRE1_SCL].ulPinConfiguration);
  NVIC_DisableIRQ(TWI0_IRQn);
  NVIC_ClearPendingIRQ(TWI0_IRQn);
  NVIC_SetPriority(TWI0_IRQn, 0);
  NVIC_EnableIRQ(TWI0_IRQn);
}

void setup() {
  gpsSerial.begin(9600);
  BTSerial.begin(9600);

  lcd.init();
  lcd.backlight();

  emergencyDoor.attach(4);
  emergencyDoor.write(0);
  myservo.attach(5);
  myservo.write(pos);

  pinMode(IRpin, INPUT);
  pinMode(MQ_3, INPUT);
  pinMode(Emer_Button, INPUT);
  pinMode(SprayLed, OUTPUT);
  pinMode(Buzzer, OUTPUT);

  pinMode(IN1, OUTPUT);   //Right Motor
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);   //Left Motor
  pinMode(IN4, OUTPUT);

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets();

  pinMode(SYNC_PIN, OUTPUT);
  digitalWrite(SYNC_PIN, LOW);
  Wire_Init();
  pTwi->TWI_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
  TWI_ConfigureMaster(pTwi, TWI_CLOCK, VARIANT_MCK);
}

void loop() {
  mpu6050.update();

  lcd.setCursor(0,0);
  lcd.print("|Safer Shielded|");
  lcd.setCursor(0,1);
  lcd.print("|Transportation|");

 /* All Functions' Loops */
  Car_Crash_Avoiding();
  GPS_System();
  corona_virus_system();
  car_emergency_system();
  Android_Controller();
  Emergency_Button_Function();
}

void Car_Crash_Avoiding() {
  delay(50);
  if (readPing() <= 9) {
    stopme();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Car will Crash!");
    lcd.setCursor(0,1);
    lcd.print(" Car Stopped!!!");

    for(int i=0; i<4; i++){
      BuzzerTone();
    }
  }
}

int readPing() {
  delay(70);
  int cm = sonar.ping_cm();
  if (cm==0) {
    cm=250;
  } return cm;
}

void Android_Controller() {
  if (BTSerial.available()) { 
   command = BTSerial.read();

   switch(command) {

    case 'F':         // Forward
      forward();
      break;

    case 'B':         // Backward
      backward();
      break;

    case 'L':         // Go Left
      left();
      break;

    case 'R':         // Go Right
      right();
      break;

    case 'S':         // Stop
      stopme();
      break;

    default:         // Garbage Data Stop
      stopme();
    }
  } 
}

void forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void backward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);  
}

void left() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void right() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopme() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void GPS_System() {  
  while (gpsSerial.available()) {gps.encode(gpsSerial.read());}

  if (gps.location.isValid()) {
    lati  = gps.location.lat();
    longi = gps.location.lng();
  }
  else{
  /*lati  = 23.770946;
    longi = 90.361998;*/
    lati  = 23.77901497396472;
    longi = 90.37162295955909;
  }
}

void car_emergency_system() {
  X_axis = mpu6050.getAngleX();
  Y_axis = mpu6050.getAngleY();
  const int val = 50;

  if (X_axis >= val || Y_axis >= val) {
   delay(wait);
   Emergency_Messeage = "Vehicle Accident Emergency Alert!";
   send_SMS();
  }
  else if (Y_axis <= -val) {
   delay(wait);
   emergencyDoor.write(100);
   Emergency_Messeage = "Vehicle Accident Emergency Alert!";
   send_SMS();
  }
  else if (X_axis <= -val) {
   distance = 0;
   delay(wait);
   Emergency_Messeage = "Vehicle Accident Emergency Alert!";
   send_SMS();
  }
}

void Emergency_Button_Function() {
  if (digitalRead(Emer_Button) == LOW) {
    lcd.clear();
    stopme();
    delay(wait);

    lcd.setCursor(0,0);
    lcd.print("Any Passenger");
    lcd.setCursor(0,1);
    lcd.print("is in Danger!");
    delay(1600);

    Emergency_Messeage = "Any Bus Passenger is in Danger!";
    send_SMS();
  }
}

void corona_virus_system() {
  if (digitalRead(IRpin) == LOW) {
    stopme();
    lcd.clear();
    checkTemp();
  }
}

void checkTemp() {
  delay(wait);
  lcd.setCursor(0,0);
  lcd.print("BodyTemp: ");
  lcd.print(mlx_body_temp());
  lcd.println("*F  ");
  delay(1000);

  if (mlx_body_temp() >= 100) {
    lcd.setCursor(0,1);
    lcd.print("Temp. is High!");
    BuzzerTone();

    Buzzer = 7;
    Emergency_Messeage = "Covid-19 Virus Patient Suspect Alert!";
    send_SMS();
  }
  else{
    lcd.setCursor(0,1);
    lcd.print("Temp. is Okay.");
    BuzzerTone();
    disinfectSpray();
  }
}

int mlx_body_temp() {
  uint16_t tempUK;
  float tempK;
  uint8_t hB, lB, pec;
  digitalWrite(SYNC_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SYNC_PIN, LOW);
  TWI_StartRead(pTwi, ADDR, TOBJ_1, 1);
  lB = readByte();
  hB = readByte();
  TWI_SendSTOPCondition(pTwi);
  pec = readByte();
  while (!TWI_TransferComplete(pTwi)) 
    ;
  tempUK = (hB << 8) | lB; 
  tempK = ((float)tempUK * 2) / 100 ;
  int HumanTemp = tempK-273.15;
  int TempF = ((HumanTemp*1.8)+32)+12;    // Celsius to Fahrenheit
  return TempF;
}

void disinfectSpray() {
  lcd.clear();
  myservo.write(170);
  delay(2000);
  myservo.write(pos);

  lcd.setCursor(0,0);
  lcd.print("Disinfecting...");

  digitalWrite(SprayLed, HIGH);
  delay(5000);

  digitalWrite(SprayLed, LOW);
  lcd.setCursor(0,1);
  lcd.print("--COMPLETE!!--");
  delay(1000);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("|Safer Shielded|");
  lcd.setCursor(0,1);
  lcd.print("|Transportation|");
}

void send_SMS() {
  stopme();

 /* Text content */
  BTSerial.println(Emergency_Messeage);
  BTSerial.print("\n");
  BTSerial.println("Bus Location : ");
  BTSerial.print(googlemapURL);
  BTSerial.print(lati, 14);
  BTSerial.print(",");
  BTSerial.println(longi, 14);
  BTSerial.print("\n");

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Emergency SMS");
  lcd.setCursor(0,1);
  lcd.println("has sent. OKAY  ");

  while(true) {BuzzerTone();}
}

void BuzzerTone() {
  digitalWrite(Buzzer, HIGH);
  delay(900);
  digitalWrite(Buzzer, LOW);
  delay(400);
}

/*

 "If you want your children to be intelligent, read them fairy tales. 
  If you want them to be more intelligent, read them more fairy tales."

                                                   ---Albert Einstein
              .-----.
            ,'(0) (0)`,
           /     ^     \
    ____ __\ \_______/ /____ __
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

uint8_t readByte() {
  while (!TWI_ByteReceived(pTwi))
    ;
  return TWI_ReadByte(pTwi);
}
static inline bool TWI_WaitTransferComplete(Twi *_twi, uint32_t _timeout) {
  while (!TWI_TransferComplete(_twi)) {
    if (TWI_FailedAcknowledge(_twi))
      return false;
    if (--_timeout == 0)
      return false;
  }
  return true;
}
static inline bool TWI_WaitByteReceived(Twi *_twi, uint32_t _timeout) {
  while (!TWI_ByteReceived(_twi)) {
    if (TWI_FailedAcknowledge(_twi))
      return false;
    if (--_timeout == 0)
      return false;
  } return true;
}
static inline bool TWI_FailedAcknowledge(Twi *pTwi) {
  return pTwi->TWI_SR & TWI_SR_NACK;
}
