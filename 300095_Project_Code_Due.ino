/* <<< ARDUINO CODE FOR - SST  >>>

  Modified 25 August 2021
  by Ragib Yasar Rahman

  © Ragib Yasar Rahman

  Pastebin: https://pastebin.com/NDz70heX  Passcode: 163415#
  Github:   https://github.com/Ragib301/Arduino_Code_of_SST
*/

#include <TinyGPS++.h>
#include <Servo.h>
#include "Wire.h"
#include <include/twi.h>
#include <MPU6050_light.h>
#include <NewPing.h>
#include <LiquidCrystal_I2C.h>

Servo myservo;
#define pos 90
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

#define TRIGGER_PIN A2
#define ECHO_PIN    A3
#define MAX_DISTANCE 300

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
int distance;

#define SprayLed 14
#define Buzzer   15
#define IRpin 3
#define MQ_3 A0
#define Emer_Button A1

#define wait 2500
bool state = true;
#define light LED_BUILTIN

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
  pinMode(Emer_Button, INPUT_PULLUP);
  pinMode(SprayLed, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  pinMode(light, OUTPUT);

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
  lcd.setCursor(0, 0);
  lcd.print("|Safer Shielded|");
  lcd.setCursor(0, 1);
  lcd.print("|Transportation|");

  /* All Functions' Loops */
  mpu6050.update();
  Car_Crash_Avoiding();
  GPS_System();
  //corona_virus_system();
  car_emergency_system();
  Android_Controller();
  Emergency_Button_Function();
}

void Car_Crash_Avoiding() {
  distance = readPing();
  if (distance <= 10) {
    stopme();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Car will Crash!");
    lcd.setCursor(0, 1);
    lcd.print(" Car Stopped!!!");

    for (int i = 0; i < 3; i++) BuzzerTone();
  }
}

int readPing() {
  delay(50);
  int uS = sonar.ping_median();
  int cm = uS / US_ROUNDTRIP_CM;
  if (cm == 0) {
    cm = 250;
  } return cm;
}

void Android_Controller() {
  while (BTSerial.available()) {
    command = BTSerial.read();
    switch (command) {

      case 'F':         // Forward
        forward();
        break;

      case 'B':         // Backward
        backward();
        break;

      case 'L':         // Go Left
        left();
        break;

      case 'G':
        front_left();
        break;

      case 'H':
        back_left();
        break;

      case 'R':         // Go Right
        right();
        break;

      case 'I':
        front_right();
        break;

      case 'J':
        back_right();
        break;

      case 'V':         // Buzzer HIGH
        digitalWrite(Buzzer, HIGH);
        break;

      case 'v':         // Buzzer LOW
        digitalWrite(Buzzer, LOW);
        break;

      case 'W':
        digitalWrite(SprayLed, HIGH);
        break;

      case 'w':
        digitalWrite(SprayLed, LOW);
        break;

      case 'S':         // Stop
        stopme();
        break;

      case 'X':
        corona_virus_system();
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

void front_left() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void back_left() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void right() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void front_right() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void back_right() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopme() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(Buzzer, LOW);
}

void GPS_System() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {
    lati  = gps.location.lat();
    longi = gps.location.lng();
  }
  else {
    lati  = 23.770946;
    longi = 90.361998;
  }
}

void car_emergency_system() {
  X_axis = mpu6050.getAngleX();
  Y_axis = mpu6050.getAngleY();
  const int val = 40;

  if (X_axis >= val || Y_axis >= val) {
    lcd.clear();
    delay(wait);

    lcd.setCursor(0, 0);
    lcd.print("Bus is Tilting!");
    lcd.setCursor(0, 1);
    lcd.print("Accident Alert!");
    delay(1000);

    Emergency_Messeage = "Vehicle Accident Emergency Alert!";
    send_SMS();
  }
  else if (Y_axis <= -val) {
    lcd.clear();
    delay(wait);

    emergencyDoor.write(100);
    lcd.setCursor(0, 0);
    lcd.print("Bus is Tilting!");
    lcd.setCursor(0, 1);
    lcd.print("Accident Alert!");
    delay(1000);

    Emergency_Messeage = "Vehicle Accident Emergency Alert!";
    send_SMS();
  }
  else if (X_axis <= -val) {
    distance = 0;
    lcd.clear();
    delay(wait);

    lcd.setCursor(0, 0);
    lcd.print("Bus is Tilting!");
    lcd.setCursor(0, 1);
    lcd.print("Accident Alert!");
    delay(1000);

    Emergency_Messeage = "Vehicle Tilting Accident Emergency Alert!";
    send_SMS();
  }
}

void Emergency_Button_Function() {
  int r = button_read();
  if (r > 0) {
    lcd.clear();
    stopme();
    delay(wait);

    lcd.setCursor(0, 0);
    lcd.print("Any Passenger");
    lcd.setCursor(0, 1);
    lcd.print("is in Danger!");
    delay(wait);

    state = false;
    if (r == 1) Emergency_Messeage = "Any system of the bus maybe malfuntions, Please take necessary actions!";
    else if (r == 3) Emergency_Messeage = "Any Bus Passenger is in Danger!";
    send_SMS();
  }
}

int button_read() {
  int count = 0;
p:  int t = 0;
  if (digitalRead(Emer_Button) == LOW) {
    digitalWrite(light, HIGH);
    while (digitalRead(Emer_Button) == LOW) {
      delay(1); t++;
    }
    digitalWrite(light, LOW);
    if (t > 3) {
      t = 0; count++;
      while (digitalRead(Emer_Button) == HIGH) {
        delay(1); t++;
        if (t > 1000) return count;
      } goto p;
    }
  } return count;
}

void corona_virus_system() {
  stopme();
  delay(300);
  lcd.clear();
  checkTemp();
}

void checkTemp() {
  delay(wait);
  lcd.setCursor(0, 0);
  lcd.print("BodyTemp: ");
  lcd.print(mlx_body_temp());
  lcd.println("*F  ");
  delay(1000);

  if (mlx_body_temp() >= 100) {
    lcd.setCursor(0, 1);
    lcd.print("Temp. is High!");
    BuzzerTone();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" Please go to");
    lcd.setCursor(0, 1);
    lcd.print("   Hospital !");
    delay(5000);
    
   /*state = false;
    Emergency_Messeage = "Any passenger's temperature is high, Virus Suspect Alert!";
    send_SMS(); */
  }
  else {
    lcd.setCursor(0, 1);
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
  int HumanTemp = tempK - 273.15;
  int TempF = ((HumanTemp * 1.8) + 32) - 5; // Celsius to Fahrenheit
  return TempF;
}

void disinfectSpray() {
  lcd.clear();
  myservo.write(180);
  delay(3100);
  myservo.write(pos);

  lcd.setCursor(0, 0);
  lcd.print("Disinfecting...");

  digitalWrite(SprayLed, HIGH);
  delay(5000);

  digitalWrite(SprayLed, LOW);
  lcd.setCursor(0, 1);
  lcd.print("<--COMPLETE!!-->");
  delay(1100);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("|Safer Shielded|");
  lcd.setCursor(0, 1);
  lcd.print("|Transportation|");
  delay(100);
}

void send_SMS() {
  stopme();

  /* Text content */
  BTSerial.println(Emergency_Messeage);
  BTSerial.print("\n");
  BTSerial.println("Bus Location : ");
  BTSerial.print(googlemapURL);
  BTSerial.print(lati, 6);
  BTSerial.print(",");
  BTSerial.println(longi, 6);
  BTSerial.print("\n");
  BTSerial.print(char(26));

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Emergency SMS");
  lcd.setCursor(0, 1);
  lcd.println("has sent. OKAY  ");

  while (true) {
    BuzzerTone();
  }
}

void BuzzerTone() {
  if (state) {
    digitalWrite(Buzzer, HIGH);
    delay(900);
    digitalWrite(Buzzer, LOW);
    delay(400);
  }
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
