#include <LiquidCrystal.h>
#include <DHT11.h>
#include <Stepper.h>
#include <DS1307.h>

#define RDA 0x80
#define TBE 0x20
#define STEPS 500

//GPIO Pointers
volatile unsigned char *port_a = (unsigned char *) 0x22;
volatile unsigned char *ddr_a = (unsigned char *) 0x21;
volatile unsigned char *port_b = (unsigned char *) 0x23;
volatile unsigned char *port_c = (unsigned char *) 0x28;
volatile unsigned char *ddr_c = (unsigned char *) 0x27;
volatile unsigned char *port_g = (unsigned char *) 0x34;
volatile unsigned char *ddr_g = (unsigned char *) 0x33;
volatile unsigned char *port_h = (unsigned char *) 0x102;
volatile unsigned char *ddr_h = (unsigned char *) 0x101;
volatile unsigned char *pin_h = (unsigned char *) 0x100;

//Serial Pointers
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

//ADC Pointers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//Timer Pointers
volatile unsigned char *myTCCR1A  = 0x80;
volatile unsigned char *myTCCR1B  = 0x81;
volatile unsigned char *myTCCR1C  = 0x82;
volatile unsigned char *myTIMSK1  = 0x6F;
volatile unsigned char *myTIFR1   = 0x36;
volatile unsigned int  *myTCNT1   = 0x84;

//LCD Display pins
const int RS = 50, EN = 52, D4 = 53, D5 = 51, D6 = 49, D7 = 47;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

//Set Humidity and Temp Pin
DHT11 DHT11(32);

//Set Stepper Pins
Stepper Stepper(STEPS, 22, 24, 26, 28);
int stepVal = 0;

//Start Button ISR Pins
const byte ledPin = 35;
const byte interruptPin = 18;
volatile byte powerState = LOW;

//Machine State
int machineState = 0;
int prevState;
int change;

//Set DS1307 RTC
DS1307 rtc(SDA, SCL);
bool sent = false;
int min;

// set millis variable
unsigned long startmill;
unsigned long mill;
const unsigned long timer = 60000;

//set reset variable
int restate = 0;


//Setup
void setup(){
  U0init(9600);
  adc_init();

  *ddr_c |= 0x05;
  *ddr_g |= 0x05;
  *ddr_a |= 0x02;
  *ddr_h &= ~(0x10);
  *port_h &= ~(0x10);

  Stepper.setSpeed(20);

  rtc.begin();
  rtc.setDOW(SUNDAY);
  rtc.setTime(5, 33, 0);
  rtc.setDate(5, 10, 2024);

  attachInterrupt(digitalPinToInterrupt(interruptPin), start_stop, RISING);

  lcd.begin(16,2);
  setDisplay_Temp_Humid();

  motorOFF();
}

//Loop
void loop(){
  if (powerState == LOW){
    *port_c &= ~(0x05);
    *port_g &= ~(0x05);
    *port_c |= 0x04 ;
    motorOFF();
  }
  else if(powerState == HIGH){
    if(machineState == 0){
      *port_c &= ~(0x05);
      *port_g &= ~(0x05);
      *port_c |= 0x01 ;
      sendTime();
      moveStepper();
      display_Temp_Humid();
      motorOFF();
      monitorTemp();
      monitorWater();
    }
    else if(machineState == 1){
      sendTime();
      *port_c &= ~(0x05);
      *port_g &= ~(0x05);
      *port_g |= 0x04;
      moveStepper();
      display_Temp_Humid();
      motorON();
      monitorTemp();
      monitorWater();
    }
    else if (machineState == 2){
      sendTime();
      displayError();
      motorOFF();
      *port_c &= ~(0x05);
      *port_g &= ~(0x05);
      *port_g |= 0x01;
      monitorWater();
      resetbutton();
    }
  }
}

//Serial Functions
void U0init(int U0baud){
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

unsigned char U0kbhit(){
  return *myUCSR0A & RDA;
}


unsigned char U0getchar(){
  return *myUDR0;
}

void U0putchar(unsigned char U0pdata){
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

//ADC Functions
void adc_init(){
  *my_ADCSRA |= 0b10000000;
  *my_ADCSRA &= 0b11011111;
  *my_ADCSRA &= 0b11110111;
  *my_ADCSRA &= 0b11111000;
  *my_ADCSRB &= 0b11110111;
  *my_ADCSRB &= 0b11111000;
  *my_ADMUX  &= 0b01111111;
  *my_ADMUX  |= 0b01000000;
  *my_ADMUX  &= 0b11011111;
  *my_ADMUX  &= 0b11100000;
}

unsigned int adc_read(unsigned char adc_channel_num){
  *my_ADMUX  &= 0b11100000;
  *my_ADCSRB &= 0b11110111;

  if(adc_channel_num > 7){
    adc_channel_num -= 8;
    *my_ADCSRB |= 0b00001000;
  }

  *my_ADMUX  += adc_channel_num;
  *my_ADCSRA |= 0x40;

  while((*my_ADCSRA & 0x40) != 0);

  return *my_ADC_DATA;
}

//Timer Functions
void my_delay(unsigned int (freq)){
  double period = 1.0/double(freq);
  double half_period = period/ 2.0;
  double clk_period = 0.0000000625;
  unsigned int ticks = half_period / clk_period;

  *myTCCR1B &= 0xF8;
  *myTCNT1 = (unsigned int) (65536 - ticks);
  *myTCCR1A = 0x0;
  *myTCCR1B |= 0x04;

  while((*myTIFR1 & 0x01)==0);
  *myTCCR1B &= 0xF8;      
  *myTIFR1 |= 0x01;
}

//Start/Stop Button
void start_stop(){
  sent = false;
  if(powerState == LOW ){
    powerState = HIGH;
  }
  else if(powerState == HIGH){
    powerState = LOW;
  }
  if(sent == false){
    U0putchar(rtc.getDOWStr());
    U0putchar(' ');
    U0putchar(rtc.getDateStr());
    U0putchar(' ');
    U0putchar(rtc.getTimeStr());
    U0putchar('\n');
    sent = true;
  }
}

//Reset Button
void resetbutton(){
  int reset;

  if (*pin_h & 0x10){
    reset = 1;
  }
  else if(reset != restate){
    if ((reset == 1) && (adc_read(8) >= 27)){
      prevState = machineState;
      machineState = 0;
      setDisplay_Temp_Humid();
    }
    else{
      prevState = machineState;
      machineState = 2;
    }
  }
  else{
    reset = 0;
  }
  
  restate = reset;
}

//Temperature and Humidity Display Functions
void display_Temp_Humid(){
  mill = millis();

  if ((mill - startmill) >= timer){
    float temperature = DHT11.readTemperature();
    float humidity = DHT11.readHumidity();

    lcd.clear();
    lcd.setCursor(0,0); 
    lcd.print("Temp: ");
    lcd.print(temperature);
    lcd.print((char)223);
    lcd.print("C");
    lcd.setCursor(0,1);
    lcd.print("Humidity: ");
    lcd.print(humidity);
    lcd.print("%");
    startmill = mill;
  }
}

void setDisplay_Temp_Humid(){
  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print("Temp: ");
  lcd.print(0.0);
  lcd.print((char)223);
  lcd.print("C");
  lcd.setCursor(0,1);
  lcd.print("Humidity: ");
  lcd.print("0.0");
  lcd.print("%");
}

//Move Stepper
void moveStepper(){
  int value = adc_read(5);
  Stepper.step(value - stepVal);
  stepVal = value;
  my_delay(40);
}

//Send Time
void sendTime(){
  check_state();
  if (change == 1)
  {
    if (sent == false)
    {
    U0putchar(rtc.getDOWStr());
    U0putchar(' ');
    U0putchar(rtc.getDateStr());
    U0putchar(' ');
    U0putchar(rtc.getTimeStr());
    U0putchar('\n');
    my_delay(40);
    sent = true;
    }
  }
}

//Monitor Temperature
void monitorTemp(){
  float temperature = DHT11.readTemperature();

  if (temperature > 23.0){
    prevState = machineState;
    machineState = 1;
  }
  else if (temperature <= 22.0){
    prevState = machineState;
    machineState = 0;
  }
  my_delay(40);
}

//Monitor Water Level
void monitorWater(){
  int water = adc_read(8);

  if(water < 10){
    prevState = machineState;
    machineState = 2;
  }
  my_delay(30);
}

//Display Error 
void displayError(){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("ERROR LOW WATER");
}

//Motor On
void motorON(){
  *port_a &= ~(0x02);
  *port_a |= (0x02);
}

//Motor Off
void motorOFF(){
  *port_a &= ~(0x02);
}

//Check State Change
void check_state(){
  if (machineState == prevState){
    change = 0;
    sent = true;
  }
  else if (machineState != prevState){
    change = 1;
    sent = false;
  }
}