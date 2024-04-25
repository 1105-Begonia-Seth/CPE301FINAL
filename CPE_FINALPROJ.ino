// Seth Begonia
// CPE 301 Final Project
// April 11 2024

#define RDA 0x80
#define TBE 0x20  

// DHT Library
#include <DHT.h>
#include <DHT_U.h> 
#define DHT11_PIN 44
DHT dht11(DHT11_PIN, DHT11);

// Real Time Clock Library
#include <RTClib.h>
RTC_DS1307 rtc;

// LCD Library
#include <LiquidCrystal.h> 
const int RS = 52, EN = 53, D4 = 50, D5 = 51, D6 = 48, D7 = 49;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

// Interrupt code
const byte interruptPin = 2;
volatile byte state = LOW;

// Water sensor start
int value = 0;

// Timer stuff 
volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;

// ADC Stuff
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
 
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;
// ADC Stuff

// A IS THE OUTPUTS
volatile unsigned char* port_a = (unsigned char*) 0x22;
volatile unsigned char* ddr_a = (unsigned char*) 0x21;
volatile unsigned char* pin_a = (unsigned char*) 0x20;

// B IS THE OUTPUTS
volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* ddr_b = (unsigned char*) 0x24;
volatile unsigned char* pin_b = (unsigned char*) 0x23;


// H IS THE OUTPUTS
volatile unsigned char* port_h = (unsigned char*) 0x102;
volatile unsigned char* ddr_h = (unsigned char*) 0x101;
volatile unsigned char* pin_h = (unsigned char*) 0x100;


// RTC CODE
char daysOfTheWeek[7][12] = {
  "Sunday",
  "Monday",
  "Tuesday",
  "Wednesday",
  "Thursday",
  "Friday",
  "Saturday"
};

void setup() {

Serial.begin(9600);

  adc_init(); // INITIALIZE THE ADC!!!!!

  // RTC CODE
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1);
  }

  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // RTC CODE

  // DHT CODE
  dht11.begin();

  // LCD SETUP
  lcd.begin(16, 2);

// LED setup
// pinMode(10, OUTPUT); PB4
// Red LED
*ddr_b |= 0x01 << 4;

// pinMode(11, OUTPUT); PB5
// Yellow LED
*ddr_b |= 0x01 << 5;

// pinMode(12, OUTPUT); PB6
// Blue LED
*ddr_b |= 0x01 << 6;

// pinMode(13, OUTPUT); PB7
// Green LED
*ddr_b |= 0x01 << 7;
// LED setup

// Water Level sensor
// pinMode(POWER_PIN, OUTPUT); configure D7 pin as an OUTPUT
  *ddr_h |= 0x01 << 4;

// digitalWrite(POWER_PIN, LOW); // turn the sensor OFF
  write_PH(4, 0); 
// Water Level sensor

// Fan setup
  // pinMode(23, OUTPUT);
  *ddr_a |= 0x01 << 1;
  
  //pinMode(25, OUTPUT);
  *ddr_a |= 0x01 << 3;
// Fan setup

// Interrupt setup
pinMode(interruptPin, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(interruptPin), blink, RISING);

}

void loop() {

  my_delay(1000);

  // DHT sensor readings
  float humi  = dht11.readHumidity();
  float tempF = dht11.readTemperature(true);

  // Take water sensor reading from function into variable
  int water = wlRead();

  // Check time funtion
  rtClock();

  // Idle state when interrupt is pressed
  if(state == HIGH){

      // Check temp and water sensor, if thresholds are met, continue idle state
      if(tempF < 76 && water > 120){
        gLED();
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Humidity: ");
        lcd.print(humi);
        lcd.print("%");
        lcd.setCursor(0,1);
        lcd.print("Temp: ");
        lcd.print(tempF);
        lcd.print((char)223);
        lcd.print("F");
        my_delay(500);

      // When temp threshold is not met but water is met, then fan until it is lowered
      }else if(tempF > 76 && water > 120){
        bLED();
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Humidity: ");
        lcd.print(humi);
        lcd.print("%");
        lcd.setCursor(0,1);
        lcd.print("Temp: ");
        lcd.print(tempF);
        lcd.print((char)223);
        lcd.print("F");
        Serial.println("Fan is On");
        write_PA(1, 0);
        write_PA(3, 1);
        my_delay(500);

      // When water is too low, error on LCD, "Water level too low" on serial monitor and press reset button to go back to idle
      }else if(water < 120){
        rLED();
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("ERROR ");
        Serial.println("Water level too low");
        my_delay(1000);
      }
      
  // Disabled state
  }else if(state == LOW){

    	yLED();

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Cooler off,press");
      lcd.setCursor(0,1);
      lcd.print("button to start");

  }

}

void gLED(){
  write_PB(7, 1);
  write_PB(6, 0);
  write_PB(5, 0);
  write_PB(4, 0);
}

void bLED(){
  write_PB(7, 0);
  write_PB(6, 1);
  write_PB(5, 0);
  write_PB(4, 0);
}

void yLED(){
  write_PB(7, 0);
  write_PB(6, 0);
  write_PB(5, 1);
  write_PB(4, 0);
}

void rLED(){
  write_PB(7, 0);
  write_PB(6, 0);
  write_PB(5, 0);
  write_PB(4, 1);
}

void rtClock(){
  DateTime now = rtc.now();
  Serial.print("Date & Time: ");
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.println(now.second(), DEC);
}

void blink() {
  state = !state;
}

// Water level reading
int wlRead(){
    // digitalWrite(POWER_PIN, HIGH);  // turn the sensor ON
  write_PH(4, 1); 

  my_delay(10);                      // wait 10 milliseconds

  value = adc_read(5); // read the analog value from sensor

  // digitalWrite(POWER_PIN, LOW); // turn the sensor OFF
  write_PH(4, 0); 

  return value;
}

void write_PA(unsigned char pin_num, unsigned char state){
  if(state == 0)
    {
      *port_a &= ~(0x01 << pin_num);
    }
    else
    {
      *port_a |= 0x01 << pin_num;
    }
} 

void write_PB(unsigned char pin_num, unsigned char state){
  if(state == 0)
    {
      *port_b &= ~(0x01 << pin_num);
    }
    else
    {
      *port_b |= 0x01 << pin_num;
    }
}

void write_PH(unsigned char pin_num, unsigned char state){
  if(state == 0)
    {
      *port_h &= ~(0x01 << pin_num);
    }
    else
    {
      *port_h |= 0x01 << pin_num;
    }
}

// Given timer function
void my_delay(unsigned int freq)
{
  // calc period
  double period = 1.0/double(freq);
  // 50% duty cycle
  double half_period = period/ 2.0f;
  // clock period def
  double clk_period = 0.0000000625;
  // calc ticks
  unsigned int ticks = half_period / clk_period;
  // stop the timer
  *myTCCR1B &= 0xF8;
  // set the counts
  *myTCNT1 = (unsigned int) (65536 - ticks);
  // start the timer
  * myTCCR1A = 0x0;
  * myTCCR1B |= 0b00000001;
  // wait for overflow
  while((*myTIFR1 & 0x01)==0); // 0b 0000 0000
  // stop the timer
  *myTCCR1B &= 0xF8;   // 0b 0000 0000
  // reset TOV           
  *myTIFR1 |= 0x01;
}

void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void U0init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
unsigned char U0kbhit()
{
  return *myUCSR0A & RDA;
}
unsigned char U0getchar()
{
  return *myUDR0;
}
void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}