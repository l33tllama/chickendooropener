#include <avr/sleep.h>
#include <avr/power.h>

#include <LiquidCrystal.h>
#include <Wire.h>
#include <Button.h>
#include <TimeLord.h>
#include "RTClib.h"
#include <EEPROM.h>

// Pins
#define PIN_ENC_A     7
#define PIN_ENC_B     8
#define PIN_ENC_BTN   6
#define PIN_PIEZO     9
#define PIN_DOOR_UP   A0
#define PIN_DOOR_DN   A1
#define PIN_LIMSW_UP  A2
#define PIN_LIMSW_DN  A3

#define DEFAULT_SUNRISE_PD_MINS 20
#define DEFAULT_SUNSET_D_MINS   30

#define CKSUM_SECRET    0b0111
// time delay before sunrise (in case chickens want to get out early)
// default - 20 - 0001 0100 - cksum - 0010
#define SUNRISE_DT_0_P  0b0010
#define SUNRISE_DT_1_P  0b0011
#define SUNRISE_DT_C_P  0b0100
#define SUNRISE_DT_0_V  0b0001
#define SUNRISE_DT_1_V  0b0100
#define SUNRISE_DT_C_V  0b0010

// time delay after sunset (to be very sure that all chickens are back in..)
// default - 30 - 0001 1110 - cksum: 1000
#define SUNSET_DT_0_P   0b0101
#define SUNSET_DT_1_P   0b0110
#define SUNSET_DT_C_P   0b0111
#define SUNSET_DT_0_V   0b0001
#define SUNSET_DT_1_V   0b1110
#define SUNSET_DT_C_V   0b1000

#define DEBOUNCE_MS 20
#define PULLUP false
#define INVERT false

volatile int f_timer=0;

RTC_DS1307 rtc;

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

DateTime now;

TimeLord tardis;

float const LATITUDE = -42.9166667;
float const LONGITUDE = 147.3333282;

byte today[6];
byte sunrise[6];
byte sunset[6];

bool isDayTime = false;
bool doorOpen = false;

unsigned int delay_time_before_sunrise;
unsigned int delay_time_after_sunset;

void readEEPROM() {

  byte sunrise_dt_v0 = EEPROM.read(SUNRISE_DT_0_P);
  byte sunrise_dt_v1 = EEPROM.read(SUNRISE_DT_1_P);
  byte sunrise_dt_c  = EEPROM.read(SUNRISE_DT_C_P);

  byte sunset_dt_v0 = EEPROM.read(SUNSET_DT_0_P);
  byte sunset_dt_v1 = EEPROM.read(SUNSET_DT_1_P);
  byte sunset_dt_c  = EEPROM.read(SUNSET_DT_C_P);

  byte sunrise_dt_c_calc = CKSUM_SECRET ^ sunrise_dt_v0;
  sunrise_dt_c_calc ^= sunrise_dt_v1;
  
  int precalc_sunrise_c = SUNRISE_DT_C_V;
  
  Serial.print("Sunrise checksum is supposed to be: ");
  Serial.println(precalc_sunrise_c);
  Serial.print("Calculated checksum: ");
  Serial.println(sunrise_dt_c_calc);

  if(sunrise_dt_c_calc != precalc_sunrise_c){
    Serial.println("Writing defeault values for sunrise delay.");

    EEPROM.write(SUNRISE_DT_0_P, SUNRISE_DT_0_V);
    EEPROM.write(SUNRISE_DT_1_P, SUNRISE_DT_1_V);
    EEPROM.write(SUNRISE_DT_C_P, SUNRISE_DT_C_V);
    Serial.println("Done.");
  }

  byte sunset_dt_c_calc = CKSUM_SECRET ^ sunset_dt_v0;
  sunset_dt_c_calc ^= sunset_dt_v1;

  int precalc_sunset_c = (int)SUNSET_DT_C_V;
  
  Serial.print("Sunset checksum is supposed to be:");
  Serial.println(precalc_sunset_c);
  Serial.print("Calculated checksum: ");
  Serial.println(sunset_dt_c_calc);

  if(sunset_dt_c_calc != precalc_sunset_c){
    Serial.println("Writing default values for sunset delay.");
    EEPROM.write(SUNSET_DT_0_P, SUNSET_DT_0_V);
    EEPROM.write(SUNSET_DT_1_P, SUNSET_DT_1_V);
    EEPROM.write(SUNSET_DT_C_P, SUNSET_DT_C_V);
    Serial.println("Done.");
  }
}


ISR(TIMER1_OVF_vect)
{
  /* set the flag. */
   if(f_timer == 0)
   {
     f_timer = 1;
   }
}

void enableInterrupts(){
  
  /* Normal timer operation.*/
  TCCR1A = 0x00; 
  
  /* Clear the timer counter register.
   * You can pre-load this register with a value in order to 
   * reduce the timeout period, say if you wanted to wake up
   * ever 4.0 seconds exactly.
   */
  TCNT1=0x0000; 
  
  /* Configure the prescaler for 1:1024, giving us a 
   * timeout of 4.09 seconds.
   */
  TCCR1B = 0x05;
  
  /* Enable the timer overlow interrupt. */
  TIMSK1=0x01;
}

void setupPins(){
  pinMode(PIN_DOOR_UP, OUTPUT);
  pinMode(PIN_DOOR_DN, OUTPUT);
  pinMode(PIN_LIMSW_UP, INPUT);
  pinMode(PIN_LIMSW_DN, INPUT);
}

void showSplash(){
  // Splash to remind the user who made this!
  lcd.print("Door Timer ");
  lcd.setCursor(0, 1);
  lcd.print("by Leo Febey");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Made at Hobart");
  lcd.setCursor(0, 1);
  lcd.print("Hackerspace!");
  delay(1000);
  lcd.clear();
}


void setup() {

  enableInterrupts();
  setupPins();

  tardis.TimeZone(11 * 60);
  tardis.Position(LATITUDE, LONGITUDE);

  // setup serial
  Serial.begin(9600);

  // setup LCD
  lcd.begin(16, 2);

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  // setup RTC
  if (! rtc.isrunning()) {
    Serial.println("RTC is not running! PANIC!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Date testing using Datetime from RTC library
  now = rtc.now();

  // temp for setting RTC date - only for compilation time! 
  // Remember to re-flash with this code commented out to stop the 
  // Arduino restting the time to compile time every reboot..
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // read eeprom settings (delays, ?)
  readEEPROM();

  showSplash();

  // testing..
  Serial.print("Date: ");
  Serial.print(now.day());
  Serial.print(" : ");
  Serial.print(now.month(), DEC);
  Serial.print(" : ");
  Serial.print(now.year(), DEC);
  Serial.print("\nTime: ");
  Serial.print(now.hour(), DEC);
  Serial.print(" : ");
  Serial.print(now.minute(), DEC);
  Serial.print(" : ");
  Serial.print(now.second(), DEC);
  Serial.print("\n");

  

  // testing..
  checkTime();
  openDoor();
  closeDoor();
}

void enterSleep() {
  set_sleep_mode(SLEEP_MODE_IDLE);
  
  sleep_enable();


  /* Disable all of the unused peripherals. This will reduce power
   * consumption further and, more importantly, some of these
   * peripherals may generate interrupts that will wake our Arduino from
   * sleep!
   */
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer2_disable();
  power_twi_disable();  

  /* Now enter sleep mode. */
  sleep_mode();
  
  /* The program will continue from here after the timer timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */
  
  /* Re-enable the peripherals. */
  power_all_enable();
}

void openDoor(){
  Serial.println("Opening the door..");
  // send high to door open motor pin
  digitalWrite(PIN_DOOR_UP, HIGH);

  int timeoutcount = 0;
  const int TIMEOUT_MAX = 220;


  // loop until limit switch is hit (also add timeout)
  Serial.print("Waiting for door to open");
  while(digitalRead(PIN_LIMSW_UP) == LOW){
    enterSleep(); // sleep for a few seconds to save power
    if(f_timer == 1)
    {
      f_timer = 0;
    }
    timeoutcount++;
    if(timeoutcount > TIMEOUT_MAX){
      Serial.print("\nERROR! door not closing properly. Check limit switch.\n");
      Serial.print("Pretending that it's closed for now..");
      break;
    }
    Serial.print(".");
  }
  Serial.println();

  // send low to door open motor pin
  digitalWrite(PIN_DOOR_UP, LOW);
  
  Serial.println("Door opened!!");
  // set door is open bool to true
  doorOpen = true;

}

void closeDoor(){
  // send high to door close motor pin
   digitalWrite(PIN_DOOR_DN, HIGH);

  int timeoutcount = 0;
  const int TIMEOUT_MAX = 220;
   
  // loop until limit switch is hit  (also add timeout)
  Serial.print("Waiting for door to close");
  while(digitalRead(PIN_LIMSW_DN) == LOW){
    enterSleep(); // sleep for a few seconds to save power
    if(f_timer == 1)
    {
      f_timer = 0;
    }
    timeoutcount++;
    if(timeoutcount > TIMEOUT_MAX){
      Serial.print("\nERROR! door not opening properly. Check limit switch.\n");
      Serial.print("Pretending that it's open for now..");
      break;
    }
     Serial.print(".");
  }
  Serial.println();
  
  // send low to door close motor pin
  digitalWrite(PIN_DOOR_DN, LOW);
  Serial.println("Door closed!!");
  // set door is open bool to false
  doorOpen = false;
  
}

void updateDoor(){
  if(isDayTime && !doorOpen){
    Serial.print("It's daytime and we are now opening the door!");
    openDoor();  
  } else if (!isDayTime && doorOpen){
    Serial.print("It's night time and we are now closing the door!");
    closeDoor();
  }
}

void checkTime(){
  //today =  {now.second(), now.minute(), now.hour(), now.day(), now.month(), now.year() } ;
  // check if it is time to open the door

  now = rtc.now();
  today[0] = now.second();
  today[1] = now.minute();
  today[2] = now.hour();
  today[3] = now.day();
  today[4] = now.month();
  today[5] = now.year();

  memcpy(sunrise, today, sizeof(byte) * 6);
  memcpy(sunset, today, sizeof(byte) * 6);

  bool evening = false;
  if (tardis.SunSet(sunset)) // if the sun will set today (it might not, in the [ant]arctic)
  {
    Serial.print("Sunset: ");
    Serial.print((int) sunset[tl_hour]);
    Serial.print(":");
    Serial.println((int) sunset[tl_minute]);

    int delta_mins = (now.hour() * 60 - sunset[tl_hour] * 60) + now.minute() - sunset[tl_minute];

    if(delta_mins > 0){
      Serial.print("Time after sunset: ");
      Serial.print((int)delta_mins / 60);
      Serial.print(":");
      Serial.println(delta_mins % 60);
      isDayTime = false;
    } else {
      Serial.print("Time until sunset: ");
      Serial.print(-delta_mins / 60);
      Serial.print(":");
      Serial.println(-delta_mins % 60);
      isDayTime = true;
    }
  }
  // get day for tomorrow if the sun has gone down already
  if(!isDayTime){
    // If in the evening, before the next day
    if(now.hour() > sunset[tl_hour]){
      Serial.println("Checking sunrise for tomorrow..?");
      // get tomorrow's sunrise
      DateTime tomorrow = now + TimeSpan(1, 0, 0, 0);
      sunrise[0] = tomorrow.second();
      sunrise[1] = tomorrow.minute();
      sunrise[2] = tomorrow.hour();
      sunrise[3] = tomorrow.day();
      sunrise[4] = tomorrow.month();
      sunrise[5] = tomorrow.year();
      evening = true;
    }
  }
  if (tardis.SunRise(sunrise)) // if the sun will rise today (it might not, in the [ant]arctic)
  {
    // Old, computationally expensive method
    //DateTime sunrise_dt(sunrise[5], sunrise[4], sunrise[3], sunrise[2], sunrise[1], sunrise[0]);
    //DateTime midnight(now.year(), now.month(), now.day(), 0, 0, 0);
    //DateTime now_plus_sunrise = now - (sunrise_dt - midnight);
    
    Serial.print("Sunrise: ");
    Serial.print((int) sunrise[tl_hour]);
    Serial.print(":");
    Serial.println((int) sunrise[tl_minute]);
    int delta_mins;
    if(!isDayTime){
      if(evening){
        // get total mins including minutes before midnight
        delta_mins = (24 - now.hour() + sunrise[tl_hour]) * 60 + (sunrise[tl_minute] + (60-now.minute()));
      } else { /* morning */
        // sunrise - now
        delta_mins = (sunrise[tl_hour] - now.hour()) * 60 + (sunrise[tl_minute] - now.minute()); 
      }
    } else {
      // time after sunrise - will be negative so below can say time after sunrise
      delta_mins = (sunrise[tl_hour] * 60 - now.hour() * 60) + sunrise[tl_minute] - now.minute();  
    }
    
    if(delta_mins < 0){
      Serial.print("Time after sunrise: ");
      Serial.print((int)-delta_mins / 60);
      Serial.print(":");
      Serial.println(-delta_mins % 60);
    } else {
      Serial.print("Time until sunrise: ");
      Serial.print(delta_mins / 60);
      Serial.print(":");
      Serial.println(delta_mins % 60);
    }
  }
  Serial.println();
  
}

void tensDigitLCD(int digit){
  if(digit / 10 < 1){
    lcd.print("0");
  }
  lcd.print(digit);
}

void updateLCD(){
 lcd.clear();
 tensDigitLCD(now.day());
 lcd.print("/");
 tensDigitLCD(now.month());
 lcd.print("/");
 lcd.print(now.year());
 lcd.print(" ");
 tensDigitLCD(now.hour());
 lcd.print(":");
 tensDigitLCD(now.minute());
  if(isDayTime){  
    lcd.setCursor(0, 1);
    lcd.print("Sunset:  ");
    tensDigitLCD(sunset[tl_hour]);
    lcd.print(":");
    tensDigitLCD(sunset[tl_minute]);
  } else {
    lcd.setCursor(0, 1);
    lcd.print("Sunrise: ");
    tensDigitLCD(sunrise[tl_hour]);
    lcd.print(":");
    tensDigitLCD(sunrise[tl_minute]);
  }
}

void loop() {
  if(f_timer==1)
  {
    f_timer = 0;
    Serial.println("Waking from sleep..");
    checkTime();
    updateDoor();
    updateLCD();
    /* Re-enter sleep mode. */
    enterSleep();
  }  
}
 
