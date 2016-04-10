/*  Chicken door timer (v2)
 *  By Leo Febey 2016
 *  leofebey (at) gmail (dot) com
 *  
 *  Program that automatically opens and closes a door on sunrise / sunset.
 *  User can also set a delay before and after sunrise and sunset to 
 *  ensure chickens (or other animals) are very certainly inside.
 *  
 *  Requires Button.h, TimeLord.h, RTClib.h (Adafruit), and included encoder.h
 */

// TODO: thoroughly read through code to ensure it's all good.
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include <LiquidCrystal.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Button.h>
#include <TimeLord.h>
#include "RTClib.h"
#include "encoder.h"
//#define DEBUG_EN true
#define DEBUG_EN_S true
#define ERR_EN true
#include "main.h"

#define LATITUDE -42.9166667
#define LONGITUDE 147.3333282

#define SERIAL_WAIT 6000
#define TIMEOUT_MAX 220
#define DELAY_MS 10
#define MAX_DELAY_MINUTES 100
#define DOOR_RETRY_SECONDS 10

#define DOOR_ERR_BEEP_TIME 450
#define DOOR_ERR_MSG1_TIME 2000
#define DOOR_ERR_MSG2_TIME 4000
#define DOOR_ERR_MSG3_TIME 7500
#define DOOR_ERR_PAUSE_TIME 12500

#define DOOR_MNTNC_TIME_DAYTIME 45 * 60000
#define DOOR_MNTNC_TIME_NIGHTIME 35 * 60000

// Pins
#define PIN_ENC_A     7
#define PIN_ENC_B     8
#define PIN_ENC_BTN   6
#define PIN_PIEZO     9
#define PIN_DOOR_UP   A0
#define PIN_DOOR_DN   A1
#define PIN_LIMSW_UP  A2
#define PIN_LIMSW_DN  A3

// Button settings
#define DEBOUNCE_MS 20
#define PULLUP false
#define INVERT false

#define DEFAULT_SUNRISE_PD_MINS 20
#define DEFAULT_SUNSET_D_MINS   30

#define MENU_COUNT 2

volatile byte f_timer=0;

char menu_pos = 0;
byte delay_val = 0;
byte door_retry_seconds_c = 0;

Button menuButton(PIN_ENC_BTN, PULLUP, INVERT, DEBOUNCE_MS);

RTC_DS1307 rtc;

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

DateTime now;

TimeLord tardis;

Encoder encoder(PIN_ENC_A, PIN_ENC_B, PIN_ENC_BTN);

byte sunrise[6];
byte sunset[6];

// process logic booleans
bool is_day_time = false;
bool door_open = false;
bool in_options_menu = false;
bool in_main_menu = false;
bool in_timeset_menu = false;
bool in_pre_delay_menu = false;
bool in_post_delay_menu = false;
bool in_time_adj_menu = false;
bool in_delay_adj_menu = false;

bool hit_bottom = false;
bool hit_top = false;
bool couldnt_close_door = false;
bool couldnt_open_door = false;

byte delay_time_before_sunrise;
byte delay_time_after_sunset;


/* Read EEPROM settings
 * Mainly just delay before/after sunrise/sunset. 
 * Additional settings may be added later.
 * Includes checksum verification for if the EEPROM starts to die after a while 
 * (unlikely but could cause serious havok)
 *  
 */
void readEEPROM() {

  // read eeprom values for sunrise and sunset delays
  byte sunrise_dt_v0 = EEPROM.read(SUNRISE_DT_0_P);
  byte sunrise_dt_v1 = EEPROM.read(SUNRISE_DT_1_P);
  byte sunrise_dt_c  = EEPROM.read(SUNRISE_DT_C_P);

  byte sunset_dt_v0 = EEPROM.read(SUNSET_DT_0_P);
  byte sunset_dt_v1 = EEPROM.read(SUNSET_DT_1_P);
  byte sunset_dt_c  = EEPROM.read(SUNSET_DT_C_P);

  // check against checksum that was pre-calculated by hand and saved as a final variable
  byte sunrise_dt_c_calc = CKSUM_SECRET ^ sunrise_dt_v0;
  sunrise_dt_c_calc ^= sunrise_dt_v1;

  char eeprom_success = 0x00;

  // if checksum mismatch - something went wrong with EEPROM writing. Try to reset 
  if(sunrise_dt_c_calc != sunrise_dt_c){
    lcd.clear();
    lcd.print("EEPROM checksum");
    lcd.setCursor(0, 1);
    lcd.print("error :(");
    ERRLN("EEPROM checksum error - Writing defeault values for sunrise delay.");
    EEPROM.write(SUNRISE_DT_0_P, SUNRISE_DT_0_V);
    EEPROM.write(SUNRISE_DT_1_P, SUNRISE_DT_1_V);
    EEPROM.write(SUNRISE_DT_C_P, SUNRISE_DT_C_V);
    DEBUGLN("Done.");
    delay_time_before_sunrise = (SUNRISE_DT_0_V << 4) + SUNRISE_DT_1_V;
  } else {
    delay_time_before_sunrise = (sunrise_dt_v0 << 4) + sunrise_dt_v1;
    eeprom_success += 0x01;
  }

  // check checksum of sunset time from EEPROM
  byte sunset_dt_c_calc = CKSUM_SECRET ^ sunset_dt_v0;
  sunset_dt_c_calc ^= sunset_dt_v1;

  if(sunset_dt_c_calc != sunset_dt_c){
    lcd.clear();
    lcd.print("EEPROM checksum");
    lcd.setCursor(0, 1);
    lcd.print("error :(");
    DEBUGLN("Writing default values for sunset delay.");
    EEPROM.write(SUNSET_DT_0_P, SUNSET_DT_0_V);
    EEPROM.write(SUNSET_DT_1_P, SUNSET_DT_1_V);
    EEPROM.write(SUNSET_DT_C_P, SUNSET_DT_C_V);
    DEBUGLN("Done.");
    delay_time_after_sunset = (SUNSET_DT_0_V << 4) + SUNSET_DT_1_V;
  } else {
    delay_time_after_sunset = (sunset_dt_v0 << 4) + sunset_dt_v1;
    eeprom_success += 0x10;
  }

  if(eeprom_success == 0x11){
    DEBUGLN("EEPROM read successfully.");
  } else{
    ERRLN("Error reading EEPROM!");
  }
}

// Interrupt routine tat resets the sleep flag
ISR(TIMER1_OVF_vect)
{
  /* set the flag. */
   if(f_timer == 0)
   {
     f_timer = 1;
   }
}

// Set interrupt registers
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
  pinMode(PIN_ENC_BTN, INPUT);
  pinMode(PIN_PIEZO, OUTPUT);
}

//LCD splash (Shouldn't be seen too much hopefully..)
void showSplash(){
  // Splash to remind the user who made this!
  tone(PIN_PIEZO, 1000, 500);
  lcd.print("Door Timer ");
  lcd.setCursor(0, 1);
  lcd.print("by Leo Febey");
  delay(1000);
  tone(PIN_PIEZO, 500, 500);
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
  
  wdt_disable();

  // setup sunrise/sunset calculator library (so good..) 
  tardis.TimeZone(11 * 60);
  tardis.Position(LATITUDE, LONGITUDE);

  // setup serial
  Serial.begin(9600);
  DEBUGLN("Serial initialised.");

  // setup LCD
  lcd.begin(16, 2);

  // setup RTC
  if (! rtc.begin()) {
    ERRLN("Couldn't find RTC");
    while (1);
  }
  
  if (! rtc.isrunning()) {
    ERRLN("RTC is not running! PANIC!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  
  //waitForSerialConnection();

  // Date testing using Datetime from RTC library
  now = rtc.now();

  // temp for setting RTC date - only for compilation time!
  // Remember to re-flash with this code commented out to stop the
  // Arduino restting the time to compile time every reboot..
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // read eeprom settings (delays, ?)
  readEEPROM();

  // Show splash (when in release mode..)
#ifdef DEBUG_EN
#else
  showSplash();
#endif
  
  // Initial time check to start things rolling..
  checkTime();

  

  // enable watchdog timer
  wdt_enable(WDTO_4S);
}

//
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


// parse input from serial
void parseInput(char * input){
  DEBUGSLN("Entered: %s\n");
  
}

// check for user input via serial
// if user enters 'i' within allowed time, start accepting input commands
void waitForSerialConnection(){
  
  volatile long int serial_wait_c = SERIAL_WAIT;
  char * input_str = (char *) malloc(sizeof(char) * 32);
  bool read_line = false;
  bool accepting_input = false;
  
  Serial.print("Enter i to enter serial command mode.\n");
  
  while(serial_wait_c > 0 ){
    while(Serial.available() > 0){
      char in_c = Serial.read();
      if (in_c == 'i'){
	accepting_input = true;
      }
    }
    _delay_ms(10);
    serial_wait_c -= 10;
  }
  
  if(serial_wait_c == 0 || !accepting_input){
    Serial.print("Serial command mode timeout..\n");
  }
  
  // if user entered 'i'
  // TODO: add timeout for this, longer - eg 20-30 seconds
  if(accepting_input){
    while(!read_line){
      char in_c;
      while(Serial.available() > 0){
	in_c = Serial.read();
      }
      
      // if user pressed enter, finish up
      if(in_c == '\n' || in_c == '\r'){
	read_line = true;
	parseInput(input_str);
	break;
      } else {
	*(input_str)++ = Serial.read();
      }
    }
  }
}

// Draw a number on the lcd with a leding zero
void tensDigitLCD(byte digit){
  if(digit / 10 < 1){
    lcd.print('0');
  }
  lcd.print((int)digit);
}

// Opens the door (with timeout)
void openDoor(){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Opening door");
  DEBUGLN("Opening the door..");
  // send high to door open motor pin (open the door)
  digitalWrite(PIN_DOOR_UP, HIGH);

  byte timeoutcount = 0;

  // loop until limit switch is hit (also add timeout)
  DEBUGLN("Waiting for door to open");
  while(digitalRead(PIN_LIMSW_UP) == LOW){
    wdt_reset();
    delay(DELAY_MS);
    timeoutcount++;
    if(timeoutcount > TIMEOUT_MAX){
      ERRLN("\nERROR! door not closing properly. Check limit switch.\n");
      ERRLN("Pretending that it's closed for now..");
      couldnt_open_door = true;
      break;
    }
    DEBUG(".");
  }
  if(timeoutcount <= TIMEOUT_MAX){
    couldnt_open_door = false;
  }
  hit_top = true;
  DEBUGLN();

  // send low to door open motor pin (stop)
  digitalWrite(PIN_DOOR_UP, LOW);

  DEBUGLN("Door opened!!");
  // set door is open bool to true
  door_open = true;

}

// Closes the door (with timeout)
void closeDoor(){
  // send high to door close motor pin
  digitalWrite(PIN_DOOR_DN, HIGH);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Closing door");
  byte timeoutcount = 0;

  // loop until limit switch is hit  (also add timeout)
  DEBUGLN("Waiting for door to close");
  while(digitalRead(PIN_LIMSW_DN) == LOW){
    wdt_reset();
    delay(DELAY_MS);
    timeoutcount++;
    if(timeoutcount > TIMEOUT_MAX){
      ERR("\nERROR! door not opening properly. Check limit switch.\n");
      ERRLN("Pretending that it's open for now..");
      couldnt_close_door = true;
      break;
    }
     DEBUG(".");
  }
  if(timeoutcount <= TIMEOUT_MAX){
    couldnt_close_door = false;
  }
  hit_bottom = true;
  DEBUGLN();

  // send low to door close motor pin
  digitalWrite(PIN_DOOR_DN, LOW);
  DEBUGLN("Door closed!!");
  // set door is open bool to false
  door_open = false;

}

void updateDoor(){
  if(is_day_time && !door_open){
    DEBUGLN("It's daytime and we are now opening the door!");
    openDoor();
  } else if (!is_day_time && door_open){
    DEBUGLN("It's night time and we are now closing the door!");
    closeDoor();
  }
  door_retry_seconds_c += 4;
  if(door_retry_seconds_c >= DOOR_RETRY_SECONDS){
    if(couldnt_open_door){
      openDoor();
    } else if (couldnt_close_door){
      closeDoor();
    }
    door_retry_seconds_c = 0;
  }
  
}

// Alert user to press a button, otherwise the system will try to open/close the door to the proper position

/*
 * |00|01|02|03|04|05|06|07|08|09|10|11|12|13|14|15|  |00|01|02|03|04|05|06|07|08|09|10|11|12|13|14|15|
 * |E |r |r |o |r |! |  |D |o |o |r |  |n |o |t |  |  |E |r |r |o |r |! |  |D |o |o |r |  |n |o |t |  |
 * |C |l |o |s |e |d |  |a |t |  |n |i |g |h |t |. |  |o |p |e |n |  |d |u |r |. |  |d |a |y |. |  |  |
 * 
 * |p |r |e |s |s |  |m |a |i |n |  |b |t |n |  |  |  |p |r |e |s |s |  |m |a |i |n |  |b |t |n |  |  |
 * |t |o |  |p |r |e |v |e |n |t |  |f |r |o |m |- |  |t |o |  |p |r |e |v |e |n |t |  |f |r |o |m |- |
 * 
 * |a |u |t |o |- |o |p |e |n |i |n |g |  |i |n |  |  |a |u |t |o |- |c |l |o |s |i |n |g |  |i |n |  |
 * |x |x |  |s |e |c |o |n |d |s |. |  |  |  |  |  |  |x |x |  |s |e |c |o |n |d |s |. |  |  |  |  |  |
 * 
 * |. |. | 
 */

void drawDoorErrMsg1(bool notClosed){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Error! Door not ");
  lcd.setCursor(0, 1);
  if(notClosed){
    lcd.print("closed at night.");  
  } else{
    lcd.print("open dur. day.  ");
  }
}

void drawDoorErrMsg2(){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Press main btn  ");
  lcd.setCursor(0, 1);
  lcd.print("to prevent from-");
}

void drawDoorErrMsg3(bool notClosed, byte seconds){
  lcd.clear();
  lcd.setCursor(0, 0);
  if(notClosed){
    lcd.print("auto-closing in ");  
  } else {
    lcd.print("auto-opening in ");
  }
  lcd.setCursor(0, 1);
  tensDigitLCD(seconds);
  lcd.print(" seconds.");
  
}
 
void waitForUserResetOpen(){

  // determine time allowed for user to fix door before auto-fixing
  bool notClosed = !is_day_time & hit_top;
  
  DateTime stopTime = rtc.now();
  DateTime fixTime;
  if(is_day_time && hit_bottom){
    fixTime = stopTime + TimeSpan(0, 0, 0, 6);
  } else if(notClosed){
    fixTime = stopTime + TimeSpan(0, 0, 0, 5);
  }
  
  bool pausedForUser = false;

  bool beep1 = false;
  bool msg1 = false;
  bool msg2 = false;
  bool msg3 = false;
  bool msg_looped = false;
  
  volatile int ms_count_beep = 0;
  volatile int ms_count_msgs = 0;
  volatile int ms_pause_time = 0;

  // while warning time not run out and user hasn't intervened
  while((ms_pause_time < DOOR_ERR_PAUSE_TIME) && !pausedForUser){
    menuButton.read();
    if(menuButton.wasReleased()){
      pausedForUser = true;
    }

    if((ms_count_beep > DOOR_ERR_BEEP_TIME) && !beep1){
      tone(PIN_PIEZO, 800, 50);  
      beep1 = true;
    } 
    else if(ms_count_beep > DOOR_ERR_BEEP_TIME * 2 + 50){
      tone(PIN_PIEZO, 2000, 50);  
      beep1 = false;
      ms_count_beep = 0;
    }
    
    if(!msg1 && (ms_count_msgs > 0) && 
        (ms_count_msgs < DOOR_ERR_MSG1_TIME)){
      drawDoorErrMsg1(notClosed);
      msg1 = true;
    } 
    else if(!msg2 && (ms_count_msgs > DOOR_ERR_MSG1_TIME) && 
              (ms_count_msgs < DOOR_ERR_MSG2_TIME)){
      drawDoorErrMsg2();
      msg2 = true;
    }
    else if(!msg3 && (ms_count_msgs > DOOR_ERR_MSG2_TIME) && 
              (ms_count_msgs < DOOR_ERR_MSG3_TIME)){
      drawDoorErrMsg3(notClosed, (DOOR_ERR_PAUSE_TIME - ms_pause_time) / 1000);
      msg3 = true;
    }
    else if(ms_count_msgs > DOOR_ERR_MSG3_TIME){
      ms_count_msgs = 0;   
      msg1 = false;
      msg2 = false;
      msg3 = false;
    }
    
    _delay_ms(10);
    ms_count_beep += 10;
    ms_count_msgs += 10;
    ms_pause_time += 10;
    wdt_reset();
  }

  // if user wants to operate on door or something.. Leave door open/close (for a fair while)
  if(pausedForUser){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Keeping door ");
    lcd.setCursor(0, 1);
    if(is_day_time && hit_bottom){
      lcd.print("open.");
    } else if(!is_day_time && hit_top){
      lcd.print("closed.");
    }
    delay(1500);

    const long int USER_FIX_TIME = 5 * 60000;
    const int USER_FIX_LCD_UPDATE_TIME = 1000;

    volatile long int fix_time_c = USER_FIX_TIME;

    // while waiting for user
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Paused for user");
    lcd.setCursor(0, 1);
    tensDigitLCD(((USER_FIX_TIME / 1000) / 60));
    lcd.print(" mins remain.");

    // while paused for user
    while(pausedForUser){
      wdt_reset();
      menuButton.read();
      // increase time if user pressed button
      if(menuButton.wasReleased()){
        pausedForUser = false;
      }
      // update LCD every 1 second
      if(fix_time_c % USER_FIX_LCD_UPDATE_TIME == 0){
        lcd.setCursor(0, 1);
        tensDigitLCD(((fix_time_c / 1000) / 60));
        lcd.print(":");
        tensDigitLCD((fix_time_c / 1000) % 60);
        lcd.print(" remains.");
      }
      // if time <= 0 -- time has run out!
      if(fix_time_c <= 0){
        pausedForUser = false;
      }
      fix_time_c -= 50;
      _delay_ms(50);
    }

    // If user took way too long, we should carry on! 
    // (also could be some real-world issue, eg something knocked limit switch?)
    tone(PIN_PIEZO, 2000, 50);
    delay(50);
    tone(PIN_PIEZO, 2000, 50);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("About to");
    lcd.setCursor(0, 1);
    if(is_day_time && hit_bottom){
      lcd.print("open");
    } else if(!is_day_time && hit_top){
      lcd.print("close");
    }
    lcd.print(" Door.");

    delay(1500);
  }
  // finally, open or close the door
  if(is_day_time && hit_bottom){
    openDoor();
    hit_bottom = false;
  } else if(!is_day_time && hit_top){
    closeDoor();
    hit_top = false;
  }
  
}

// check limit switches - if door is open when it should be closed
void checkLimitSwitches(){
  // If daytime, only check if door has been manually or accidentally closed
  if(is_day_time){
    DEBUGSLN("Checking if door closed at daytime.");
    // quickly power motor to check if door is hitting wrong limit switch
    // if down switch pressed and day time
    digitalWrite(PIN_DOOR_DN, HIGH);
    delay(1);
    if(digitalRead(PIN_LIMSW_DN) == HIGH){
      DEBUGSLN("Hit bottom!");
      hit_bottom = true;
    }
    digitalWrite(PIN_DOOR_DN, LOW);
    digitalWrite(PIN_DOOR_UP, HIGH);
    delay(2);
    digitalWrite(PIN_DOOR_UP, LOW);
    
    if(hit_bottom){
      waitForUserResetOpen();
    }
  } else {
    DEBUGSLN("Checking if door opened at night time.");
    
    // if up switch is pressed at day time
    digitalWrite(PIN_DOOR_UP, HIGH);
    delay(1);
    if(digitalRead(PIN_LIMSW_UP) == HIGH){
      hit_top = true;
      DEBUGSLN("Hit bottom!");
    }
    digitalWrite(PIN_DOOR_UP, LOW);
    digitalWrite(PIN_DOOR_DN, HIGH);
    delay(2);
    digitalWrite(PIN_DOOR_DN, LOW);
    
    if(hit_top){
      waitForUserResetOpen();
    }
  }
}

// Get time from RTC and calculate sunrise/sunset to display on LCD
void checkTime(){
  //today =  {now.second(), now.minute(), now.hour(), now.day(), now.month(), now.year() } ;
  // check if it is time to open the door

  byte today[6];

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
#ifdef DEBUG_EN
    Serial.print("Sunset: ");
    Serial.print((int) sunset[tl_hour]);
    Serial.print(":");
    Serial.println((int) sunset[tl_minute]);
#endif    

    char delta_mins = (now.hour() * 60 - sunset[tl_hour] * 60) + now.minute() - sunset[tl_minute];

    if(delta_mins > 0){
#ifdef DEBUG_EN      
      Serial.print("Time after sunset: ");
      Serial.print((int)delta_mins / 60);
      Serial.print(":");
      Serial.println(delta_mins % 60);
#endif      
      is_day_time = false;
    } else {
#ifdef DEBUG_EN      
      Serial.print("Time until sunset: ");
      Serial.print(-delta_mins / 60);
      Serial.print(":");
      Serial.println(-delta_mins % 60);
#endif      
      is_day_time = true;
    }

  }
  // get day for tomorrow if the sun has gone down already
  if(!is_day_time){
    // If in the evening, before the next day
    if(now.hour() > sunset[tl_hour]){
      DEBUGLN("Checking sunrise for tomorrow..?");
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

#ifdef DEBUG_EN
    Serial.print("Sunrise: ");
    Serial.print((int) sunrise[tl_hour]);
    Serial.print(":");
    Serial.print((int) sunrise[tl_minute]);
#endif
    char delta_mins;
    if(!is_day_time){
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
#ifdef DEBUG_EN
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
#endif
  }
  DEBUGLN();

}

void updateLCD(){
 lcd.clear();
 tensDigitLCD(now.day());
 lcd.print('/');
 tensDigitLCD(now.month());
 lcd.print('/');
 lcd.print(now.year());
 lcd.print(' ');
 tensDigitLCD(now.hour());
 lcd.print(':');
 tensDigitLCD(now.minute());
  if(is_day_time){
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

// update delay on lcd screen
void drawDelaySetMenu(){
  lcd.clear();
  lcd.setCursor(0, 0);
  if(in_pre_delay_menu){
    lcd.print("Pre: ");
  } else if(in_post_delay_menu){
    lcd.print("Post: ");
  }
  lcd.print((int)delay_val);
  lcd.print(" mins");
}

// update delay value loop - check encoder, if change, update lcd with new val
void updatedelay_valLoop(){
  encState eState = encoder.read();
  if (eState == ENC_DEC) {
    if (delay_val == 0) {
      delay_val = MAX_DELAY_MINUTES - 1;
    } else {
      delay_val = (delay_val - 1);
    }
    tone(PIN_PIEZO, 100 + delay_val * 12, 25);
    drawDelaySetMenu();
  } else if (eState == ENC_INC) {
    delay_val = (delay_val + 1) % (MAX_DELAY_MINUTES);
    tone(PIN_PIEZO, 100 + delay_val * 12, 25);
    drawDelaySetMenu();
  }
}

// write new delay value to EEPROM (including checksum)
void setdelay_valEEPROM(bool sunrise, byte val){
  byte val_l = (val & 0b11110000) >> 4;
  byte val_r = val & 0b00001111;
  byte checksum = CKSUM_SECRET ^ val_l;
  checksum ^= val_r;
  if(sunrise){
    EEPROM.write(SUNRISE_DT_0_P, val_l);
    EEPROM.write(SUNRISE_DT_1_P, val_r);
    EEPROM.write(SUNRISE_DT_C_P, checksum);
  } else {
    EEPROM.write(SUNSET_DT_0_P, val_l);
    EEPROM.write(SUNSET_DT_1_P, val_r);
    EEPROM.write(SUNSET_DT_C_P, checksum);
  }
}

// when user selects pre/post delay, draw 
void updateDelayLoop(){
  bool set_delay_value = false;
  // get latest EEPROM values..
  readEEPROM();
  if(in_pre_delay_menu){
    delay_val = delay_time_before_sunrise;
  } else if(in_post_delay_menu){
    delay_val = delay_time_after_sunset;
  }
  drawDelaySetMenu();
  while(!set_delay_value){
    wdt_reset();
    updatedelay_valLoop();
    menuButton.read();
    if(menuButton.wasReleased()){
      tone(PIN_PIEZO, 100 + delay_val * 12, 100);
      delay(200);
      tone(PIN_PIEZO, 100 + delay_val * 12, 100);
      set_delay_value = true;
      bool sunrise = in_pre_delay_menu;
      setdelay_valEEPROM(sunrise, delay_val);
    }
  }
}

// read encoder and update time delta value on LCD
char updatetime_valLoop(char prev){
  encState eState = encoder.read();
  if (eState == ENC_DEC) {
    prev--;
    tone(PIN_PIEZO, 300 + prev * 12, 25);
    drawAdjustTime(prev);
  } else if (eState == ENC_INC) {
    prev++;
    tone(PIN_PIEZO, 300 + prev * 12, 25);
    drawAdjustTime(prev);
  }
  return prev;
}

inline void adjustTimeMins(char mins){
  DateTime adjusted(rtc.now() + TimeSpan(0, 0, mins, 0));
  rtc.adjust(adjusted);
}

// adjusting the time
void adjustTimeLoop(){
  now = rtc.now();
  drawAdjustTime(0);
  bool set_time_dt_val = false;
  char deltaTimeMins = 0;
  while(!set_time_dt_val){
    wdt_reset();
    deltaTimeMins = updatetime_valLoop(deltaTimeMins);
    menuButton.read();
    if(menuButton.wasReleased()){
      tone(PIN_PIEZO, 300 + deltaTimeMins * 12, 100);
      delay(200);
      tone(PIN_PIEZO, 300 + deltaTimeMins * 12, 100);
      adjustTimeMins(deltaTimeMins);
      set_time_dt_val = true;      
    }
  }
  
}

// draw time adjust screen on LCD
inline void drawAdjustTime(char delta){
  lcd.clear();
  tensDigitLCD(now.hour());
  lcd.print(":");
  tensDigitLCD(now.minute());
  lcd.setCursor(0, 1);
  if (delta > 0){
    lcd.print("+");
  }
  lcd.print((int)delta);
  lcd.print(" mins.");
}

// pre/post selection menu 1 - before sunrise
inline void drawPreDelayMenu(){
  lcd.clear();
  lcd.print("Set delay before");
  lcd.setCursor(0, 1);
  lcd.print("Sunrise. [1/2]");
  in_pre_delay_menu = true;
  in_post_delay_menu = false;
}

// pre/post selection menu - after sunset
inline void drawPostDelayMenu(){
  lcd.clear();
  lcd.print("Set delay after");
  lcd.setCursor(0, 1);
  lcd.print("sunset.  [2/2]");
  in_pre_delay_menu = false;
  in_post_delay_menu = true;
}

// time adjust menu option
inline void drawTimeOption(){
  lcd.clear();
  lcd.print("Adjust time by");
  lcd.setCursor(0, 1);
  lcd.print("mins [1/2]");
  in_time_adj_menu = true;
  in_delay_adj_menu = false;
}

// delay adjust menu option
inline void drawDelayOption(){
  lcd.clear();
  lcd.print("Set pre/post");
  lcd.setCursor(0, 1);
  lcd.print("delays [2/2]");
  in_delay_adj_menu = true;
  in_time_adj_menu = false;
}

// cycling through menus (delay / time / pre/post delay)
void drawSelectedMenu(){
  lcd.setCursor(0, 0);
  if(menu_pos == 0){
    if(!in_main_menu){
      drawPreDelayMenu();
      //in_pre_delay_menu = true;
    } else {
      drawTimeOption();
    }
  } else if (menu_pos == 1){
    if(!in_main_menu){
      drawPostDelayMenu();
    } else {
      drawDelayOption();
    }
  }
}

// menu updater. On encoder pos change, draw new menu
void updateSetupMenu(){
  lcd.setCursor(0, 0);
  encState eState = encoder.read();
  if (eState == ENC_DEC) {
    tone(PIN_PIEZO, 400, 25);
    if (menu_pos == 0) {
      menu_pos = MENU_COUNT - 1;
    } else {
      menu_pos = (menu_pos - 1);
    }
    drawSelectedMenu();
    //DEBUG("Menu item: ");
    //DEBUGLN(menu_pos);
  } else if (eState == ENC_INC) {
    tone(PIN_PIEZO, 400, 25);
    menu_pos = (menu_pos + 1) % (MENU_COUNT);
    //DEBUG("Menu item: ");
    //DEBUGLN(menu_pos);
    drawSelectedMenu();
  }
}

// check if main menu button was pressed. Then run menu routine.
void checkIfButtonPressed(){
  menuButton.read();
  if(menuButton.wasReleased()){
    tone(PIN_PIEZO, 1000, 50);
    // if in main menu (date/time) - not in options yet
    if(!in_options_menu){
      in_options_menu = true;
      in_main_menu = true;
      // show options menu
      lcd.clear();
      // draw first option of options menu - adjust time
      drawTimeOption();
      
    } else if(in_delay_adj_menu){ /* user chose delay adjustion - draw first menu, set pre-delay */
      in_main_menu = false;
      in_delay_adj_menu = false;
      DEBUGLN("Entering delay adjust options menu.");
      drawPreDelayMenu();
    } else if (in_time_adj_menu){ /* user chose time adjustion - run time adjust loop */
      in_main_menu = false;
      in_time_adj_menu = false;
      DEBUGLN("Entering time adjust options menu.");
      adjustTimeLoop();
      in_options_menu = false;
    } else if (in_pre_delay_menu || in_post_delay_menu){ /* user chose pre/post delay - run delay adjust loop */
      DEBUGLN("drawing menu screen to update delay value");
      
      updateDelayLoop();
      in_options_menu = false;
    } 
  }

}

void loop() {

  // check if menu button pressed - enter delay time settings menu
  checkIfButtonPressed();
  
  // if in a menu, update menu
  if(in_options_menu){
    updateSetupMenu();
  } else {
    // main loop - sleep for about 4s (maximum for timer?) then check time and update door
    if(f_timer==1) {
      f_timer = 0;
      DEBUGLN("Waking from sleep..");
      
      // get time from rtc and check if it's day time/night time
      checkTime();

      // check to see if limit switches have been hit when they're not supposed to - could be user intervention
      checkLimitSwitches();
      
      // open/close door if it needs to do so
      updateDoor();

      // show time, sunrise or sunset times
      updateLCD();
      
      /* Re-enter sleep mode. */
      enterSleep();
    }
  }
  wdt_reset();
} 
