#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#include <LiquidCrystal.h>
#include <Wire.h>
#include <Button.h>
#include <TimeLord.h>
#include "RTClib.h"
#include <EEPROM.h>
#include "encoder.h"

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

#define PZ_VOL 254

Button menuButton(PIN_ENC_BTN, PULLUP, INVERT, DEBOUNCE_MS);

#define MENU_COUNT 2

volatile int f_timer=0;

//const int DELAYTIME_SEC = 20;
const int TIMEOUT_MAX = 220;
const int DELAY_MS = 10;
const int MAX_DELAY_MINUTES = 100;
int menuPos = 0;
int delayVal = 0;

RTC_DS1307 rtc;

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

DateTime now;

TimeLord tardis;

Encoder encoder(PIN_ENC_A, PIN_ENC_B, PIN_ENC_BTN);

float const LATITUDE = -42.9166667;
float const LONGITUDE = 147.3333282;

byte today[6];
byte sunrise[6];
byte sunset[6];

// process logic booleans
bool isDayTime = false;
bool doorOpen = false;
bool inSetupMenu = false;
bool inPreDelayMenu = false;
bool inPostDelayMenu = false;
bool setDelayValue = false;
bool hitBottom = false;
bool hitTop = false;

unsigned int delay_time_before_sunrise;
unsigned int delay_time_after_sunset;


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

  int eepromSuccess = 0x00;
  
  //int precalc_sunrise_c = sunrise_dt_c;

  // if checksum mismatch - something went wrong with EEPROM writing. Try to reset 
  if(sunrise_dt_c_calc != sunrise_dt_c){
    lcd.clear();
    lcd.print("EEPROM checksum");
    lcd.setCursor(0, 1);
    lcd.print("error :(");
    Serial.println("EEPROM checksum error - Writing defeault values for sunrise delay.");
    EEPROM.write(SUNRISE_DT_0_P, SUNRISE_DT_0_V);
    EEPROM.write(SUNRISE_DT_1_P, SUNRISE_DT_1_V);
    EEPROM.write(SUNRISE_DT_C_P, SUNRISE_DT_C_V);
    Serial.println("Done.");
    delay_time_before_sunrise = (SUNRISE_DT_0_V << 4) + SUNRISE_DT_1_V;
  } else {
    delay_time_before_sunrise = (sunrise_dt_v0 << 4) + sunrise_dt_v1;
    eepromSuccess += 0x01;
  }

  // check checksum of sunset time from EEPROM
  byte sunset_dt_c_calc = CKSUM_SECRET ^ sunset_dt_v0;
  sunset_dt_c_calc ^= sunset_dt_v1;

  //int precalc_sunset_c = (int)SUNSET_DT_C_V;

  if(sunset_dt_c_calc != sunset_dt_c){
    lcd.clear();
    lcd.print("EEPROM checksum");
    lcd.setCursor(0, 1);
    lcd.print("error :(");
    Serial.println("Writing default values for sunset delay.");
    EEPROM.write(SUNSET_DT_0_P, SUNSET_DT_0_V);
    EEPROM.write(SUNSET_DT_1_P, SUNSET_DT_1_V);
    EEPROM.write(SUNSET_DT_C_P, SUNSET_DT_C_V);
    Serial.println("Done.");
    delay_time_after_sunset = (SUNSET_DT_0_V << 4) + SUNSET_DT_1_V;
  } else {
    delay_time_after_sunset = (sunset_dt_v0 << 4) + sunset_dt_v1;
    eepromSuccess += 0x10;
  }

  if(eepromSuccess == 0x11){
    Serial.println("EEPROM read successfully.");
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

// LCD splash (Shouldn't be seen too much hopefully..)
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

  // setup LCD
  lcd.begin(16, 2);

  // setup RTC
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  
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

void openDoor(){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Opening door");
  Serial.println("Opening the door..");
  // send high to door open motor pin
  digitalWrite(PIN_DOOR_UP, HIGH);

  int timeoutcount = 0;

  // loop until limit switch is hit (also add timeout)
  Serial.print("Waiting for door to open");
  while(digitalRead(PIN_LIMSW_UP) == LOW){
    wdt_reset();
    delay(DELAY_MS);
    timeoutcount++;
    if(timeoutcount > TIMEOUT_MAX){
      Serial.print("\nERROR! door not closing properly. Check limit switch.\n");
      Serial.print("Pretending that it's closed for now..");
      break;
    }
    Serial.print(".");
  }
  hitTop = true;
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
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Closing door");
  int timeoutcount = 0;

  // loop until limit switch is hit  (also add timeout)
  Serial.print("Waiting for door to close");
  while(digitalRead(PIN_LIMSW_DN) == LOW){
    wdt_reset();
    delay(DELAY_MS);
    timeoutcount++;
    if(timeoutcount > TIMEOUT_MAX){
      Serial.print("\nERROR! door not opening properly. Check limit switch.\n");
      Serial.print("Pretending that it's open for now..");
      break;
    }
     Serial.print(".");
  }
  hitBottom = true;
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

// Alert user to press a button, otherwise the system will try to open/close the door to the proper position
void waitForUserResetOpen(){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Door will ");
  if(isDayTime && hitBottom){
    lcd.print("open");
  } else if(!isDayTime && hitTop){
    lcd.print("close");
  }
  lcd.setCursor(0, 1);
  lcd.print("Unless press btn");
  
  DateTime stopTime = rtc.now();
  DateTime fixTime;
  if(isDayTime && hitBottom){
    fixTime = stopTime + TimeSpan(0, 0, 0, 6);
  } else if(!isDayTime && hitTop){
    fixTime = stopTime + TimeSpan(0, 0, 0, 5);
  }
  
  bool pausedForUser = false;
  
  // while warning time not run out and user hasn't intervened
  while(fixTime.unixtime() - rtc.now().unixtime() > 0 && !pausedForUser){
    wdt_reset();
    menuButton.read();
    if(menuButton.wasReleased()){
      pausedForUser = true;
    }
    tone(PIN_PIEZO, 2000, 50);
    delay(50);
    tone(PIN_PIEZO, 2000, 50);
  }

  // if user wants to operate on door or something.. Leave door open/close (for a fair while)
  if(pausedForUser){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Keeping door ");
    lcd.setCursor(0, 1);
    if(isDayTime && hitBottom){
      lcd.print("open");
    } else if(!isDayTime && hitTop){
      lcd.print("close");
    }
    lcd.print("Press btn");

    // Calculate timeout - we can't pause indefinately just incase of user error!
    stopTime = rtc.now();
    bool timeOut = false;
    
    if(isDayTime && hitBottom){
      fixTime = stopTime + TimeSpan(0, 0, 50, 0);
    } else if(!isDayTime && hitTop){
      fixTime = stopTime + TimeSpan(0, 0, 30, 0);
    }

    // while waiting for user
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Waiting for user");
    lcd.setCursor(0, 1);
    lcd.print("intervention.");
    
    while(!timeOut && pausedForUser){
      wdt_reset();
      menuButton.read();
      if(menuButton.wasReleased()){
        pausedForUser = false;
      }
      if(fixTime.unixtime() - rtc.now().unixtime() > 0){
        timeOut = true;
      }
    }

    // If user took way too long, we should carry on! (also could be some real-world issue, eg something knocked limit switch?)
    if(timeOut){
      tone(PIN_PIEZO, 2000, 50);
      delay(50);
      tone(PIN_PIEZO, 2000, 50);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Sorry! Have to");
      lcd.setCursor(0, 1);
      if(isDayTime && hitBottom){
        lcd.print("open");
      } else if(!isDayTime && hitTop){
        lcd.print("close");
      }
      lcd.print(" Door.");
    } else {
      lcd.clear();
      lcd.setCursor(0, 0);
      if(isDayTime && hitBottom){
        lcd.print("Opening");
      } else if(!isDayTime && hitTop){
        lcd.print("Closing");
      }
      lcd.print(" door");
    }
  }
  // finally, open or close the door
  if(isDayTime && hitBottom){
    openDoor();
    hitBottom = false;
  } else if(!isDayTime && hitTop){
    closeDoor();
    hitTop = false;
  }
  
}

// check limit switches - if door is open when it should be closed
void checkLimitSwitches(){
  // If daytime, only check if door has been manually or accidentally closed
  if(isDayTime){
    Serial.print("Checking if door closed at daytime.");
    // quickly power motor to check if door is hitting wrong limit switch
    // if down switch pressed and day time
    digitalWrite(PIN_DOOR_DN, HIGH);
    if(digitalRead(PIN_LIMSW_DN) == HIGH){
      Serial.print("Hit bottom!");
      hitBottom = true;
    }
    delay(1);
    digitalWrite(PIN_DOOR_DN, LOW);
    digitalWrite(PIN_DOOR_UP, HIGH);
    delay(2);
    digitalWrite(PIN_DOOR_UP, LOW);
    if(hitBottom){
      waitForUserResetOpen();
    }
  } else {
    Serial.print("Checking if door opened at night time.");
    
    // if up switch is pressed at day time
    digitalWrite(PIN_DOOR_UP, HIGH);
    if(digitalRead(PIN_LIMSW_UP) == LOW){
      hitTop = true;
      Serial.print("Hit bottom!");
    }
    delay(1);
    digitalWrite(PIN_DOOR_UP, LOW);
    digitalWrite(PIN_DOOR_DN, HIGH);
    delay(2);
    digitalWrite(PIN_DOOR_DN, LOW);
    if(hitTop){
      waitForUserResetOpen();
    }
  }
}

// Get time from RTC and calculate sunrise/sunset to display on LCD
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

// update delay on lcd screen
void drawDelaySetMenu(){
  lcd.clear();
  lcd.setCursor(0, 0);
  if(inPreDelayMenu){
    lcd.print("Pre: ");
  } else if(inPostDelayMenu){
    lcd.print("Post: ");
  }
  lcd.print(delayVal);
  lcd.print(" mins");
}

// update delay value loop - check encoder, if change, update lcd with new val
void updateDelayValLoop(){
  encState eState = encoder.read();
  if (eState == ENC_DEC) {
    if (delayVal == 0) {
      delayVal = MAX_DELAY_MINUTES - 1;
    } else {
      delayVal = (delayVal - 1);
    }
    tone(PIN_PIEZO, 100 + delayVal * 12, 25);
    drawDelaySetMenu();
  } else if (eState == ENC_INC) {
    delayVal = (delayVal + 1) % (MAX_DELAY_MINUTES);
    tone(PIN_PIEZO, 100 + delayVal * 12, 25);
    drawDelaySetMenu();
  }
}

// write new delay value to EEPROM (including checksum)
void setDelayValEEPROM(bool sunrise, int val){
  int val_l = (val & 0b11110000) >> 4;
  int val_r = val & 0b00001111;
  int checksum = CKSUM_SECRET ^ val_l;
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

// When button clicked, change delay value
void updateDelaySettings(){
  // get latest EEPROM values..
  readEEPROM();
  if(inPreDelayMenu){
    delayVal = delay_time_before_sunrise;
  } else if(inPostDelayMenu){
    delayVal = delay_time_after_sunset;
  }
  drawDelaySetMenu();
  while(!setDelayValue){
    wdt_reset();
    updateDelayValLoop();
    menuButton.read();
    if(menuButton.wasReleased()){
      tone(PIN_PIEZO, 100 + delayVal * 12, 100);
      delay(200);
      tone(PIN_PIEZO, 100 + delayVal * 12, 100);
      // TODO: set delay value in EEPROM
      setDelayValue = true;
      bool sunrise = inPreDelayMenu;
      setDelayValEEPROM(sunrise, delayVal);
    }
  }
}

// For drawing main menu
void drawPreDelayMenu(){
  lcd.clear();
  lcd.print("Set delay before");
  lcd.setCursor(0, 1);
  lcd.print("Sunrise. [1/2]");
  inPreDelayMenu = true;
  inPostDelayMenu = false;
}

void drawPostDelayMenu(){
  lcd.clear();
  lcd.print("Set delay after");
  lcd.setCursor(0, 1);
  lcd.print("sunset.  [2/2]");
  inPreDelayMenu = false;
  inPostDelayMenu = true;
}

void drawSelectedMenu(){
  lcd.setCursor(0, 0);
  if(menuPos == 0){
    drawPreDelayMenu();
    inPreDelayMenu = true;
  } else if (menuPos == 1){
    drawPostDelayMenu();
    inPostDelayMenu = true;
  }
}

// menu updater. On encoder pos change, draw new menu
void updateSetupMenu(){
  lcd.setCursor(0, 0);
  encState eState = encoder.read();
  if (eState == ENC_DEC) {
    tone(PIN_PIEZO, 400, 25);
    if (menuPos == 0) {
      menuPos = MENU_COUNT - 1;
    } else {
      menuPos = (menuPos - 1);
    }
    drawSelectedMenu();
    Serial.print("Menu item: ");
    Serial.println(menuPos);
  } else if (eState == ENC_INC) {
    tone(PIN_PIEZO, 400, 25);
    menuPos = (menuPos + 1) % (MENU_COUNT);
    Serial.print("Menu item: ");
    Serial.println(menuPos);
    drawSelectedMenu();
  }
}

// check if main menu button was pressed. Then run menu routine.
void checkIfButtonPressed(){
  menuButton.read();
  if(menuButton.wasReleased()){
    tone(PIN_PIEZO, 1000, 50);
    // if in main menu (date/time)
    if(!inSetupMenu){
      lcd.clear();
      inSetupMenu = true;
      inPreDelayMenu = true;
      inPostDelayMenu = false;
      drawPreDelayMenu();
    } else if (inPreDelayMenu || inPostDelayMenu){
      Serial.println("drawing menu screen to update delay value");
      setDelayValue = false;
      updateDelaySettings();
      inSetupMenu = false;
    }
  }

}

void loop() {

  // check if menu button pressed - enter delay time settings menu
  checkIfButtonPressed();
  
  // if in menu, update menu
  if(inSetupMenu){
    updateSetupMenu();
  } else {
    // main loop - sleep for about 4s (maximum for timer?) then check time and update door
    if(f_timer==1) {
      f_timer = 0;
      Serial.println("Waking from sleep..");
      
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
