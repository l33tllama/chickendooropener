#ifndef MAIN_H_
#define MAIN_H_

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

#ifdef DEBUG_EN
  #define DEBUG(c) Serial.print(c);
  #define DEBUGLN(c) Serial.println(c);
#else
  #define DEBUG(c) ;
  #define DEBUGLN(c) ;
#endif


#ifdef ERR_EN
  #define ERR(c) Serial.print(c);
  #define ERRLN(c) Serial.println(c);
#else
  #define ERR(c) ;
  #define ERRLN(c) ;
#endif

#endif
