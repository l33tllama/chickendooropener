#ifndef ENCODER_H_
#define ENCODER_H_

typedef enum EncoderState {
    ENC_SAME = 0x00, 
    ENC_DEC = 0x01,
    ENC_INC = 0x02
  } encState;
class Encoder{

private:

  char encoderPos;
  char lastEncoderPos;
  char encoderPinALast;
  char n;
  char _enc_a_pin;
  char _enc_b_pin;
  char _enc_btn_pin;

public:
  Encoder();

  Encoder(char enc_a_pin, char enc_b_pin, char enc_btn_pin);

  encState read();
  
};

#endif
