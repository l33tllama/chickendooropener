#ifndef ENCODER_H_
#define ENCODER_H_

typedef enum EncoderState {
    ENC_SAME = 0x00, 
    ENC_DEC = 0x01,
    ENC_INC = 0x02
  } encState;
class Encoder{

private:

  int encoderPos;
  int lastEncoderPos;
  unsigned int encoderPinALast;
  unsigned int n;
  unsigned int _enc_a_pin;
  unsigned int _enc_b_pin;
  unsigned int _enc_btn_pin;

public:
  Encoder();

  Encoder(unsigned int enc_a_pin, unsigned int enc_b_pin, unsigned int enc_btn_pin);

  encState read();
  
};

#endif
