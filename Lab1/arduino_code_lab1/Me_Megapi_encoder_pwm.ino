/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * @file    Me_Megapi_encoder_pwm.ino
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2016/07/14
 * @brief   Description: this file is sample code for Megapi encoder motor device.
 *
 * Function List:
 *    1. uint8_t MeEncoderOnBoard::getPortB(void);
 *    2. uint8_t MeEncoderOnBoard::getIntNum(void);
 *    3. void MeEncoderOnBoard::pulsePosPlus(void);
 *    4. void MeEncoderOnBoard::pulsePosMinus(void);
 *    5. void MeEncoderOnBoard::setMotorPwm(int pwm);
 *    6. double MeEncoderOnBoard::getCurrentSpeed(void);
 *    7. void MeEncoderOnBoard::loop(void);
 *    8. void MeEncoderOnBoard::setTarPWM(int16_t pwm_value);
 *
 * \par History:
 * <pre>
 * <Author>     <Time>        <Version>      <Descr>
 * Mark Yan     2016/07/14    1.0.0          build the new
 * </pre>
 */

#include <MeMegaPi.h>

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT4);

void isr_process_encoder1(void)
{
  if(digitalRead(Encoder_1.getPortB()) == 0)
  {
    Encoder_1.pulsePosMinus();
  }
  else
  {
    Encoder_1.pulsePosPlus();
  }
}

void isr_process_encoder2(void)
{
  if(digitalRead(Encoder_2.getPortB()) == 0)
  {
    Encoder_2.pulsePosMinus();
  }
  else
  {
    Encoder_2.pulsePosPlus();
  }
}

void setup()
{
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  Serial.begin(115200);
  
  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}

void loop()
{
  if(Serial.available())
  {
    char a = Serial.read();
    switch(a)
    {
      case '0':
      Encoder_1.setTarPWM(-20);
      Encoder_2.setTarPWM(20);
      break;
      case '1':
      Encoder_1.setTarPWM(-30);
      Encoder_2.setTarPWM(30);
      break;
      case '2':
      Encoder_1.setTarPWM(-40);
      Encoder_2.setTarPWM(40);
      break;
      case '3':
      Encoder_1.setTarPWM(-45);
      Encoder_2.setTarPWM(45);
      break;
      case '4':
      Encoder_1.setTarPWM(-50);
      Encoder_2.setTarPWM(50);
      break;
      case '5':
      Encoder_1.setTarPWM(-55);
      Encoder_2.setTarPWM(55);
      break;
      case '6':
      Encoder_1.setTarPWM(-60);
      Encoder_2.setTarPWM(60);
      break;
    }
  }
  
  Encoder_1.loop();
  Encoder_2.loop();
  Serial.print(Encoder_1.getCurPos());
  Serial.print(" ");
  Serial.print(Encoder_1.getCurrentSpeed());
  Serial.print(" ");
  Serial.print(Encoder_2.getCurPos());
  Serial.print(" ");
  Serial.println(Encoder_2.getCurrentSpeed());
}
