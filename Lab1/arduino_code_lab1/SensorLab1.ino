/**
 * Stub for Lab1 code, Sensors and Sensing at Orebro University
 * 2018
 * 
 * The purpose of this lab is to implement a velocity PID controller
 * for a DC motor with an onboard encoder, connected to the MakeBlock 
 * MegaPi board. Please note: the MegaPi libraries already implement 
 * many of the functionalities needed for this lab. You are only 
 * allowed to use the following library calls:
 *    1. uint8_t MeEncoderOnBoard::getPortB(void);      //get second channel port number
 *    2. uint8_t MeEncoderOnBoard::getIntNum(void);     //get interrupt port number
 *    3. void MeEncoderOnBoard::setMotorPwm(int pwm);   //set a desired control to the motor
 * 
 * 
 * This sketch is based on the example under 
 * Me_On_Board_encoder>Me_MegaPi_encoder_pwm.
 * 
 * This code is licensed under the BSD license.
 * Author list:
 *  - Todor Stoyanov
 */

#include <MeMegaPi.h>

#include <ros.h>
#include <makeblock_odom/MotorStates.h>
#include <makeblock_odom/SetPID.h>
#include <makeblock_odom/SetVel.h>
#include <makeblock_odom/ResetOdom.h>

//////////////// struct definitions ///////////////////
/** structure to hold PID parameters */
  
struct PIDParameters {
  float Kp_;
  float Kd_;
  float Ki_;
  float u_max_; //Maximum controller output (<= max PWM)
  float u_min_; //Minimum controller output [>= -(max PWM)]
  float I_;     //Serves as memory for the integral term [i.e., I=dT*(Ki*e_0, ... , Ki*e_t)]

  PIDParameters(float Kp, float Ki, float Kd, float u_max, float u_min, float I) : 
  Kp_(Kp), Kd_(Kd), Ki_(Ki), u_max_(u_max), u_min_(u_min), I_(I) {};
};

/** structure to hold values associated with a controlled motor */

struct MotorState {
  float r_;  //setpoint value at current iteration (in radians/sec)
  float e_;  //error at current iteration
  float de_; //error derivative at current iteration
  float u_;  //computed control

  MotorState(float r, float e, float de, float u): r_(r),e_(e),de_(de),u_(u) {};
};

/** structure to hold values for an encoder */

struct EncoderState {
  long ticks_;  //raw number of ticks
  float ratio_; //number of ticks per radian
  float pos_;   //position in radians
  float vel_;   //velocity in radians/sec
  EncoderState(long ticks, float ratio, float pos, float vel): ticks_(ticks),ratio_(ratio),pos_(pos),vel_(vel) {};
};

////////////// global variables ////////////////////
//the maximum value we can command the motor
const short MAX_PWM_MEGAPI = 255;
//values bellow this do not make the motor move, so we can clamp them
const short MIN_ACTUATION_PWM = 20;
//TODO: set correct value for ticks per radian 
const float ENC_TICKS = 368 / (2 * PI); 

//TODO: here setup encoders to the appropriate ports
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT4);

PIDParameters* pid_left_vc = new PIDParameters(1.0, 0.0, 0.0, MAX_PWM_MEGAPI, -MAX_PWM_MEGAPI, 0.0);  //Velocity controller PID parameters for Motor 0
PIDParameters* pid_right_vc = new PIDParameters(1.0, 0.0, 0.0, MAX_PWM_MEGAPI, -MAX_PWM_MEGAPI, 0.0); //Velocity controller PID parameters for Motor 1
  
MotorState* left_motor_state = new MotorState(0.0,0,0,0);   //accumulator for left motor
MotorState* right_motor_state = new MotorState(0.0,0,0,0);  //accumulator for right motor
  
EncoderState* left_encoder = new EncoderState(0,ENC_TICKS,0,0);
EncoderState* right_encoder = new EncoderState(0,ENC_TICKS,0,0);

//ids of the two motors
const short LEFT_MOTOR=0;
const short RIGHT_MOTOR=1;

//timers
unsigned long t_new, t_old, t_old_comm, t_old_serial;

const unsigned long dT = 10000L;                  //sampling time in microseconds
const unsigned long dT_serial = 50000L;           //sampling time for output in microseconds
const unsigned long dT_velcmd = 500000L;          //timeout of velocity command 

//velocity filtering parameter
float alpha = 0.05;

///////////////////////////////////////////////////////////////////////////////////////////////
/////////////////ROS Callbacks//////////////////////////
//callback function for setting new PID parameters
void setPIDCallback(const makeblock_odom::SetPID::Request &req, makeblock_odom::SetPID::Response &res);
void setVelCallback(const makeblock_odom::SetVel::Request &req, makeblock_odom::SetVel::Response &res);

///////////////////ROS global variables/////////////////////////

ros::NodeHandle nh;
makeblock_odom::MotorStates state;
ros::Publisher state_publisher("/motor_states", &state);
ros::ServiceServer<makeblock_odom::SetPID::Request, makeblock_odom::SetPID::Response> pid_server("set_pid", &setPIDCallback);
ros::ServiceServer<makeblock_odom::SetVel::Request, makeblock_odom::SetVel::Response> vel_server("set_vel", &setVelCallback);

////////////////////////////////////////////////////////////////////////////////
//////////////////////////// function declarations /////////////////////////////
////////////////////////////////////////////////////////////////////////////////
int clampPwm(int motor_pwm);
void updateEncoder(EncoderState * encoder);
float pid(float e, float de, PIDParameters* p);

//setup interupts and serial communication
void setup()
{
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  
  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  //setup timers
  t_old = micros();
  t_old_serial = micros();

  //setup state header
  state.header.stamp.nsec = 0;
  state.header.stamp.sec = 0;
  state.header.frame_id = "robot";  

  //Set set point for motors
  left_motor_state->r_ = 0;
  right_motor_state->r_ = 0;

  ////// ROS initializations////
  nh.initNode();
  nh.advertise(state_publisher);
  nh.advertiseService(pid_server);
  nh.advertiseService(vel_server);
}


/////////////////////////////////////////////////////////////////
/////////////////// Main function ///////////////////////////////
/////////////////////////////////////////////////////////////////

void loop()
{
  //spin and check if we should publish
  t_new = micros();
  nh.spinOnce();

  //Check if it's time to publish new MotorStates message.
  unsigned long serial_time_elapsed = timeElapsed(t_new, t_old_serial);
  if (serial_time_elapsed > dT_serial)
  { 
    //Update time stamp
    state.header.stamp.nsec += (t_new - t_old_serial) * 1000;
    if(state.header.stamp.nsec > 1000000000){
      state.header.stamp.sec++;
      state.header.stamp.nsec -= 1000000000;
    }
    
    //Update left motor
    state.left.ticks = left_encoder->ticks_;
    state.left.pos = left_encoder->pos_;
    state.left.vel = left_encoder->vel_;
    state.left.pid.error = left_motor_state->e_;
    state.left.pid.setpoint = left_motor_state->r_;
    state.left.pid.kp = pid_left_vc->Kp_;
    state.left.pid.kd = pid_left_vc->Kd_;
    state.left.pid.ki = pid_left_vc->Ki_;
    
    //Update right motor
    state.right.ticks = right_encoder->ticks_;
    state.right.pos = right_encoder->pos_;
    state.right.vel = right_encoder->vel_;
    state.right.pid.error = right_motor_state->e_;
    state.right.pid.setpoint = right_motor_state->r_;
    state.right.pid.kp = pid_right_vc->Kp_;
    state.right.pid.kd = pid_right_vc->Kd_;
    state.right.pid.ki = pid_right_vc->Ki_;

    //Publish state
    state_publisher.publish(&state);
    t_old_serial = t_new;
  }
  
  //Do nothing if the sampling period didn't pass yet
  unsigned long time_elapsed = timeElapsed(t_new, t_old);
  if (time_elapsed < dT)
    return;
  
  //calculate new encoder velocity and position
  updateEncoder(left_encoder);
  updateEncoder(right_encoder);

  // calculate error and derivative
  float leftMotorError = left_motor_state->r_ - left_encoder->vel_;
  left_motor_state->de_ = (leftMotorError - left_motor_state->e_);
  left_motor_state->e_ = leftMotorError;

  float rightMotorError = right_motor_state->r_ - right_encoder->vel_;
  right_motor_state->de_ = (rightMotorError - right_motor_state->e_);
  right_motor_state->e_ = rightMotorError;
  
  // calculate control
  left_motor_state->u_ = pid(left_motor_state->e_, left_motor_state->de_, pid_left_vc);
  right_motor_state->u_ = pid(right_motor_state->e_, right_motor_state->de_, pid_right_vc);
  
  // if control is bellow minmum or above maximum command, clamp
  int left_motor_pwm = clampPwm(-(int)left_motor_state->u_);
  int right_motor_pwm = clampPwm((int)right_motor_state->u_);
    
  // actuate motors using setMotorPwm
  Encoder_1.setMotorPwm(left_motor_pwm);
  Encoder_2.setMotorPwm(right_motor_pwm);

  t_old = t_new;
}

unsigned long timeElapsed(unsigned long new_time, unsigned long old_time){
  unsigned long time_elapsed = new_time - old_time;
  
  return time_elapsed;
}

int clampPwm(int motor_pwm){
  if(motor_pwm < MIN_ACTUATION_PWM && motor_pwm > -MIN_ACTUATION_PWM)
    return 0;
  else if(motor_pwm > MAX_PWM_MEGAPI)
    return MAX_PWM_MEGAPI;
  else if(motor_pwm < -MAX_PWM_MEGAPI)
    return -MAX_PWM_MEGAPI;
  return motor_pwm;
}

void updateEncoder(EncoderState * encoder){
  float enc_pos, enc_vel;
 
  enc_pos = encoder->ticks_/encoder->ratio_;
  enc_vel = (float)(enc_pos - encoder->pos_)*1e6 / (float)dT;
  encoder->pos_ = enc_pos;
  
  //apply low-pass filter
  encoder->vel_ = (1-alpha)*encoder->vel_ + alpha*enc_vel;
}

//TODO: add and implement callbacks here.
void setPIDCallback(const makeblock_odom::SetPID::Request &req, makeblock_odom::SetPID::Response &res) {
  if(req.motor_id == LEFT_MOTOR){
    pid_left_vc->Kp_ = req.kp;
    pid_left_vc->Kd_ = req.kd;
    pid_left_vc->Ki_ = req.ki;
    res.success = true;
  }
  else if(req.motor_id == RIGHT_MOTOR){
    pid_right_vc->Kp_ = req.kp;
    pid_right_vc->Kd_ = req.kd;
    pid_right_vc->Ki_ = req.ki;
    res.success = true;
  }
  else{
    res.success = false;
  }
}

void setVelCallback(const makeblock_odom::SetVel::Request &req, makeblock_odom::SetVel::Response &res){
  if(req.motor_id == LEFT_MOTOR){
    left_motor_state->r_ = req.vel;
    res.success = true;
  }
  else if(req.motor_id == RIGHT_MOTOR){
    right_motor_state->r_ = req.vel;
    res.success = true;
  }
  else{
    res.success = false;
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
///////////////// NOTE: REMAINING FUNCTIONS CAN BE COPPIED FROM THE PREVIOUS LAB ////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

//Left motor
void isr_process_encoder1(void)
{
  if(digitalRead(Encoder_1.getPortB()) == 0)
  {
    left_encoder->ticks_++;
  }
  else
  {
    left_encoder->ticks_--;
  }
}

//Right motor
void isr_process_encoder2(void)
{
  if(digitalRead(Encoder_2.getPortB()) == 0)
  {
    right_encoder->ticks_--;
  }
  else
  {
    right_encoder->ticks_++;
  }
}

//--------------------------------------------------------------------------//
//   Here compute the PID control for a given error, de, and PID params     //
//--------------------------------------------------------------------------//

  float pid(float e, float de, PIDParameters* p){
    //update the integral term
    p->I_ += e;  
    
    //compute the control value
    float samplingTime = ((float)timeElapsed(t_new, t_old)) / 1000000.0;
    float control_value = p->Kp_ * e + p->Ki_ * p->I_ + p->Kd_ * de / samplingTime;
    
    //clamp the control value and back-calculate the integral term (the latter to avoid windup)
    if(control_value >= p->u_max_){
      control_value = p->u_max_;
      p->I_ -= e;
    }
    else if(control_value <= p->u_min_){
      control_value = p->u_min_;
      p->I_ -= e;
    }
    else if(control_value < MIN_ACTUATION_PWM && control_value > -MIN_ACTUATION_PWM){
      control_value = 0.0;
      p->I_ = 0.0;
    }
    
    return control_value;
  }
