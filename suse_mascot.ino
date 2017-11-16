#include <IRremote.h>
#include <IRremoteInt.h>
#include <ilTimer.h>
#include <Servo.h>

#define hip_correction    6
#define hip_neutral       90
#define hip_right         00
#define hip_left          180
#define Hip(pos)          hip.write(pos+hip_correction)

#define head_correction   0
#define head_neutral      90
#define head_left         50
#define head_right        130
#define Head(pos)         head.write(pos+head_correction)

#define leg_l_correction  0
#define leg_l_back        30
#define leg_l_neutral     90
#define leg_l_ahead       140
#define Leg_l(pos)        leg_l.write(pos+leg_l_correction)
   
#define leg_r_correction  0
#define leg_r_back        110
#define leg_r_neutral     90
#define leg_r_ahead       30
#define Leg_r(pos)        leg_r.write(pos+leg_r_correction) 

#define tail_neutral      90
#define tail_left         40
#define tail_right        140
#define tail_happy_left   0
#define tail_happy_right  180
#define Tail(pos)         tail.write(pos) 

#define state_stop        0
#define state_walking     1
#define state_happy       2
#define state_moveHead    3
#define state_happy2      4
#define state_crazy        5

#define IR_pin           2
#define IR_up            0xFF629D
#define IR_ok            0xFF02FD
#define IR_1             0xFF6897
#define IR_2             0xFF9867
#define IR_3             0xFFB04F
#define IR_4             0xFF30CF
#define IR_5             0xFF18E7
#define IR_6             0xFF7A85
#define IR_7             0xFF10EF
#define IR_8             0xFF38C7
#define IR_9             0xFF5AA5

Servo leg_l;
Servo leg_r;
Servo hip;
Servo head;
Servo tail;

int state_step = 0;
int state = state_stop;

IRrecv irrecv(IR_pin); 
decode_results results;

void setup() {
  leg_l.attach(9); 
  leg_r.attach(10);
  hip.attach(11);
  head.attach(6);
  tail.attach(5);

  irrecv.enableIRIn(); // Start the receiver
  irrecv.blink13(true);

  Serial.begin(9600);

  //stop();
  walk();
  //happy();
  //moveHead();
  //happy2();
  //state_crazy();

  randomSeed(analogRead(0));
}

void neutral(){
  Head(head_neutral);
  Hip(hip_neutral);
  Leg_l(leg_l_neutral);
  Leg_r(leg_r_neutral);
  Tail(tail_neutral);
}


void stop(){
  if (state ==  state_stop)
    return;
    
  state =  state_stop;

  neutral();
}

void walk(){
  if (state ==  state_walking)
    return;
    
  state =  state_walking;
  state_step=0;
  walk_seq();
}

void happy(){
  if (state ==  state_happy)
    return;
    
  state =  state_happy;
  state_step=0;
  neutral();
  happy_seq();
}

void moveHead(){
  if (state ==  state_moveHead)
    return;
    
  state =  state_moveHead;
  state_step=0;
  neutral();
  moveHead_seq();
}

void happy2(){
  if (state ==  state_happy2)
    return;
    
  state =  state_happy2;
  state_step=0;
  neutral();
  happy2_seq();
}

void crazy(){
  if (state ==  state_crazy)
    return;
    
  state =  state_crazy;
  state_step=0;
  neutral();
  crazy();
}


void moveHead_seq(){
  int delayTimeout = 300;
  
  if (state != state_moveHead)
    return;

  switch (state_step){
    // LEFT step
    // move the hip and head to stabilize
    case 0:
      Head(head_left);
    break;
    case 1:
      Head(head_right);
    break;
  }

  state_step = (state_step + 1) % 2;
  
  ilTimerControl.timeout(moveHead_seq, delayTimeout);
}

void happy_seq(){
  int delayTimeout = 300;
  
  if (state != state_happy)
    return;

  switch (state_step){
    // LEFT step
    // move the hip and head to stabilize
    case 0:
      Tail(tail_happy_left);
    break;
    case 1:
      Tail(tail_happy_right);
    break;
  }

  state_step = (state_step + 1) % 2;
  
  ilTimerControl.timeout(happy_seq, delayTimeout);
}


void happy2_seq(){
  int delayTimeout = 250;
  
  if (state != state_happy2)
    return;

  switch (state_step){
    // LEFT step
    // move the hip and head to stabilize
    case 0:
      Tail(tail_happy_right);
      Head(head_left);
    break;
    case 1:
      Tail(tail_happy_left);
      Head(head_right);
    break;
  }

  state_step = (state_step + 1) % 2;
  
  ilTimerControl.timeout(happy2_seq, delayTimeout);
}

void crazy_seq(){
  int delayTimeout = 500;
  int p;
  
  if (state != state_crazy)
    return;

  p = random(30, 150);
  Leg_l(p);
  p = random(30, 150);
  Leg_r(p);
  p = random(30, 150);
  Hip(p);
  p = random(30, 150);
  Head(p);
  p = random(30, 150);
  Tail(p);

  //Serial.println(p);
  
  ilTimerControl.timeout(state_crazy, delayTimeout);
}

/**
 * Walk sequence
 */
void walk_seq(){
  int delayTimeout = 100;
  
  if (state != state_walking)
    return;
  
  switch (state_step){
    // LEFT step
    // move the hip and head to stabilize
    case 0:
      Head(head_left);
      Hip(hip_right);
      Tail(tail_left);
    break;

    // up the left leg
    case 1:
      Leg_l(leg_l_ahead);
    break;

    // down the hip
    case 2:
      Hip(hip_neutral);
    break;

    case 3:
      Leg_l(leg_l_neutral);
    break;

    // RIGHT step
    // move the hip and head to stabilize
    case 4:
      Head(head_right);
      Hip(hip_left);
      Tail(tail_right);
    break;

    // up the left leg
    case 5:
      Leg_r(leg_r_ahead);
    break;

    // down the hip
    case 6:
      Hip(hip_neutral);
    break;

    case 7:
      Leg_r(leg_r_neutral);
    break;
  }

  state_step = (state_step + 1) % 8;
  
  ilTimerControl.timeout(walk_seq, delayTimeout);
}


void loop() {
  ilTimerControl.check();

  if (irrecv.decode(&results)) {
    if (results.decode_type == NEC) {
      //Serial.println(results.value,HEX);
      switch(results.value){
        case IR_up:
        case IR_5:
          walk();
          break;
        case IR_ok:
          stop();
          break;
        case IR_1:
          happy();
          break;
        case IR_2:
          moveHead();
          break;
        case IR_3:
          happy2();
          break;
        case IR_4:
          crazy();
          break;
      }
    }
    irrecv.resume();
  }


}

