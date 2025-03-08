#include <Arduino.h>
#include "IRremote.h"
#include <avr/sleep.h>

#define TASK1 0
#define TASK2 1
#define TASK3 2
#define TASK4 3
#define NUM_STATES 4

#define SMCR_POWER_SAVE (1 << SM1) | (1 << SM0)

#define PROX_INPUT  7

bool timer_ready = false;
int state = TASK1;

int receiver = 11; // Signal Pin of IR receiver to Arduino Digital Pin 11

IRrecv irrecv(receiver);     // create instance of 'irrecv'
decode_results results;      // create instance of 'decode_results'
unsigned long key_value = 0;

void init_timer()
{
//set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
}

void init_power_save()
{
  SMCR |= SMCR_POWER_SAVE;
  SMCR |= (1 << SE);
}

void translateIR() // takes action based on IR code received

// describing Remote IR codes 
{

  // if (results.value == 0XFFFFFFFF)
  // results.value = key_value;
  switch(results.value)

  {
  case 0xFFA25D: Serial.println("POWER"); break;
  case 0xFFE21D: Serial.println("FUNC/STOP"); break;
  case 0xFF629D: Serial.println("VOL+"); break;
  case 0xFF22DD: Serial.println("FAST BACK");    break;
  case 0xFF02FD: Serial.println("PAUSE");    break;
  case 0xFFC23D: Serial.println("FAST FORWARD");   break;
  case 0xFFE01F: Serial.println("DOWN");    break;
  case 0xFFA857: Serial.println("VOL-");    break;
  case 0xFF906F: Serial.println("UP");    break;
  case 0xFF9867: Serial.println("EQ");    break;
  case 0xFFB04F: Serial.println("ST/REPT");    break;
  case 0xFF6897: Serial.println("0");    break;
  case 0xFF30CF: Serial.println("1");    break;
  case 0xFF18E7: Serial.println("2");    break;
  case 0xFF7A85: Serial.println("3");    break;
  case 0xFF10EF: Serial.println("4");    break;
  case 0xFF38C7: Serial.println("5");    break;
  case 0xFF5AA5: Serial.println("6");    break;
  case 0xFF42BD: Serial.println("7");    break;
  case 0xFF4AB5: Serial.println("8");    break;
  case 0xFF52AD: Serial.println("9");    break;
  case 0xFFFFFFFF: Serial.println(" REPEAT");break;  

  default: 
    Serial.println(" other button   ");

  }// End Case
        key_value = results.value;
  delay(500); // Do not get immediate repeat


} //END translateIR

void setup() {


  pinMode(7, INPUT);

  irrecv.enableIRIn(); // Start the receiver


  sei();//allow interrupts
  init_timer();
  init_power_save();
  Serial.begin(115200);
}


void task1_func()
{
  if (irrecv.decode(&results)) // have we received an IR signal?
  {
    irrecv.resume(); // receive the next value
    translateIR(); 
  }  
}

void task2_func()
{
  int sensorValue = digitalRead(7);
  Serial.println(sensorValue);
}

void task3_func()
{


}

  int count = 0;

void loop() {

  if(timer_ready)
  {
    Serial.print("State = ");
    Serial.println(state);
    switch(state)
    {
      case TASK1: 
      Serial.println("task1");
      task1_func();
      break;
      case TASK2:
      Serial.println("task2");
      task2_func();
      break;
      case TASK3:
      task1_func();
      Serial.println("task1 again");
      break;
      case TASK4: break;
      default: break;
    }
    state++;
    if(state%(NUM_STATES-1) == 0) state = TASK1;
    timer_ready = false;
    count++;
    if(count == 5) 
    {
      Serial.println("timer went off");
      count = 0;
    }
  }
  // sleep_enable();
  // sleep_cpu();
  // sleep_disable();
}

// timer1 overflow
ISR(TIMER1_COMPA_vect) {
// process the timer1 overflow here
  timer_ready = true;
}

