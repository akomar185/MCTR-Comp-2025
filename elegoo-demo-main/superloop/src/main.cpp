#include <Arduino.h>
#include "IRremote.h"
#include <avr/sleep.h>


void init_timer()
{
//set timer2 interrupt at 61Hz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 255;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);  
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
}

void init_power_save()
{
  SMCR |= (1 << SM1) | (1 << SM0);
  SMCR |= (1 << SE);
}

void init_idle()
{
  SMCR &= ~(1 << SM2) & ~(1 << SM1) & ~(1 << SM0);
  SMCR |= (1 << SE);
}

void task1()
{
  Serial.println("Task 1");

}

void task2()
{
  Serial.println("Task 2");
}

void task3()
{
  Serial.println("Task 3");
}

void setup() {
  sei();//allow interrupts
  init_timer();
  init_power_save();
  Serial.begin(115200);
}

void atmega_sleep()
{
  sleep_enable();
  sleep_cpu();
  sleep_disable();
}

void loop() {

  task1();
  task2();
  task3();
  atmega_sleep();
}

// // timer2 overflow
// ISR(TIMER2_COMPA_vect) {
// // process the timer1 overflow here
// }

