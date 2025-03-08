#include <Arduino.h>
#include "circular_buffer.h"
#include <avr/sleep.h>

volatile circular_buffer timer4_buffer;
volatile circular_buffer timer3_buffer;
volatile circular_buffer timer5_buffer;

void init_timers()
{
  //set timer1 interrupt at 1Hz
  TCCR4A = 0;// set entire TCCR1A register to 0
  TCCR4B = 0;// same for TCCR1B
  TCNT4  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR4A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR4B |= (1 << WGM42);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR4B |= (1 << CS42) | (1 << CS40);  
  // enable timer compare interrupt
  TIMSK4|= (1 << OCIE4A);

  //set timer3 interrupt at 50Hz
  TCCR3A = 0;// set entire TCCR1A register to 0
  TCCR3B = 0;// same for TCCR1B
  TCNT3  = 0;//initialize counter value to 0
  // set compare match register for 50hz increments
  OCR3A = 311;// = ((16*10^6) / (1*1024)) / 50 - 1 (must be <65536)
  // turn on CTC mode
  TCCR3B |= (1 << WGM32);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR3B |= (1 << CS32) | (1 << CS30);  
  // enable timer compare interrupt
  TIMSK3 |= (1 << OCIE3A);

  //set timer5 interrupt at 100Hz
  TCCR5A = 0;// set entire TCCR1A register to 0
  TCCR5B = 0;// same for TCCR1B
  TCNT5  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR5A = 155;// = ((16*10^6) / (1*1024)) / 100 - 1 (must be <65536)
  // turn on CTC mode
  TCCR5B |= (1 << WGM52);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR5B |= (1 << CS52) | (1 << CS50);  
  // enable timer compare interrupt
  TIMSK5 |= (1 << OCIE5A);
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

void atmega_sleep()
{
  sleep_enable();
  sleep_cpu();
  sleep_disable();
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
  // put your setup code here, to run once:
  sei();//allow interrupts
  init_timers();
  init_idle();
  Serial.begin(115200);
}

void loop() {
  static int val;
  if ((val = pop(&timer3_buffer)) >= 0)
  {
      task2();
  }
  if ((val = pop(&timer5_buffer)) >= 0)
  {
      task3();
  }
  if ((val = pop(&timer4_buffer)) >= 0)
  {
      task1();
  }
  atmega_sleep();
}


ISR(TIMER1_COMPA_vect)
{
 push(1, &timer4_buffer);
}

ISR(TIMER3_COMPA_vect)
{
 push(1, &timer3_buffer);
}

ISR(TIMER5_COMPA_vect)
{
 push(1, &timer5_buffer);
}