//
// Created by matt on 6/22/15.
//
#include "mbed.h"

int main()
{

  DigitalOut ledOne(LED1);
  DigitalOut ledTwo(LED2);

  while(true)
  {
    wait(0.1);
    ledOne = !ledOne;
    ledTwo = !ledOne;
  }
}