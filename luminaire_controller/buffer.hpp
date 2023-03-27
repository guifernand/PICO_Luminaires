#ifndef BUFFER_HPP
#define BUFFER_HPP

#include "Arduino.h"
#include <string>
#include <iostream>
#include <vector>
using namespace std;

class Buffer {
  private:
      //buffer size for last minute - minute/(sampling period) 
      static const int BUFFER_SIZE {6000};
      //head - next position to write to
      //n_stored - number of values stored     
      int head {0}, n_stored {0};

      //buffer for the illuminance measured values
      float sensed[BUFFER_SIZE] = { 0 };
      //buffer for the duty cycle values
      float duty_cicle[BUFFER_SIZE] = { 0 };
  
  public:
      Buffer(){}
      
      /*
       * Adds data to the buffers
       * inputs: sensed_ - illuminance measured value to store in buffer
       *         duty_cicle_ - duty cycle value to store in buffer
       */
      void addData(float sensed_, float duty_cicle_){    
          sensed[head] = sensed_;
          duty_cicle[head] = duty_cicle_;
          
          head = (head + 1)%BUFFER_SIZE;
  
          if (n_stored < 6000){
            n_stored++;
          }
          
      }
      
      /*
       * Read the data stored for the last minute before the call of the showData and store it in a string 
       * inputs: var - variable to print the last minute buffer of ('l' for illuminance/'d' for duty cycle)
       *         desk - id of the desk
       */
      String showData(char var, char desk){
          Serial.println(var);
          
          String lm_var = "b ";
          int temp_head = head;
          int tail{0};
          
          if (n_stored == BUFFER_SIZE){
            tail = (temp_head + 1) % BUFFER_SIZE;
          }else{
            tail = 0;
          }

          //store the buffers so that no data is written 
          //over by the controller interrupt
          float* sensed_print = sensed;
          float* duty_cicle_print = duty_cicle;
          
          if (var  == 'l'){
            while(tail != temp_head){
              //iterate over stored illuminance data a add to string
              lm_var = lm_var + "," + sensed_print[tail];
              tail = (tail + 1)%BUFFER_SIZE; 
            }
          }else if (var == 'd'){ 
            //iterate over stored duty cycle data a add to string
            while(tail != temp_head){
              lm_var = lm_var + "," + duty_cicle_print[tail];
              tail = (tail + 1)%BUFFER_SIZE;
            }
          }else {
            return "err";
          }

          lm_var = lm_var + "\n";
          return lm_var;
      }

};

#endif
