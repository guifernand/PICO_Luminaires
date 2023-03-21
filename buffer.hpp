#ifndef BUFFER_HPP
#define BUFFER_HPP

#include "Arduino.h"
#include <string>
#include <iostream>
using namespace std;

class Buffer {
  private:
      static const int BUFFER_SIZE {6000};
      int head {0}, n_stored {0};

      struct DataPoint {
        float sensed {0};
        float duty_cicle {0};
      };
      
      DataPoint data[BUFFER_SIZE];
  
  public:
      Buffer(){}
  
      void addData(float sensed_){    
          data[head].sensed = sensed_;
          
          head = (head + 1)%BUFFER_SIZE;
  
          if (n_stored < 6000){
            n_stored++;
          }
          
      }

      float getSensed(int before_t = 0){
        return data[(head - before_t)%BUFFER_SIZE].sensed;
      }

      float getDuty(int before_t = 0){
        return data[(head - before_t)%BUFFER_SIZE].duty_cicle;
      }
      
      DataPoint get_pastData(int before_t = 0){
         return data[(head - before_t)%BUFFER_SIZE];
      }
      
      const char* showData(char var, char desk){
          string lm_var = "b";
          int tail = (head + BUFFER_SIZE - n_stored + 1 ) % BUFFER_SIZE;
          Serial.println("Entro");
          noInterrupts();
          if (var  == 'l'){
            while(tail != head){
              lm_var = lm_var + "," + to_string(data[tail].sensed);
              tail = (tail + 1)%BUFFER_SIZE;
            }
          }else if (var == 'd'){
            while(tail != head){
              lm_var = lm_var + to_string(data[tail].duty_cicle);
              tail = (tail + 1)%BUFFER_SIZE;
            }
          }
          lm_var = lm_var + "\n";
          interrupts();
          Serial.print("string -->"); Serial.println(lm_var.c_str());
          return lm_var.c_str();
      }

};

#endif
