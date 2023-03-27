#ifndef PID_H
#define PID_H

#include "Arduino.h"
#include <utility> 
using namespace std;

class pid {
    float I, D, K, Ti, Td, b, h, y_old, Kold, bold, N, Tt, ad, bd, ao;
    bool anti_windup, feedback;
  public:
    explicit pid( float _h, float _K = 1, float b_ = 1, float Ti_ = 1, float Td_ = 0, float N_ = 10, float Tt_ = 1);
      ~pid() {};
      pair<float, float> compute_control( float r, float y );
      float housekeep( float r, float y, float u, float v  );
      inline void set_point(float Gain);
      inline bool getAntiW();
      inline void setAntiW(int val);
      inline bool getFeedB();
      inline void setFeedB(int val);
      
};

inline float pid::housekeep( float r, float y, float u, float v ) {
  float e = r - y;
  if (anti_windup){
    I += (K*h/Ti)*e + ao*(u-v);
  }else{
    I += (K*h/Ti)*e;
  }
  y_old = y;

  return r - y;
}

inline bool pid::getAntiW(){
  return anti_windup;
}

inline void pid::setAntiW(int val){
  anti_windup = val;
}

inline bool pid::getFeedB(){
  return feedback; 
}


inline void pid::setFeedB(int val){
  feedback = val; 
}

inline void pid::set_point(float Gain){
  b = 1/(K*Gain);
}


#endif //PID_H
