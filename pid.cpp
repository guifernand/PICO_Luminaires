#include "pid.h"

pid::pid( float _h, float _K, float b_,float Ti_, float Td_, float N_,  float Tt_ )
    // member variable initialization list
    : h {_h}, K {_K}, b {b_}, Ti {Ti_}, Td {Td_}, Tt{Tt_}, N {N_}, I {0.0}, D {0.0}, y_old{0.0}, Kold{_K}, bold{b_}, anti_windup {true}, feedback {true}
  { 
    ad = Td/(Td+N*h);
    bd = Td*K*N/(Td+N*h);
    ao = h/Tt; 
  } // should check arguments validity
  
pair<float, float> pid::compute_control( float r, float y ) {
  float P = K*(b*r-y); 
  D = ad*D-bd*(y-y_old);
  I = I + Kold*(bold*r-y)-K*(b*r-y); 
  Kold = K;
  bold = b;
  
  float v = P+I+D;
  float u = 0;
  
  if( v < 0 ){
    u = 0;
  } else if( v > 4095 ){
    u = 4095;
  } else {
    u = v;
  }
  
  return make_pair( u, v );
}



/*void pid::re_compute_coefs(float gain){
  

}*/
