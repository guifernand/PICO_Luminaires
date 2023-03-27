#ifndef METRICS_HPP
#define METRICS_HPP

#include <tuple>
using namespace std;

//experimentaly determined value
#define PMAX 0.0018 

/*
 N - number of iterations of the done sumations, 
     used to perform calculations such as 1/N
 d_k1 - duty cycle at (current instant - 1)
 d_k2 - duty cycle at (current instant - 2)
 t_k1 - time in seconds of the (current instant - 1)
 */
int N_count {0};
float E {0}, vis_sum {0}, flick_sum {0}, d_k1 {0}, d_k2 {0} , t_k1 {0};

/*
Compute the energy consumed at the desk
inputs: t_k - time in seconds of the current measusrements
*/
void consume_E(float t_k){
	E += PMAX * d_k1 * (t_k - t_k1);
  
}
/*
 Compute the sumation for the visibility error
 inputs: ref - current reference value (lux)
         sensed - measured value of illuminance (lux)
 */
void vis_err(float ref, float sensed){
	vis_sum += max(0, ref - sensed);
}

/*
 Compute the sumation for the flicker error
 inputs: d_k - current duty cycle
 */
void flick_err(float d_k){
	if ((d_k - d_k1)*(d_k1 - d_k2) < 0){
		flick_sum = flick_sum + abs(d_k - d_k1) + abs(d_k1 - d_k2);
	}  
}

/*
 Compute the metrics of the system
 inputs: ref - current reference value (lux)
         sensed - measured value of illuminance (lux)
         pwm - current duty cycle codification value
         t_k - current time
 */
tuple<int, float, float, float> compute_metrics(float ref, float sensed, int pwm, unsigned long t_k){
	N_count++;
  float d_k = pwm*4095;
	consume_E(t_k);
	vis_err(ref, sensed);
	flick_err(d_k);

  t_k1 = t_k;
  d_k1 = d_k;
  d_k2 = d_k1;
  return make_tuple(N_count, E, vis_sum, flick_sum);
}

#endif
