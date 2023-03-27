#ifndef METRICS_HPP
#define METRICS_HPP

#include <tuple>
#define PMAX 0.108
using namespace std;

int N {0};
float E {0}, vis_sum {0}, flick_sum {0}, d_k1 {0}, d_k2 {0} , t_k1 {0};


void consume_E(float d_k, float t_k){
	E += PMAX * d_k * (t_k - t_k1);
  t_k1 = t_k;
}

void vis_err(float ref, float sensed){
	vis_sum += max(0, ref - sensed);
}

void flick_err(float d_k){
	if ((d_k - d_k1)*(d_k1 - d_k2) < 0){
		flick_sum = flick_sum + abs(d_k - d_k1) + abs(d_k1 - d_k2);
	} 
  d_k1 = d_k;
  d_k2 = d_k1;
}

tuple<int, float, float, float> compute_metrics(float ref, float sensed, int pwm, unsigned long t_k){
	N++;
  float d_k = pwm*4095;
	consume_E(d_k, t_k);
	vis_err(ref, sensed);
	flick_err(d_k);

  return make_tuple(N, E, vis_sum, flick_sum);
}

#endif
