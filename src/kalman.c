// the simplest Kalman filter (1x1)
// https://gist.github.com/jannson/9951716

#include <string.h>
#include "kalman.h"

void kalman_init(kalman_filter_t *filter, float Q, float R)
{
    memset(filter, 0, sizeof(*filter));
    filter->Q = Q;
    filter->R = R;
}

float kalman_update(kalman_filter_t *filter, float measured)
{
    float x_temp_est = filter->x_est_last;
    float P_temp = filter->P_last + filter->Q;
    //calculate the Kalman gain
    filter->K = P_temp * (1.0/(P_temp + filter->R));

    //correct
    float x_est = x_temp_est + filter->K * (measured - x_temp_est); 
    filter->P = (1- filter->K) * P_temp;
    
    filter->x_est_last = x_est;
    filter->P_last = filter->P;
    
    return x_est;
}
