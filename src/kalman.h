#pragma once

typedef struct
{
    //initial values for the kalman filter
    float x_est_last;
    float P_last;
    //the noise in the system
    float Q;
    float R;
    
    float K;
    float P;
} kalman_filter_t;

void kalman_init(kalman_filter_t *filter, float Q, float R);

float kalman_update(kalman_filter_t *filter, float measured);
