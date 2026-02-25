// signal_raw_data.c

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <complex.h>
#include "parameters.h"

// Barker Code
const double complex BARKER_C[CODE_LEN] = {1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1};

const double BARKER_REAL[13] = {1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1};

// Steering Vector
// theta: 入射角, d_lambda: 天線間距(單位波長)
void get_steering_vector(double complex *sv, double theta_deg) {

    double theta_rad = theta_deg * PI / 180.0;

    for(int i = 0; i < NUM_ANTENNAS; i++) {
        // e^(-j * 2*pi * i * 0.5 * sin(theta))
        double phase = -2.0 * PI * i * 0.5 * sin(theta_rad);
        sv[i] = cexp(I * phase);
    }
}

// PMCW Signal Model
void SIMO_PMCW_RADAR_SIGNAL(double complex data[NUM_ANTENNAS][SIGNAL_LEN], int *haveTarget, int *target_dist, int *target_angle) {

    *haveTarget = rand() % 2;

    static int angle_step = 5;
    static int range_step = 2;

    // 1. 初始化雜訊
    for (int ch = 0; ch < NUM_ANTENNAS; ch++) {
        for (int i = 0; i < SIGNAL_LEN; i++) {
            // 產生複數高斯雜訊
            double real = ((double)rand()/RAND_MAX - 0.5) * 0.1;
            double imag = ((double)rand()/RAND_MAX - 0.5) * 0.1;
            data[ch][i] = real + I*imag;
        }
    }

     // 2. 加入目標訊號 (如果有目標)
    if (*haveTarget) {
        *target_dist += range_step;

        *target_angle += angle_step;
        if (*target_angle > 45 || *target_angle < -45) angle_step *= -1;

        double complex sv[NUM_ANTENNAS];
        get_steering_vector(sv, (double)*target_angle);

        for (int k = 0; k < 13; k++) {
            if (*target_dist + k >= SIGNAL_LEN) break;
            for (int ch = 0; ch < NUM_ANTENNAS; ch++) {
                // 訊號 = Barker Code * Steering Vector * 訊號強度
                data[ch][*target_dist + k] += BARKER_REAL[k] * sv[ch] * 2.0;
            }
        }
    }

}