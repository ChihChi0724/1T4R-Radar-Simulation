// dsp_utils.c

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <complex.h>
#include "parameters.h"

// --- [RANGE DSP] ---

// Matched Filter
void matched_filter(double complex* input, double* output) {
    // 初始化 output
    memset(output, 0, sizeof(double) * SIGNAL_LEN);

    for (int i = 0; i < SIGNAL_LEN - CODE_LEN; i++) {
        double sum = 0;

        for (int j = 0; j < CODE_LEN; j++) {
            // input[i+j] 是接收訊號, BARKER[j] 是參考訊號
            sum += input[i + j] * BARKER_REAL[j];
        }

        double power = creal(sum) * creal(sum) + cimag(sum) * cimag(sum);

        output[i] = power; // 這裡產生 Pulse Compression 後的結果
    }
}

// CA-CFAR 
int cfar_detector(double* signal, int length) {
    int detected_index = -1; // -1 代表沒抓到

    // Cell Under Test
    int start_idx = CFAR_TRAIN + CFAR_GUARD;
    int end_idx = length - (CFAR_TRAIN + CFAR_GUARD);

    for (int i = start_idx; i < end_idx; i++) {
        double noise_sum = 0;

        // 計算左邊訓練單元的總和
        for (int j = 1; j <= CFAR_TRAIN; j++) {
            noise_sum += signal[i - CFAR_GUARD - j]; // 左邊
            noise_sum += signal[i + CFAR_GUARD + j]; // 右邊
        }

        double noise_avg = noise_sum / (2 * CFAR_TRAIN); // 計算雜訊平均底噪
        double threshold = noise_avg * CFAR_OFFSET;      // 設定動態門檻

        // 如果訊號強度 > 門檻，且該點是區域最大值(簡單防抖動)，則判定為目標
        if (signal[i] > threshold && signal[i] > signal[i-1] && signal[i] > signal[i+1]) {
            detected_index = i;
            break; // 簡化：假設一個視窗只抓一個最強目標
        }
    }
    return detected_index;
}

// --- [ANGLE DSP] ---

// CAPON
// 掃描 -60 到 +60 度
void capon(double complex snapshot[NUM_ANTENNAS], int *estimated_angle) {

    double max_spectrum = -1.0;
    int best_angle = 0;

    // Covariance Matrix
    double complex Rxx[NUM_ANTENNAS][NUM_ANTENNAS];
    for (int i = 0; i < NUM_ANTENNAS; i++) {
        for (int j = 0; j < NUM_ANTENNAS; j++) {
            Rxx[i][j] = snapshot[i] * conj(snapshot[j]);
        }
    }
    
    for (int theta = -60; theta <= 60; theta += 2) {
        double complex a[NUM_ANTENNAS]; // Steering Vector
        get_steering_vector(a, (double)theta);

        // 計算 P = a^H * R * a (Bartlett Power)
        double complex power_c = 0;
        for (int i = 0; i < NUM_ANTENNAS; i++) {
            for (int j = 0; j < NUM_ANTENNAS; j++) {
                // a[i]^* * R[i][j] * a[j]
                power_c += conj(a[i]) * Rxx[i][j] * a[j];
            }
        }
        
        double p_mag = creal(power_c); // Power 是實數
        if (p_mag > max_spectrum) {
            max_spectrum = p_mag;
            best_angle = theta;
        }
    }
    *estimated_angle = best_angle;
}