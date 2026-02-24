// paramerers.h

#ifndef PARAMETERS_H   // 1. 如果沒定義過這個符號
#define PARAMETERS_H   // 2. 就定義它

#include <complex.h>

// --- 雷達規格 ---
#define NUM_ANTENNAS 4      // 1T4R
#define SIGNAL_LEN 100      // Range Bins
#define CODE_LEN 13         // Barker Code Length
#define PI 3.14159265358979323846

// --- CFAR 參數 ---
#define CFAR_GUARD 2        // Guard Cells
#define CFAR_TRAIN 4        // Reference Cells
#define CFAR_OFFSET 13     // Scaling Factor

// Barker Code
extern const double complex BARKER_C[CODE_LEN];

extern const double BARKER_REAL[CODE_LEN];

// 1. Raw Data Frame (4天線 x 100點)
typedef struct {
    double complex data[NUM_ANTENNAS][SIGNAL_LEN];
    int frame_id;
} RadarFrame;

// 2. Detection Task (Consumer 1 傳給 Consumer 2 的工單)
typedef struct {
    int frame_id;
    int range_bin_idx;
    double complex snapshot[NUM_ANTENNAS]; // 該距離點的 4 個天線值
} DoaTask;

// signal_model
void get_steering_vector(double complex *sv, double theta_deg);
void SIMO_PMCW_RADAR_SIGNAL(double complex data[NUM_ANTENNAS][SIGNAL_LEN], int *target_dist, int *target_angle);

// DSP function
void matched_filter(double complex* input, double* output);
int cfar_detector(double* signal, int length);
void capon(double complex snapshot[NUM_ANTENNAS], int *estimated_angle);

#endif 