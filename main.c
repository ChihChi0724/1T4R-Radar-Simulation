// main.c

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <complex.h>
#include "parameters.h"

// --- 執行緒參數 ---
#define RAW_BUFFER_SIZE 5   // 存放完整 Frame 的 Buffer
#define TASK_QUEUE_SIZE 10  // 存放待解角度任務的 Queue

// ==========================================
// 全域共享資源 (需用 Mutex 保護)
// ==========================================

// Buffer 1: Raw Data
RadarFrame raw_buffer[RAW_BUFFER_SIZE];
int raw_count = 0, raw_in = 0, raw_out = 0;
pthread_mutex_t raw_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t raw_full = PTHREAD_COND_INITIALIZER;
pthread_cond_t raw_empty = PTHREAD_COND_INITIALIZER;

// Buffer 2: DOA Task Queue
DoaTask task_queue[TASK_QUEUE_SIZE];
int task_count = 0, task_in = 0, task_out = 0;
int is_range_done = 0; // 用來通知 DOA 前面已經處理完畢了
pthread_mutex_t task_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t task_has_data = PTHREAD_COND_INITIALIZER; // 不需要 full wait，滿了就丟棄任務即可

// ==========================================
// 執行緒函數
// ==========================================

// --- Producer: 模擬 1T4R 接收 ---
void* producer_1t4r(void* arg) {
    srand(time(NULL));

    int haveTarget = 0;
    int target_dist = 30; // 初始距離
    int target_angle = -45; // 初始角度 

    for (int f = 0; f < 20; f++) {

        usleep(100000); // 100ms Sampling Time

        pthread_mutex_lock(&raw_mutex);
        while (raw_count == RAW_BUFFER_SIZE) {
            pthread_cond_wait(&raw_empty, &raw_mutex);
        }

        RadarFrame *frame = &raw_buffer[raw_in];
        frame->frame_id = f;

        SIMO_PMCW_RADAR_SIGNAL(frame->data, &haveTarget, &target_dist, &target_angle);

        if (haveTarget) {
            printf("[RX] Frame %d: True Dist=%d, True Angle=%d deg\n", f, target_dist, target_angle);
        } else {
            printf("[RX] Frame %d: No Target", f);
        }

        raw_in = (raw_in + 1) % RAW_BUFFER_SIZE;
        raw_count++;
        pthread_cond_signal(&raw_full);
        pthread_mutex_unlock(&raw_mutex);
    }

    return NULL;
}

// --- Consumer 1: Range Detection ---
void* consumer_range(void* arg) {
    while (1) {
        pthread_mutex_lock(&raw_mutex);
        while (raw_count == 0) {
            pthread_cond_wait(&raw_full, &raw_mutex);
        }

        int idx = raw_out;
        RadarFrame frame = raw_buffer[idx]; // 複製一份出來處理
        
        raw_out = (raw_out + 1) % RAW_BUFFER_SIZE;
        raw_count--;
        pthread_cond_signal(&raw_empty);
        pthread_mutex_unlock(&raw_mutex);

        // --- Step 1: Range Processing ---
        // Matched Filter (Non-coherent Integration)
        double mf_output[SIGNAL_LEN];
        double combined_power[SIGNAL_LEN] = {0};

        for (int ch = 0; ch < NUM_ANTENNAS; ch++) {
            matched_filter(frame.data[ch], mf_output);
            for (int k = 0; k < SIGNAL_LEN; k++) {
                combined_power[k] += mf_output[k];
            }
        }

        int detected_idx = cfar_detector(combined_power, SIGNAL_LEN);

        // 若判定為有目標
        if (detected_idx != -1) { 
            printf("  -> [Range DSP] Frame %d Det at %d. Send to DOA...\n", frame.frame_id, detected_idx);
            
            // --- 派發任務給 DOA Thread ---
            pthread_mutex_lock(&task_mutex);
            if (task_count < TASK_QUEUE_SIZE) {
                task_queue[task_in].frame_id = frame.frame_id;
                task_queue[task_in].range_bin_idx = detected_idx;
                
                // 複製該 Range Bin 的 4 個天線值 (Snapshot)
                for(int ch = 0; ch < NUM_ANTENNAS; ch++) {
                    task_queue[task_in].snapshot[ch] = frame.data[ch][detected_idx];
                }

                task_in = (task_in + 1) % TASK_QUEUE_SIZE;
                task_count++;
                pthread_cond_signal(&task_has_data);
            }
            pthread_mutex_unlock(&task_mutex);
        } else {
             printf("  -> [Range DSP] Frame %d No Target.\n", frame.frame_id);
        }
        
        if (frame.frame_id >= 19) {
            pthread_mutex_lock(&task_mutex);
            is_range_done = 1; // 宣告 Range DSP 已經不再送資料了
            pthread_cond_signal(&task_has_data); 
            pthread_mutex_unlock(&task_mutex);
            break; 
        }
    }
    return NULL;
}

// --- Consumer 2: DOA Estimation ---
void* consumer_doa(void* arg) {
    while (1) {
        pthread_mutex_lock(&task_mutex);
        
        while (task_count == 0 && is_range_done == 0) {
             pthread_cond_wait(&task_has_data, &task_mutex);
        }

        if (task_count == 0 && is_range_done == 1) {
            pthread_mutex_unlock(&task_mutex);
            break; 
        }

        DoaTask task = task_queue[task_out];
        task_out = (task_out + 1) % TASK_QUEUE_SIZE;
        task_count--;
        pthread_mutex_unlock(&task_mutex);

        // --- CAPON Processing ---
        int estimated_angle = 0;
        capon(task.snapshot, &estimated_angle);

        // 模擬大量矩陣運算耗時
        usleep(200000); 

        printf("    => [DOA DSP] Frame %d Result: Dist=%d, Angle=%d deg\n", 
               task.frame_id, task.range_bin_idx, estimated_angle);

    }
    return NULL;
}

int main() {
    pthread_t t_rx, t_range, t_doa;

    printf("=== 1T4R Radar Simulation (Pipeline Mode) ===\n");

    pthread_create(&t_rx, NULL, producer_1t4r, NULL);
    pthread_create(&t_range, NULL, consumer_range, NULL);
    pthread_create(&t_doa, NULL, consumer_doa, NULL);

    pthread_join(t_rx, NULL);
    pthread_join(t_range, NULL);
    pthread_join(t_doa, NULL);

    printf("=== End Simulation ===\n");
    return 0;
}