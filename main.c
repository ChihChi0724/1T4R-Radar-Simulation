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
pthread_mutex_t task_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t task_has_data = PTHREAD_COND_INITIALIZER; // 不需要 full wait，滿了就丟棄任務即可

// ==========================================
// 執行緒函數
// ==========================================

// --- Producer: 模擬 1T4R 接收 ---
void* producer_1t4r(void* arg) {
    srand(time(NULL));

    int target_dist = 30; // 初始距離
    int target_angle = -45; // 初始角度 (掃描到 +45)

    for (int f = 0; f < 20; f++) {

        usleep(100000); // 100ms Sampling Time

        pthread_mutex_lock(&raw_mutex);
        while (raw_count == RAW_BUFFER_SIZE) {
            pthread_cond_wait(&raw_empty, &raw_mutex);
        }

        RadarFrame *frame = &raw_buffer[raw_in];
        frame->frame_id = f;

        SIMO_PMCW_RADAR_SIGNAL(frame->data, &target_dist, &target_angle);

        printf("[RX] Frame %d: True Dist=%d, True Angle=%d deg\n", f, target_dist, target_angle);

        raw_in = (raw_in + 1) % RAW_BUFFER_SIZE;
        raw_count++;
        pthread_cond_signal(&raw_full);
        pthread_mutex_unlock(&raw_mutex);
    }

    return NULL;
}

void* consumer_range(void* arg) {
    while (1) {
        pthread_mutex_lock(&raw_mutex);
        while (raw_count == 0) {
            // 實際專案要處理 thread exit 條件，這裡省略
             pthread_cond_wait(&raw_full, &raw_mutex);
        }

        double mf_output[SIGNAL_LEN]; // 用來存匹配濾波後的結果

        int idx = raw_out;
        RadarFrame frame = raw_buffer[idx]; // 複製一份出來處理 (或用指標)
        
        raw_out = (raw_out + 1) % RAW_BUFFER_SIZE;
        raw_count--;
        pthread_cond_signal(&raw_empty);
        pthread_mutex_unlock(&raw_mutex);

        // --- Step 1: Range Processing (只用 Channel 0 做快速偵測) ---
        // Matched Filter
        matched_filter(frame.data[0], mf_output);

        int detected_idx = cfar_detector(mf_output, SIGNAL_LEN);

        // 若判定為有目標
        if (detected_idx != -1) { 
            printf("  -> [Range DSP] Frame %d Det at %d. Send to DOA...\n", frame.frame_id, detected_idx);
            
            // --- Step 2: 派發任務給 DOA Thread ---
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
        
        if (frame.frame_id >= 19) break; // 模擬結束條件
    }
    return NULL;
}

// --- Consumer 2: DOA Estimation (精密運算) ---
void* consumer_doa(void* arg) {
    while (1) {
        pthread_mutex_lock(&task_mutex);
        while (task_count == 0) {
             pthread_cond_wait(&task_has_data, &task_mutex);
        }

        DoaTask task = task_queue[task_out];
        task_out = (task_out + 1) % TASK_QUEUE_SIZE;
        task_count--;
        pthread_mutex_unlock(&task_mutex);

        // --- Step 3: CAPON Processing ---

        // 2. 角度掃描 (Scan)
        int estimated_angle = 0;
        capon(task.snapshot, &estimated_angle);

        // 模擬大量矩陣運算耗時
        usleep(200000); 

        printf("    => [DOA DSP] Frame %d Result: Dist=%d, Angle=%d deg\n", 
               task.frame_id, task.range_bin_idx, estimated_angle);

        if (task.frame_id >= 19) break;
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