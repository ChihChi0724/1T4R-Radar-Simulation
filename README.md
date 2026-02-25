# 1T4R SIMO Radar Baseband Processing Pipeline 
**(多執行緒 1T4R 雷達基頻訊號處理模擬系統)**

## 📝 專案簡介 (Project Overview)
本專案為一個基於 **C 語言** 與 **POSIX Threads (Pthreads)** 實作的雷達基頻訊號處理模擬系統。
針對 1T4R (單發四收) SIMO 架構，設計了一套 **Producer-Consumer Pipeline**。透過多執行緒技術，將高頻寬的ADC、低延遲的距離偵測 (Range DSP)，以及高運算量的角度估計 (DOA DSP) 進行解耦，完美模擬了真實雷達晶片中硬體加速器與 DSP Core 的協作模式，達成 Latency Hiding 與 Real-time Processing 的系統需求。

## 🚀 核心技術與亮點 (Technical Highlights)
* **平行運算架構 (Parallel Computing):** 設計三階段 Pipeline 執行緒，解決 DOA 矩陣分解耗時導致的系統阻塞與 Packet Drop 問題。
* **執行緒安全與同步 (Thread Safety & Sync):** 實作 Thread-safe 的 **Ring Buffer** 與 Task Queue，精準操作 `Mutex` 與 `Condition Variable`，避免 Race Condition 並實現 Graceful Shutdown。
* **基頻演算法落地 (Baseband Algorithms):** * 實作 **Matched Filter (匹配濾波)** 進行 Barker Code 脈衝壓縮。
  * 實作動態門檻 **CA-CFAR** 演算法，提升抗雜訊強健性。
  * 實作基於 Covariance Matrix 的 **Capon/Bartlett Beamforming** 進行高解析度 DOA (Direction of Arrival) 角度估計。
* **C 語言模組化設計:** 嚴格遵守 `.h` 與 `.c` 檔案分離規範，並自製 `Makefile` 自動化編譯流程。

## ⚙️ 系統架構圖 (System Architecture)

```text
[Thread 1: RX Producer]  --> 模擬 1T4R 天線接收、加入雜訊與相位延遲
        │
        ▼
 ( Ring Buffer 1 )       --> Thread-Safe Raw Data Buffer
        │
        ▼
[Thread 2: Range DSP]    --> [快路徑] Matched Filter + CA-CFAR 距離偵測
        │                    (若無目標則丟棄，若有目標則擷取 Snapshot)
        ▼
 ( Task Queue 2 )        --> Thread-Safe Snapshot Queue
        │
        ▼
[Thread 3: Angle DSP]    --> [慢路徑] Covariance Matrix + Capon DOA 掃描