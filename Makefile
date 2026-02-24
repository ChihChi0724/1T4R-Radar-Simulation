# ==========================================
# 1T4R 雷達模擬系統 Makefile
# ==========================================

# 1. 定義編譯器與參數 (Variables)
CC      = gcc
CFLAGS  = -Wall -g -O2    # -Wall: 顯示警告, -g: 支援 GDB 除錯, -O2: 基礎最佳化(適合 DSP 運算)
LDFLAGS = -lpthread -lm   # 連結函式庫: pthread (多執行緒), m (數學函式庫)

# 2. 定義目標檔與原始碼
TARGET  = demo
SRCS    = main.c dsp_utils.c signal_raw_data.c
OBJS    = $(SRCS:.c=.o)   # 將 SRCS 裡的 .c 替換成 .o (即 main.o dsp_utils.o signal_raw_data.o)
HEADERS = parameters.h    # 標頭檔 (如果您尚未改名，請將此處改為 paramerers.h)

# ==========================================
# 編譯規則 (Rules)
# ==========================================

# 預設目標：當你只輸入 `make` 時會執行的目標
all: $(TARGET)

# 連結 (Linking): 將所有 .o 檔結合起來產生最終執行檔
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJS) $(LDFLAGS)

# 編譯 (Compiling): 告訴 make 如何把 .c 變成 .o
# $< 代表依賴條件的第一個檔 (即 .c), $@ 代表目標檔 (即 .o)
%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

# 清理 (Clean): 刪除編譯產生的過程檔與執行檔
# .PHONY 告訴 make 這是一個假目標，避免剛好有一個檔案叫 "clean" 時發生衝突
.PHONY: clean
clean:
	rm -f $(OBJS) $(TARGET)