# ==========================================
# 1T4R 雷達模擬系統 Makefile
# ==========================================

# Variables
CC      = gcc
CFLAGS  = -Wall -g -O2    # -Wall: 顯示警告, -g: 支援 GDB 除錯, -O2: 基礎最佳化
LDFLAGS = -lpthread -lm   # 連結函式庫: pthread (多執行緒), m (數學函式庫)

# 定義目標檔與原始碼
TARGET  = demo
SRCS    = main.c dsp_utils.c signal_raw_data.c
OBJS    = $(SRCS:.c=.o)   
HEADERS = parameters.h    


# 預設目標
all: $(TARGET)

# Linking
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJS) $(LDFLAGS)

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

# Clean
.PHONY: clean
clean:
	rm -f $(OBJS) $(TARGET)