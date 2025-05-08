# CXX = g++
# CC = gcc

# CFLAGS = -Wall -O2
# CXXFLAGS = `pkg-config --cflags opencv4`
# LDFLAGS = `pkg-config --libs opencv4`

# all: main

# main: main.cpp image.c
# 	$(CXX) $(CFLAGS) main.cpp image.c -o main $(CXXFLAGS) $(LDFLAGS)
# 编译器设置
CXX = g++
CC = gcc

# 编译参数
CFLAGS = -Wall -O2
CXXFLAGS = -Wall -O2 `pkg-config --cflags opencv4`
LDFLAGS = `pkg-config --libs opencv4`

# 源码与目标
SRCS = main.cpp image.c
OBJS = main.o image.o
TARGET = main

# 默认目标
all: $(TARGET)

# 编译目标
$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET) $(LDFLAGS)

# 清理命令
clean:
	rm -f $(TARGET) *.o
