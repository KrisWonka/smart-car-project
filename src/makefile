CXX = g++
CC = gcc

CFLAGS = -Wall -O2
CXXFLAGS = `pkg-config --cflags opencv4`
LDFLAGS = `pkg-config --libs opencv4`

all: main

main: main.cpp image.c
	$(CXX) $(CFLAGS) main.cpp image.c -o main $(CXXFLAGS) $(LDFLAGS)
