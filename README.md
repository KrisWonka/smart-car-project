# Dual Sensor Smart Line-Following Car 🚗

This project is a smart transport vehicle using dual sensors:
- Camera (8-neighborhood image processing)
- Grayscale sensor (8-channel)

## Modules
- `main.cpp`: Entry
- `image.c/h`: Image line detection & centerline completion
- `makefile` — compilation config

## Features
- Line completion using Kalman, slope extension, least squares
- Sensor fusion to improve turn angle estimation


## How to Run
1. Compile with: make
2. Run: ./main