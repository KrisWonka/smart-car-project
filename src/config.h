// config.h
#ifndef CONFIG_H
#define CONFIG_H

// 图像尺寸
#define IMAGE_W 188
#define IMAGE_H 120

// Otsu补偿值
#define OTSU_BIAS -20   // 图像二值化时的偏移值（让图像更黑或更亮）负值：让图像更黑，抑制白块；正值：让图像更亮

// 中线控制相关
#define BOTTOM_Y 15  // 控制用的中线高度行
#define WINDOW 5              // 前后差分的窗口大小
#define  N_POINT 30            //最小二乘法拟合点数
// 控制系数
#define K_P 0.55
#define K_D 0.3

// 舵机控制限幅
#define MAX_ANGLE 40

// // 灰度传感器
// #define I2C_DEVICE "/dev/i2c-0"
// #define I2C_ADDR 0x4C

// // 串口设备
// #define SERIAL_PORT "/dev/ttyS1"

#endif
