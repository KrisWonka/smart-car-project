#include <opencv2/opencv.hpp>
#include <stdio.h>
#include "image.h" // 头文件
#include "config.h" // 参数配置文件
#include <time.h>

// 灰度传感器----------------------------------------------------------------------------------
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

// 串口
#include <termios.h>

#define I2C_DEVICE "/dev/i2c-0"
#define I2C_ADDR 0x4C
// #define MAX_ANGLE 40

int init_serial(const char* device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("打开串口失败");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

float read_gray_angle(float thresh = 230.0f, float dist = 100.0f /*unit：mm*/) {
    int file;
    if ((file = open(I2C_DEVICE, O_RDWR)) < 0) {
        perror("无法打开I2C设备");
        return 0;
    }

    if (ioctl(file, I2C_SLAVE, I2C_ADDR) < 0) {
        perror("设置I2C地址失败");
        close(file);
        return 0;
    }

    // 初始化命令
    uint8_t init1[2] = {0xCF, 0xFF};
    write(file, init1, 2);
    uint8_t init2 = 0xB0;
    write(file, &init2, 1);

    // 读取灰度数据
    uint8_t cmd = 0xB0;
    write(file, &cmd, 1);
    uint8_t buf[8];
    if (read(file, buf, 8) != 8) {
        perror("读取失败");
        close(file);
        return 0;
    }
    close(file);

    printf("读取数据: ");
    for (int i = 0; i < 8; i++) {
        printf("%d ", buf[i]);
    }
    printf("\n");

    // 计算灰度传感器角度
    float positions[8] = {-38.5, -27.5, -16.5, -5.5, 5.5, 16.5, 27.5, 36.5};
    float sum_pos = 0;
    int count = 0;
    for (int i = 0; i < 8; i++) {
        if (buf[i] < thresh) {  
            sum_pos += positions[i];
            count++;
        }
    }

    if (count == 0) return 0; 

    float center_offset = sum_pos / count; 
    float angle_rad = atan(center_offset / dist);  // 弧度
    float angle_deg = angle_rad * 180.0 / M_PI;    // 角度

    return angle_deg;
}

//摄像头------------------------------------------------------------------------------------------------
extern "C" {
    void process_current_frame(void); // 主函数
}

int main()
{   int serial_fd = init_serial("/dev/ttyS1");  // 串口名
    if (serial_fd == -1) return -1;
    // 初始化 I2C
    struct timespec last_time, current_time;
    int frame_counter = 0;
    double elapsed;
    
    clock_gettime(CLOCK_MONOTONIC, &last_time);
    
    // 初始化摄像头
    cv::VideoCapture cap(0);//摄像头编号
    // cv::VideoCapture cap("/dev/video4", cv::CAP_V4L2);
    
    // 设置相机参数
    // cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    // cap.set(cv::CAP_PROP_FPS, 60);


    if (!cap.isOpened()) {
        printf("摄像头打开失败\n");
        return -1;
    }

    while (1) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) continue;

        // 转灰度
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0); // 高斯滤波 
        cv::equalizeHist(gray, gray);  // 提高对比度
        cv::resize(gray, gray, cv::Size(IMAGE_W, IMAGE_H)); //

        // 复制到 C 数组
        memcpy(original_image, gray.data, IMAGE_W * IMAGE_H);

        // 调用主函数
        process_current_frame();

        // 计算偏移量      
        // Step 1: 位置偏差
        int bottom_y = BOTTOM_Y;
        int window = WINDOW;
        int count=0;
        int sum=0;
        for (int i = IMAGE_H -bottom_y - window; i < IMAGE_H -bottom_y + window; i++) {
            sum+=center_line_final[i];
            count++;
        }
        int x_centerline = sum / count;
        int img_center = IMAGE_W / 2;
        float lateral_error = x_centerline - img_center;
        printf("x_centerline = %d | lateral_error = %.2f\n", x_centerline, lateral_error);
        // Step 2: 航向角误差（最小二乘法）
            // int dy = -10;
            // int x1 = center_line_final[bottom_y];
            // int x2 = center_line_final[bottom_y + dy];  // 注意方向
            // float dx = x1 - x2;
            // float heading_error = atan2(dy, dx) * 180.0 / M_PI +90;
            // printf("heading_error = %.2f\n", heading_error);
        float sum_x = 0.0f, sum_y = 0.0f, sum_xy = 0.0f, sum_yy = 0.0f;
        int n_points = N_POINT;

        for (int k = 0; k < n_points; ++k) {
            int y = IMAGE_H - 2 - k;             // 从底部往上选点 (y decreasing)
            float x = center_line_final[y];      // 中线的 x 坐标
            sum_x += x;
            sum_y += y;
            sum_xy += x * y;
            sum_yy += y * y;
        }

        float denominator = (n_points * sum_yy - sum_y * sum_y);
        float slope = (denominator == 0.0f) ? 0.0f :
                    (n_points * sum_xy - sum_x * sum_y) / denominator;

        float heading_error = atan(slope) * 180.0f / M_PI;  // 正值代表向右偏，负值向左偏
        // Step 3: 舵机角度计算
        float Kp = K_P, Kd = K_D;
        float steering_angle = Kp * lateral_error + Kd * heading_error;
        if (steering_angle > MAX_ANGLE) steering_angle = MAX_ANGLE;
        if (steering_angle < -MAX_ANGLE) steering_angle = -MAX_ANGLE;

        // 灰度角度读取
        float gray_angle = read_gray_angle();

        // 加权融合
        float fused_angle = 1.0f * steering_angle + 0.0f * gray_angle;
        
        // 互补协同滤波融合
        // static float last_cam_angle = 0.0f;
        // static float last_gray_angle = 0.0f;

        // // 波动幅度（简单差分法）
        // float cam_variation = fabs(cam_angle - last_cam_angle);
        // float gray_variation = fabs(gray_angle - last_gray_angle);

        // float angle_diff = fabs(cam_angle - gray_angle);
        // float fused_angle = 0.0f;

        // if (angle_diff < 10.0f) {
        //     // 两者接近，直接平均
        //     fused_angle = (cam_angle + gray_angle) / 2.0f;
        // } else {
        //     // 取波动更小者
        //     fused_angle = (gray_variation < cam_variation) ? gray_angle : cam_angle;
        // }

        // // 更新历史值
        // last_cam_angle = cam_angle;
        // last_gray_angle = gray_angle;
        
        // 打印角度
        printf("CAM: %.2f°, GRAY: %.2f°, FUSED: %.2f°\n", steering_angle, gray_angle, fused_angle);

        // 发送数据
        char msg[16];
        snprintf(msg, sizeof(msg), "%.2f\n", fused_angle);
        write(serial_fd, msg, strlen(msg));
        // printf("%s\n",msg);


    //可视化
        // 把处理后的 bin_image 转回 Mat 显示
        cv::Mat processed(IMAGE_H, IMAGE_W, CV_8UC1);
        for (int i = 0; i < IMAGE_H; i++) {
            for (int j = 0; j < IMAGE_W; j++) {
                processed.at<uint8_t>(i, j) = bin_image[i][j];
            }
        }
        // 放大显示
        cv::Mat resized;
        cv::resize(processed, resized, cv::Size(IMAGE_W * 4, IMAGE_H * 4));

        // 处理后的图像
        cv::imshow("Processed Image", resized);
        // // 原图像
        // cv::imshow("frame", frame);
        if (cv::waitKey(1) == 27) break; // 按 ESC 退出

        // 计算帧率
        frame_counter++;
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        elapsed = (current_time.tv_sec - last_time.tv_sec) + 
        (current_time.tv_nsec - last_time.tv_nsec) / 1e9;

        if (elapsed >= 1.0) {
            printf("FPS: %d\n", frame_counter);
            frame_counter = 0;
            last_time = current_time;
        }
    }

    return 0;
}
