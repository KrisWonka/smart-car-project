#include <opencv2/opencv.hpp>
#include <stdio.h>
#include "image.h" // 头文件
// #include "config.h" // 补线配置文件
#include <time.h>

//灰度传感器----------------------------------------------------------------------------------
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#define I2C_DEVICE "/dev/i2c-0"
#define I2C_ADDR 0x4C
#define MAX_ANGLE 40

float read_gray_angle(float thresh = 170.0f, float dist = 100.0f /*unit：mm*/) {
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
        if (buf[i] < thresh) {  // 视为命中黑线
            sum_pos += positions[i];
            count++;
        }
    }

    if (count == 0) return 0;  // 没有黑点，返回0角度

    float center_offset = sum_pos / count;  // mm单位
    float angle_rad = atan(center_offset / dist);  // 弧度
    float angle_deg = angle_rad * 180.0 / M_PI;    // 转为角度

    return angle_deg;
}

//摄像头------------------------------------------------------------------------------------------------
extern "C" {
    void process_current_frame(void); // 声明 C 函数
}

int main()
{   struct timespec last_time, current_time;
    int frame_counter = 0;
    double elapsed;
    
    clock_gettime(CLOCK_MONOTONIC, &last_time);
    
    // 初始化摄像头
    cv::VideoCapture cap(8);//摄像头编号
    // cv::VideoCapture cap("/dev/video4", cv::CAP_V4L2);
    // 设置相机参数
    // cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    // cap.set(cv::CAP_PROP_FPS, 60);


    if (!cap.isOpened()) {
        printf("摄像头打开失败\n");
        return -1;
    }

    // const char* method_names[] = {"最小二乘法", "斜率延伸法", "多项式拟合法", "卡尔曼滤波"};
    // printf("[INFO] 当前补线方法: %s\n", method_names[PATCH_METHOD]);

    while (1) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) continue;

        // 转灰度
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0); // 5x5核
        cv::equalizeHist(gray, gray);  // 提高对比度
        cv::resize(gray, gray, cv::Size(IMAGE_W, IMAGE_H)); // 尺寸适配

        // 复制到 C 数组
        memcpy(original_image, gray.data, IMAGE_W * IMAGE_H);

        // 调用图像处理算法
        process_current_frame();

        // 计算偏移量 & 打印

        //单cam版
        // int sum = 0;
        // for (int i = IMAGE_H - 10; i < IMAGE_H; i++)
        //     sum += center_line_final[i];
        // int offset = (sum / 10) - (IMAGE_W / 2)+11;
        // printf("偏移量: %d\n", offset);

        // 加串口发送 offset 给小车

        //双传感器版
        // 图像角度计算
        int sum = 0;
        for (int i = IMAGE_H - 15; i < IMAGE_H; i++)
            sum += center_line_final[i];
        float cam_offset = (sum / 15.0f) - (IMAGE_W / 2.0f) + 11.0f;
        float cam_angle = cam_offset * 0.4f;  // 每像素约0.4度

        // 灰度角度读取
        float gray_angle = read_gray_angle();

        // 加权融合
        // float fused_angle = 0.7f * cam_angle + 0.3f * gray_angle;
        // =======【互补协同滤波融合】======= //
        static float last_cam_angle = 0.0f;
        static float last_gray_angle = 0.0f;

        // 波动幅度（简单差分法）
        float cam_variation = fabs(cam_angle - last_cam_angle);
        float gray_variation = fabs(gray_angle - last_gray_angle);

        float angle_diff = fabs(cam_angle - gray_angle);
        float fused_angle = 0.0f;

        if (angle_diff < 10.0f) {
            // 两者接近，直接平均
            fused_angle = (cam_angle + gray_angle) / 2.0f;
        } else {
            // 取波动更小者
            fused_angle = (gray_variation < cam_variation) ? gray_angle : cam_angle;
        }

        // 更新历史值
        last_cam_angle = cam_angle;
        last_gray_angle = gray_angle;
        // =======【融合结束】======= //
        
        // 打印角度
        printf("CAM: %.2f°, GRAY: %.2f°, FUSED: %.2f°\n", cam_angle, gray_angle, fused_angle);




        // 把处理后的 bin_image 转回 Mat 显示
        cv::Mat processed(IMAGE_H, IMAGE_W, CV_8UC1);
        for (int i = 0; i < IMAGE_H; i++) {
            for (int j = 0; j < IMAGE_W; j++) {
                processed.at<uint8_t>(i, j) = bin_image[i][j];
            }
        }
        // 放大显示 (否则188x120)
        cv::Mat resized;
        cv::resize(processed, resized, cv::Size(IMAGE_W * 4, IMAGE_H * 4));

        // 显示处理后的图像
        cv::imshow("Processed Image", resized);
        // 显示原图像
        cv::imshow("frame", frame);
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
