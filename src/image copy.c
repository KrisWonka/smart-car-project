#include "image.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "config.h"

uint8_t original_image[IMAGE_H][IMAGE_W];
uint8_t bin_image[IMAGE_H][IMAGE_W];
int points_l[MAX_POINTS][2];
int points_r[MAX_POINTS][2];
int dir_l[MAX_POINTS];
int dir_r[MAX_POINTS];
int l_border[IMAGE_H];
int r_border[IMAGE_H];
int center_line[IMAGE_H];
int l_count, r_count, hightest;

void read_pgm(const char *filename)
{
    FILE *fp = fopen(filename, "rb");
    if (!fp) {
        printf("Error opening image\n");
        exit(1);
    }
    char header[20];
    fread(header, sizeof(char), 15, fp);
    fread(original_image, sizeof(uint8_t), IMAGE_W * IMAGE_H, fp);
    fclose(fp);
}

void save_pgm(const char *filename, uint8_t image[IMAGE_H][IMAGE_W])
{
    FILE *fp = fopen(filename, "wb");
    fprintf(fp, "P5\n%d %d\n255\n", IMAGE_W, IMAGE_H);
    fwrite(image, sizeof(uint8_t), IMAGE_W * IMAGE_H, fp);
    fclose(fp);
}

void turn_to_bin(void)
{
    int hist[256] = {0};
    int total = IMAGE_W * IMAGE_H;
    for (int i = 0; i < IMAGE_H; i++)
        for (int j = 0; j < IMAGE_W; j++)
            hist[original_image[i][j]]++;

    int sum = 0, sumB = 0, q1 = 0, q2 = 0;
    double maxVar = 0;
    uint8_t threshold = 0;

    for (int i = 0; i < 256; i++) sum += i * hist[i];

    for (int t = 0; t < 256; t++) {
        q1 += hist[t];
        if (q1 == 0) continue;
        q2 = total - q1;
        if (q2 == 0) break;
        sumB += t * hist[t];
        double m1 = (double)sumB / q1;
        double m2 = (double)(sum - sumB) / q2;
        double varBetween = (double)q1 * q2 * (m1 - m2) * (m1 - m2);
        if (varBetween > maxVar) {
            maxVar = varBetween;
            threshold = t;
        }
    }

    for (int i = 0; i < IMAGE_H; i++)
        for (int j = 0; j < IMAGE_W; j++)
            bin_image[i][j] = (original_image[i][j] < threshold) ? WHITE_PIXEL : BLACK_PIXEL;
}

void add_black_border(uint8_t image[IMAGE_H][IMAGE_W])
{
    for (int i = 0; i < IMAGE_H; i++) {
        image[i][0] = 0;                     // 左边
        image[i][IMAGE_W - 1] = 0;           // 右边
    }

    for (int j = 0; j < IMAGE_W; j++) {
        image[0][j] = 0;                     // 顶部
        image[IMAGE_H - 1][j] = 0;           // 底部
    }
}


void get_start_point(int start_row, int *l_x, int *l_y, int *r_x, int *r_y)
{
    *l_x = 0; *l_y = start_row;
    *r_x = 0; *r_y = start_row;

    for (int i = IMAGE_W / 2; i > BORDER_MIN; i--)
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0) {
            *l_x = i;
            break;
        }

    for (int i = IMAGE_W / 2; i < BORDER_MAX; i++)
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i + 1] == 0) {
            *r_x = i;
            break;
        }
}
// 获取左右边界起始点
void search_l_r(int l_start_x, int l_start_y, int r_start_x, int r_start_y)
{
    const int8_t seeds_l[8][2] = { {0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,0},{1,1} };
    const int8_t seeds_r[8][2] = { {0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1} };

    int cx_l = l_start_x, cy_l = l_start_y;
    int cx_r = r_start_x, cy_r = r_start_y;
    l_count = r_count = 0;
    hightest = 0;

    for (int iter = 0; iter < MAX_POINTS; iter++) {
        points_l[l_count][0] = cx_l;
        points_l[l_count][1] = cy_l;
        l_count++;

        points_r[r_count][0] = cx_r;
        points_r[r_count][1] = cy_r;
        r_count++;

        int found = 0;
        for (int i = 0; i < 8; i++) {
            int nx = cx_l + seeds_l[i][0];
            int ny = cy_l + seeds_l[i][1];
            int nxn = cx_l + seeds_l[(i + 1) & 7][0];
            int nyn = cy_l + seeds_l[(i + 1) & 7][1];
            if (bin_image[ny][nx] == 0 && bin_image[nyn][nxn] == 255) {
                cx_l = nx;
                cy_l = ny;
                dir_l[l_count - 1] = i;
                found = 1;
                break;
            }
        }
        if (!found) break;

        found = 0;
        for (int i = 0; i < 8; i++) {
            int nx = cx_r + seeds_r[i][0];
            int ny = cy_r + seeds_r[i][1];
            int nxn = cx_r + seeds_r[(i + 1) & 7][0];
            int nyn = cy_r + seeds_r[(i + 1) & 7][1];
            if (bin_image[ny][nx] == 0 && bin_image[nyn][nxn] == 255) {
                cx_r = nx;
                cy_r = ny;
                dir_r[r_count - 1] = i;
                found = 1;
                break;
            }
        }
        if (!found) break;

        if (abs(cx_r - cx_l) <= 1 && abs(cy_r - cy_l) <= 1) {
            hightest = (cy_r + cy_l) >> 1;
            break;
        }
        if (cy_r < cy_l) continue;
    }
}
// 获取左边界
void get_left(void)
{
    for (int i = 0; i < IMAGE_H; i++) l_border[i] = BORDER_MIN;

    for (int i = 0; i < l_count; i++) {
        int y = points_l[i][1];
        int x = points_l[i][0];
        if (l_border[y] == BORDER_MIN) l_border[y] = x + 1;
    }
}
// 获取右边界
void get_right(void)
{
    for (int i = 0; i < IMAGE_H; i++) r_border[i] = BORDER_MAX;

    for (int i = 0; i < r_count; i++) {
        int y = points_r[i][1];
        int x = points_r[i][0];
        if (r_border[y] == BORDER_MAX) r_border[y] = x - 1;
    }
}

// 计算中心线
void compute_center_line(void)
{
    for (int i = 0; i < IMAGE_H; i++) {
        center_line[i] = (l_border[i] + r_border[i]) >> 1;
    }
}

// 保存边界线数据到文件
void save_borders(void)
{
    FILE *fp = fopen("border_output.txt", "w");
    for (int i = 0; i < IMAGE_H; i++) {
        fprintf(fp, "Row %d: L=%d R=%d C=%d\n", i, l_border[i], r_border[i], center_line[i]);
    }
    fclose(fp);
}


//补线函数
// 最小二乘法求斜率 k 和截距 b
void linear_regression(int y_start, int y_end, int *border, double *k, double *b)
{
    int n = y_end - y_start + 1;
    double sum_x = 0, sum_y = 0, sum_x2 = 0, sum_xy = 0;

    for (int i = y_start; i <= y_end; i++) {
        sum_x += i;
        sum_y += border[i];
        sum_x2 += i * i;
        sum_xy += i * border[i];
    }

    *k = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
    *b = (sum_y - (*k) * sum_x) / n;
}

// 最小二乘法补线
void patch_lines_least_square(void)
{
    // int break_num_l = -1, break_num_r = -1;

    // // 检测左边界拐点（斜率突变）
    // for (int i = 2; i < l_count - 2; i++) {
    //     int dx1 = points_l[i - 1][0] - points_l[i - 2][0];
    //     int dy1 = points_l[i - 1][1] - points_l[i - 2][1];
    //     int dx2 = points_l[i][0] - points_l[i - 1][0];
    //     int dy2 = points_l[i][1] - points_l[i - 1][1];

    //     float k1 = (float)dx1 / (dy1 + 1e-5);
    //     float k2 = (float)dx2 / (dy2 + 1e-5);

    //     if (fabs(k2 - k1) > 2.0 && k1 * k2 < 0) {
    //         break_num_l = points_l[i][1];
    //         printf("[拐点检测] 左拐点：y=%d, k1=%.2f, k2=%.2f\n", break_num_l, k1, k2);
    //         break;
    //     }
    // }

    // // 检测右边界拐点
    // for (int i = 2; i < r_count - 2; i++) {
    //     int dx1 = points_r[i - 1][0] - points_r[i - 2][0];
    //     int dy1 = points_r[i - 1][1] - points_r[i - 2][1];
    //     int dx2 = points_r[i][0] - points_r[i - 1][0];
    //     int dy2 = points_r[i][1] - points_r[i - 1][1];

    //     float k1 = (float)dx1 / (dy1 + 1e-5);
    //     float k2 = (float)dx2 / (dy2 + 1e-5);

    //     if (fabs(k2 - k1) > 2.0 && k1 * k2 < 0) {
    //         break_num_r = points_r[i][1];
    //         printf("[拐点检测] 右拐点：y=%d, k1=%.2f, k2=%.2f\n", break_num_r, k1, k2);
    //         break;
    //     }
    // }

    // // ---- 补左边 ----
    // if (break_num_l > 0 && break_num_l + 15 < IMAGE_H) {
    //     int y_start = break_num_l + 5;
    //     int y_end = break_num_l + 15;
    //     double k, b;
    //     linear_regression(y_start, y_end, l_border, &k, &b);

    //     for (int y = break_num_l; y >= 0; y--) {
    //         l_border[y] = (int)(k * y + b);
    //     }
    // }

    // // ---- 补右边 ----
    // if (break_num_r > 0 && break_num_r + 15 < IMAGE_H) {
    //     int y_start = break_num_r + 5;
    //     int y_end = break_num_r + 15;
    //     double k, b;
    //     linear_regression(y_start, y_end, r_border, &k, &b);

    //     for (int y = break_num_r; y >= 0; y--) {
    //         r_border[y] = (int)(k * y + b);
    //     }
    // }
    int break_num_l = 0, break_num_r = 0;
    int i;
    float slope_l_rate = 0, intercept_l = 0;
    int start = 0, end = 0;

    // -------- 十字识别 - 左边界（从上往下） --------
    for (i = l_count - 8; i >= 1; i--) {
        if (dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] == 6 && dir_l[i + 5] == 6 && dir_l[i + 7] == 6) {
            break_num_l = points_l[i][1];
            printf("十字拐点(左)：行 = %d, i = %d\n", break_num_l, i);
            break;
        }
    }

    // -------- 十字识别 - 右边界（从上往下） --------
    for (i = r_count - 8; i >= 1; i--) {
        if (dir_r[i - 1] == 4 && dir_r[i] == 4 && dir_r[i + 3] == 6 && dir_r[i + 5] == 6 && dir_r[i + 7] == 6) {
            break_num_r = points_r[i][1];
            printf("十字拐点(右)：行 = %d, i = %d\n", break_num_r, i);
            break;
        }
    }

    // -------- 补左边 --------
    if (break_num_l > 10 && break_num_l + 15 < IMAGE_H) {
        start = break_num_l;
        end = break_num_l + 10;
        double k, b;
        linear_regression(start, end, l_border, &k, &b);
        for (int y = break_num_l; y < IMAGE_H - 1; y++) {
            l_border[y] = (int)(k * y + b);
            if (l_border[y] < BORDER_MIN) l_border[y] = BORDER_MIN;
            if (l_border[y] > BORDER_MAX) l_border[y] = BORDER_MAX;
        }
    }

    // -------- 补右边 --------
    if (break_num_r > 10 && break_num_r + 15 < IMAGE_H) {
        start = break_num_r;
        end = break_num_r + 10;
        double k, b;
        linear_regression(start, end, r_border, &k, &b);
        for (int y = break_num_r; y < IMAGE_H - 1; y++) {
            r_border[y] = (int)(k * y + b);
            if (r_border[y] < BORDER_MIN) r_border[y] = BORDER_MIN;
            if (r_border[y] > BORDER_MAX) r_border[y] = BORDER_MAX;
        }
    }
}

// 斜率延伸法
void patch_lines_slope(void)
{
    int dy = 10; // 取最后10行
    int k_l = 0, k_r = 0;
    for (int i = l_count - dy; i < l_count - 1; i++)
        k_l += points_l[i + 1][0] - points_l[i][0];
    k_l /= dy;

    for (int i = r_count - dy; i < r_count - 1; i++)
        k_r += points_r[i + 1][0] - points_r[i][0];
    k_r /= dy;

    for (int y = points_l[l_count - 1][1]; y < IMAGE_H; y++)
        l_border[y] = l_border[y - 1] + k_l;

    for (int y = points_r[r_count - 1][1]; y < IMAGE_H; y++)
        r_border[y] = r_border[y - 1] + k_r;
}

// 多项式拟合法（二阶）
void patch_lines_poly(void)
{
    int n = 10; // 取拐点前10行
    double sum_x = 0, sum_x2 = 0, sum_x3 = 0, sum_x4 = 0;
    double sum_y_l = 0, sum_xy_l = 0, sum_x2y_l = 0;
    double sum_y_r = 0, sum_xy_r = 0, sum_x2y_r = 0;

    for (int i = hightest - n; i < hightest; i++) {
        double x = i;
        double y_l = l_border[i];
        double y_r = r_border[i];
        sum_x += x;
        sum_x2 += x * x;
        sum_x3 += x * x * x;
        sum_x4 += x * x * x * x;
        sum_y_l += y_l;
        sum_xy_l += x * y_l;
        sum_x2y_l += x * x * y_l;
        sum_y_r += y_r;
        sum_xy_r += x * y_r;
        sum_x2y_r += x * x * y_r;
    }

    double denom = n * sum_x2 * sum_x4 + 2 * sum_x * sum_x2 * sum_x3 - sum_x2 * sum_x2 * sum_x2 - n * sum_x3 * sum_x3 - sum_x * sum_x * sum_x4;

    // 左边界
    double a_l = (sum_y_l * sum_x2 * sum_x4 + sum_x * sum_x3 * sum_x2y_l + sum_x2 * sum_x3 * sum_xy_l - sum_x2 * sum_x2 * sum_x2y_l - sum_y_l * sum_x3 * sum_x3 - sum_x * sum_x4 * sum_xy_l) / denom;
    double b_l = (n * sum_x2y_l * sum_x4 + sum_x * sum_x3 * sum_y_l + sum_x2 * sum_x3 * sum_y_l - sum_x2 * sum_x2 * sum_x2y_l - n * sum_x3 * sum_xy_l - sum_x * sum_x4 * sum_y_l) / denom;
    double c_l = (n * sum_x2 * sum_x2y_l + sum_x * sum_x3 * sum_y_l + sum_x2 * sum_x3 * sum_y_l - sum_x2 * sum_x2 * sum_y_l - n * sum_x3 * sum_y_l - sum_x * sum_x4 * sum_y_l) / denom;

    // 补线
    for (int y = hightest; y < IMAGE_H; y++) {
        l_border[y] = (int)(a_l * y * y + b_l * y + c_l);
        r_border[y] = BORDER_MAX - (l_border[y] - BORDER_MIN); // 对称补右
    }
}

// 卡尔曼滤波补线

bool is_crossroad(int x, int y)
{
    int count = 0;
    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            if (i == 0 && j == 0) continue;
            int nx = x + j;
            int ny = y + i;
            if (nx < 0 || nx >= IMAGE_W || ny < 0 || ny >= IMAGE_H) continue;
            if (bin_image[ny][nx] == 0) count++; // 黑色像素
        }
    }
    return count >= 4;
}

void continue_border_kalman_from(int y_start)
{
    double x_l = l_border[y_start - 1];
    double v_l = x_l - l_border[y_start - 2];
    double p_l = 1, q = 0.01, r = 5;

    double x_r = r_border[y_start - 1];
    double v_r = x_r - r_border[y_start - 2];
    double p_r = 1;

    for (int y = y_start; y < IMAGE_H; y++) {
        if (l_border[y] == BORDER_MIN || is_crossroad((int)x_l, y - 1)) {
            x_l += v_l;
            p_l += q;
            double k = p_l / (p_l + r);
            x_l += k * (l_border[y - 1] - x_l);
            v_l = x_l - l_border[y - 1];
            p_l *= (1 - k);
            l_border[y] = (int)x_l;
        }

        if (r_border[y] == BORDER_MAX || is_crossroad((int)x_r, y - 1)) {
            x_r += v_r;
            p_r += q;
            double k = p_r / (p_r + r);
            x_r += k * (r_border[y - 1] - x_r);
            v_r = x_r - r_border[y - 1];
            p_r *= (1 - k);
            r_border[y] = (int)x_r;
        }
    }
}


void patch_lines_kalman(void)
{
    double x_l = l_border[hightest - 1];
    double v_l = l_border[hightest - 1] - l_border[hightest - 2]; // 初始速度
    double p_l = 1, q = 0.01, r = 5;

    double x_r = r_border[hightest - 1];
    double v_r = r_border[hightest - 1] - r_border[hightest - 2];
    double p_r = 1;

    for (int y = hightest; y < IMAGE_H; y++) {
        // --- 左边界 ---
        x_l += v_l;
        p_l += q;

        double k = p_l / (p_l + r);

        int measured_l = l_border[y - 1]; // 默认用上一帧

        // 如果是十字路口中心，我们不相信当前图像值，改用预测值
        if (is_crossroad(measured_l, y - 1)) {
            measured_l = (int)x_l; // 用预测值代替图像值
        }

        x_l += k * (measured_l - x_l);
        v_l = x_l - l_border[y - 1];
        p_l *= (1 - k);
        l_border[y] = (int)x_l;

        // --- 右边界 ---
        x_r += v_r;
        p_r += q;

        k = p_r / (p_r + r);
        int measured_r = r_border[y - 1];
        if (is_crossroad(measured_r, y - 1)) {
            measured_r = (int)x_r;
        }

        x_r += k * (measured_r - x_r);
        v_r = x_r - r_border[y - 1];
        p_r *= (1 - k);
        r_border[y] = (int)x_r;
    }

}

// 总控函数（替换原 patch_lines）
void patch_lines(void)
{
#if PATCH_METHOD == 0 // 最小二乘法
    patch_lines_least_square(); 
#elif PATCH_METHOD == 1 // 斜率延伸法
    patch_lines_slope();
#elif PATCH_METHOD == 2 // 多项式拟合
    patch_lines_poly();
#elif PATCH_METHOD == 3 // 卡尔曼滤波
    //patch_lines_kalman();
    continue_border_kalman_from(hightest);
#endif
}



// 绘制中心线
void draw_center_line(void)
{
    for (int i = 0; i < IMAGE_H; i++) {
        if (center_line[i] >= 0 && center_line[i] < IMAGE_W)
            bin_image[i][center_line[i]] = 128; // 设置为灰色
        
        // 画左边界（蓝色调：中等灰度）
        if (l_border[i] > 0 && l_border[i] < IMAGE_W)
            bin_image[i][l_border[i]] = 100;

        // 画右边界（蓝色调：深一些）
        if (r_border[i] > 0 && r_border[i] < IMAGE_W)
            bin_image[i][r_border[i]] = 200;
        }
    
}

// 主处理函数
void process_current_frame(void)
{
    turn_to_bin();
    add_black_border(bin_image); 
    int l_x, l_y, r_x, r_y;
    get_start_point(IMAGE_H - 2, &l_x, &l_y, &r_x, &r_y);
    search_l_r(l_x, l_y, r_x, r_y);
    get_left();
    get_right();
    patch_lines();
    compute_center_line();
    draw_center_line();
    //save_pgm("binary_output.pgm", bin_image);
    save_borders();
}