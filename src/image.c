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

//补线
int l_border_top[IMAGE_H];
int r_border_top[IMAGE_H];
int center_line2[IMAGE_H];
int l_count2, r_count2;
int points_l2[MAX_POINTS][2];
int points_r2[MAX_POINTS][2];
int dir_l2[MAX_POINTS];
int dir_r2[MAX_POINTS];


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
    int bias = OTSU_BIAS;  // 负值：让图像更黑，抑制白块；正值：让图像更亮
    // printf("偏移值：%d\n", bias);
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
            // threshold = t;
            threshold = (t + bias < 0) ? 0 : ((t + bias > 255) ? 255 : t + bias);
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

//获取起点
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
//从上往下获取边界
void search_l_r_top_down(int l_start_x, int l_start_y, int r_start_x, int r_start_y)
{
    const int8_t seeds_r[8][2] = { {0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,0},{1,1} };
    const int8_t seeds_l[8][2] = { {0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1} };

    int cx_l = l_start_x, cy_l = l_start_y;
    int cx_r = r_start_x, cy_r = r_start_y;
    l_count = r_count = 0;
    hightest = IMAGE_H - 1;

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
    l_count2 = l_count;
    r_count2 = r_count;

    for (int i = 0; i < l_count; i++) {
        points_l2[i][0] = points_l[i][0];
        points_l2[i][1] = points_l[i][1];
        dir_l2[i] = dir_l[i];
    }

    for (int i = 0; i < r_count; i++) {
        points_r2[i][0] = points_r[i][0];
        points_r2[i][1] = points_r[i][1];
        dir_r2[i] = dir_r[i];
    }
    // printf("l_count: %d, r_count: %d\n", l_count, r_count);
}

void get_left_down(void)
{
    for (int i = 0; i < IMAGE_H; i++) l_border_top[i] = BORDER_MIN;

    for (int i = 0; i < l_count2; i++) {
        int y = points_l2[i][1];
        int x = points_l2[i][0];
        if (l_border_top[y] == BORDER_MIN) l_border_top[y] = x + 1;
    }
}

void get_right_down(void)
{
    for (int i = 0; i < IMAGE_H; i++) r_border_top[i] = BORDER_MAX;

    for (int i = 0; i < r_count2; i++) {
        int y = points_r2[i][1];
        int x = points_r2[i][0];
        if (r_border_top[y] == BORDER_MAX) r_border_top[y] = x - 1;
    }
}

void compute_center_line_down(void)
{
    for (int i = 0; i < IMAGE_H; i++) {
        center_line2[i] = (l_border_top[i] + r_border_top[i]) >> 1;
    }
}

int center_line2_filtered[IMAGE_H];  // 滤波后的中线
void kalman_filter_center_line2(int y_start)
{   
    float x = center_line2[1];  // 位置估计值
    float v = center_line2[2] - center_line2[1];;  // 速度（斜率）估计值
    float p_x = 0.3, p_v = 0.3;  // 初始误差
    float q_x = 0.05, q_v = 0.05;  // 过程噪声
    float r = 4.0;  // 测量噪声
    float k_x, k_v;  // 卡尔曼增益

    // 保留 y1 以下的部分为原始 center_line 数据
    for (int y = y_start; y < IMAGE_H; y++) {
        center_line2_filtered[y] = center_line[y]; 
    }

    for (int y = 1; y < y_start; y++) {
        int z = center_line2[y];  // 测量值（中心线位置）

        // 更新卡尔曼滤波
        k_x = p_x / (p_x + r);  // 位置卡尔曼增益
        k_v = p_v / (p_v + r);  // 斜率卡尔曼增益

        // 更新位置和斜率
        x = x + k_x * (z - x);  // 更新位置
        v = v + k_v * ((z - x) - v);  // 更新斜率（位置的变化）

        // 更新误差估计
        p_x = (1 - k_x) * p_x + q_x;
        p_v = (1 - k_v) * p_v + q_v;

        center_line2_filtered[y] = (int)x;  // 存储滤波后的结果
    }
}

//下往上断裂
int detect_break_in_center_line(void)
{
    for (int y = IMAGE_H - 2; y > 5; y--) {
        int c1 = center_line[y];
        int c2 = center_line[y - 1];
        if (abs(c1 - c2) > 5) { // 超过？像素跳变，视为断裂
            // printf("中线断裂点 y1 = %d\n", y);//-----------------------------------check
            y++;
            return y;
        }
    }
    return -1; // 没有断裂
}

//上往下断裂
int detect_break_in_center_line_down(int *fit_lx, int *fit_rx)
{
    for (int y = 5; y < IMAGE_H - 1; y++) {
        int c1 = center_line2[y];
        int c2 = center_line2[y + 1];
        if (abs(c1 - c2) > 5) {
            *fit_lx = l_border_top[y];  // 取断裂点前一行的左右边界
            *fit_rx = r_border_top[y];
            // printf("补线起点 y2 = %d, 左=%d, 右=%d\n", y, *fit_lx, *fit_rx);//--------------------------check
            y--;
            return y;
        }
    }
    return -1;
}

//最小二乘法拟合
void fit_line_upward(int base_y, float *k_l, float *b_l, float *k_r, float *b_r)
{
    int window = 10;

    float sum_x = 0, sum_y_l = 0, sum_xy_l = 0, sum_x2 = 0;
    float sum_y_r = 0, sum_xy_r = 0;
    int count = 0;

    for (int y = base_y - window; y < base_y; y++) {
        if (y <= 0 || y >= IMAGE_H) continue;

        int lx = l_border_top[y];
        int rx = r_border_top[y];
        if (lx <= 0 || rx <= 0 || lx >= IMAGE_W || rx >= IMAGE_W) continue;

        sum_x += y;
        sum_y_l += lx;
        sum_y_r += rx;
        sum_xy_l += y * lx;
        sum_xy_r += y * rx;
        sum_x2 += y * y;
        count++;
    }

    if (count == 0) {
        *k_l = *b_l = *k_r = *b_r = 0;
        return;
    }

    float denom = count * sum_x2 - sum_x * sum_x + 1e-5;
    *k_l = (count * sum_xy_l - sum_x * sum_y_l) / denom;
    *b_l = (sum_y_l - (*k_l) * sum_x) / count;

    *k_r = (count * sum_xy_r - sum_x * sum_y_r) / denom;
    *b_r = (sum_y_r - (*k_r) * sum_x) / count;

    // printf("拟合左边界: k=%.2f, b=%.2f\n", *k_l, *b_l);//-----------------------------------------check
    // printf("拟合右边界: k=%.2f, b=%.2f\n", *k_r, *b_r);//-----------------------------------------check
}

//补线
void patch_border_line_downward(int start_y, int end_y, float k_l, float b_l, float k_r, float b_r)
{
    for (int y = start_y; y <= end_y && y < IMAGE_H; y++) {
        int lx = (int)(k_l * y + b_l);
        int rx = (int)(k_r * y + b_r);

        // 限制左右边界不越界
        lx = lx < BORDER_MIN ? BORDER_MIN : (lx > BORDER_MAX ? BORDER_MAX : lx);
        rx = rx < BORDER_MIN ? BORDER_MIN : (rx > BORDER_MAX ? BORDER_MAX : rx);

        l_border_top[y] = lx;
        r_border_top[y] = rx;
    }
}

//合并中心线
int center_line_final[IMAGE_H];

void merge_center_lines_final(int y_split)
{   
    // printf("合并中心线 y_split = %d\n", y_split);//----------------------------------------check
    for (int y = 0; y <= y_split; y++) {
        center_line_final[y] = center_line2_filtered[y];
    }
    for (int y = y_split + 1; y < IMAGE_H; y++) {
        center_line_final[y] = center_line[y];
    }
}

//中心线平滑处理：
void smooth_center_line_blend(int y1, int range) {
    for (int i = 0; i < range; i++) {
        int y = y1 - range / 2 + i;
        if (y <= 0 || y >= IMAGE_H) continue;

        float t = i / (float)(range - 1);
        int val2 = center_line2_filtered[y];
        int val1 = center_line[y];
        center_line_final[y] = (int)((1 - t) * val1 + t * val2);
    }
}








//绘制
// 绘制下方中心线
void draw_center_line(void)
{
    for (int i = 0; i < IMAGE_H; i++) {
        if (center_line[i] >= 0 && center_line[i] < IMAGE_W){
            bin_image[i][center_line[i]] = 180; // 设置颜色
        }
}
}

// 绘制上方中心线
void draw_center_line_down(void)
{
    for (int i = 0; i < IMAGE_H; i++) {
        if (center_line2[i] >= 0 && center_line2[i] < IMAGE_W) {
            bin_image[i][center_line2[i]] = 120; 
        }
    }
}

// 绘制最终中心线
void draw_center_line_final(void)
{
    for (int i = 0; i < IMAGE_H; i++) {
        int x = center_line_final[i];
        if (x >= 0 && x < IMAGE_W){
            bin_image[i][x] = 120; 
        }
    }
}









// 主处理函数
void process_current_frame(void)
{
//图像处理---------------------------------------------------
    turn_to_bin();
    add_black_border(bin_image); 
//画线---------------------------------------------------
    int l_x, l_y, r_x, r_y;
    get_start_point(IMAGE_H - 2, &l_x, &l_y, &r_x, &r_y);
    search_l_r(l_x, l_y, r_x, r_y);
    get_left();
    get_right();
    compute_center_line();
    // draw_center_line();

    int l_top_x, l_top_y, r_top_x, r_top_y;
    get_start_point(1, &l_top_x, &l_top_y, &r_top_x, &r_top_y);
    search_l_r_top_down(l_top_x, l_top_y, r_top_x, r_top_y);
    get_left_down();
    get_right_down();
    compute_center_line_down();
    // draw_center_line_down();
//补线---------------------------------------------------
    // compute_center_line_down();

    int y1 = detect_break_in_center_line();
    int fit_lx, fit_rx;
    int y2 = detect_break_in_center_line_down(&fit_lx, &fit_rx);

    float k_l, b_l, k_r, b_r;
    fit_line_upward(y2 - 1, &k_l, &b_l, &k_r, &b_r);

    patch_border_line_downward(y2, y1, k_l, b_l, k_r, b_r);
    compute_center_line_down(); 
    kalman_filter_center_line2(y1);
    // smooth_center_line_blend(y1, 10); 
    merge_center_lines_final(y1);
    draw_center_line_final();
    // draw_center_line();


//调试--------------------------------------------------- 
    save_pgm("binary_output.pgm", bin_image);
    //save_borders();
}