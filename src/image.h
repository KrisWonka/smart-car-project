#ifndef IMAGE_H

#include <stdint.h>

#define IMAGE_W 188
#define IMAGE_H 120
#define MAX_POINTS 5000
#define WHITE_PIXEL 255
#define BLACK_PIXEL 0
#define BORDER_MIN 1
#define BORDER_MAX (IMAGE_W - 2)

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t original_image[IMAGE_H][IMAGE_W];
extern uint8_t bin_image[IMAGE_H][IMAGE_W];
extern int l_border[IMAGE_H];
extern int r_border[IMAGE_H];
extern int center_line[IMAGE_H];

extern int l_border_top[IMAGE_H];
extern int r_border_top[IMAGE_H];
extern int center_line2[IMAGE_H];
extern int points_l2[MAX_POINTS][2];
extern int points_r2[MAX_POINTS][2];
extern int dir_l2[MAX_POINTS];
extern int dir_r2[MAX_POINTS];
extern int l_count2;
extern int r_count2;

extern int center_line_final[IMAGE_H];
extern int center_line2_filtered[IMAGE_H];

void read_pgm(const char *filename);
void save_pgm(const char *filename, uint8_t image[IMAGE_H][IMAGE_W]);
void turn_to_bin(void);
void get_start_point(int start_row, int *l_x, int *l_y, int *r_x, int *r_y);
void search_l_r(int l_start_x, int l_start_y, int r_start_x, int r_start_y);
void get_left(void);
void get_right(void);
void compute_center_line(void);
void save_borders(void);
void process_current_frame(void);

// 全局变量声明（若需要跨文件访问）
extern int l_border_top[IMAGE_H];
extern int r_border_top[IMAGE_H];
extern int center_line2[IMAGE_H];
extern int l_count2, r_count2;
extern int points_l2[MAX_POINTS][2];
extern int points_r2[MAX_POINTS][2];

// 函数声明（按模块组织）

// 从上往下八邻域边界搜索
void search_l_r_top_down(int start_row);
void get_left_down(void);
void get_right_down(void);
void compute_center_line_down(void);
void draw_center_line_down(void);

//补线
// 断裂检测
int detect_break_in_center_line(void);
int detect_break_in_center_line_down(int *fit_lx, int *fit_rx);

// 最小二乘拟合 + 补线
void fit_line_upward(int base_y, float *k_l, float *b_l, float *k_r, float *b_r);
void patch_border_line_downward(int start_y, int end_y, float k_l, float b_l, float k_r, float b_r);
// 卡尔曼滤波
void kalman_filter_center_line2(int y_start);
// 合并并绘制最终中心线
void smooth_center_line_blend(int y1, int range);
void merge_center_lines_final(int y_split);
void draw_center_line_final(void);

// 中心线计算（补线用）
void compute_center_line_down(void);

#ifdef __cplusplus
}
#endif

#endif