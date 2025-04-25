#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <string.h>
#include <math.h>

#define DEVICE "/dev/video4"

void list_resolutions(int fd, double target_ratio) {
    struct v4l2_fmtdesc fmt;
    struct v4l2_frmsizeenum frmsize;
    struct v4l2_frmivalenum frmival;

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    printf("Supported resolutions matching ratio %.2f:\n", target_ratio);

    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmt) == 0) {
        memset(&frmsize, 0, sizeof(frmsize));
        frmsize.pixel_format = fmt.pixelformat;

        while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) == 0) {
            if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
                int w = frmsize.discrete.width;
                int h = frmsize.discrete.height;
                double ratio = (double)w / h;

                if (fabs(ratio - target_ratio) < 0.05) {
                    // 查最大 FPS
                    memset(&frmival, 0, sizeof(frmival));
                    frmival.pixel_format = fmt.pixelformat;
                    frmival.width = w;
                    frmival.height = h;

                    int max_fps = 0;

                    while (ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmival) == 0) {
                        if (frmival.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
                            int fps = (int)(1.0 / (frmival.discrete.numerator / (double)frmival.discrete.denominator));
                            if (fps > max_fps) max_fps = fps;
                        }
                        frmival.index++;
                    }

                    printf("  %dx%d (%.2f)  Max FPS: %d\n", w, h, ratio, max_fps);
                }
            }
            frmsize.index++;
        }
        fmt.index++;
    }
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        printf("Usage: %s <aspect_ratio>\n", argv[0]);
        printf("Example: %s 16:9\n", argv[0]);
        return 1;
    }

    int w, h;
    if (sscanf(argv[1], "%d:%d", &w, &h) != 2 || h == 0) {
        fprintf(stderr, "Invalid aspect ratio format. Use like 16:9 or 4:3\n");
        return 1;
    }
    double target_ratio = (double)w / h;

    int fd = open(DEVICE, O_RDWR);
    if (fd < 0) {
        perror("Failed to open camera");
        return 1;
    }

    list_resolutions(fd, target_ratio);

    close(fd);
    return 0;
}
