#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <string.h>

#define DEVICE "/dev/video4"

void list_resolutions(int fd, double target_ratio) {
    struct v4l2_fmtdesc fmt;
    struct v4l2_frmsizeenum frmsize;

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    printf("Supported resolutions (Aspect Ratio: %.2f):\n", target_ratio);

    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmt) == 0) {
        memset(&frmsize, 0, sizeof(frmsize));
        frmsize.pixel_format = fmt.pixelformat;

        while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) == 0) {
            if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
                int w = frmsize.discrete.width;
                int h = frmsize.discrete.height;
                double ratio = (double)w / h;

                if (fabs(ratio - target_ratio) < 0.05) { // 允许一点误差
                    printf("  %dx%d (%.2f)\n", w, h, ratio);
                }
            }
            frmsize.index++;
        }
        fmt.index++;
    }
}

int main() {
    int fd = open(DEVICE, O_RDWR);
    if (fd < 0) {
        perror("Failed to open video device");
        return 1;
    }

    double target_ratio = 4.0 / 3.0;
    list_resolutions(fd, target_ratio);

    close(fd);
    return 0;
}
