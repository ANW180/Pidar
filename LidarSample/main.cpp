#include "urg_sensor.h"
#include "urg_utils.h"
#include "urg_detect_os.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

int open_urg_sensor(urg_t *urg, int argc, char *argv[])
{
#if defined(URG_WINDOWS_OS)
    const char *device = "COM4";
#elif defined(URG_LINUX_OS)
    const char *device = "/dev/ttyACM0";
    //const char *device = "/dev/ttyUSB0";
#else
    const char *device = "/dev/tty.usbmodemfa131";
#endif
    urg_connection_type_t connection_type = URG_SERIAL;
    long baudrate_or_port = 115200;
    //const char *ip_address = "localhost";
    const char *ip_address = "192.168.0.10";
    int i;

    // \~japanese Ú±^CvÌØÖŠ
    for (i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "-e")) {
            connection_type = URG_ETHERNET;
            baudrate_or_port = 10940;
            device = ip_address;
        }
    }

    // \~japanese Ú±
    if (urg_open(urg, connection_type, device, baudrate_or_port) < 0) {
        printf("urg_open: %s, %ld: %s\n",
            device, baudrate_or_port, urg_error(urg));
        return -1;
    }

    return 0;
}

static void print_data(urg_t *urg, long data[], int data_n, long time_stamp)
{
#if 1
    int front_index;

    (void)data_n;

    // \~japanese OûÌf[^ÌÝð\Š
    front_index = urg_step2index(urg, 0);
    printf("%ld [mm], (%ld [msec])\n", data[front_index], time_stamp);

#else
    (void)time_stamp;

    int i;
    long min_distance;
    long max_distance;

    // \~japanese SÄÌf[^Ì X-Y ÌÊuð\Š
    urg_distance_min_max(urg, &min_distance, &max_distance);
    for (i = 0; i < data_n; ++i) {
        long l = data[i];
        double radian;
        long x;
        long y;

        if ((l <= min_distance) || (l >= max_distance)) {
            continue;
        }
        radian = urg_index2rad(urg, i);
        x = (long)(l * cos(radian));
        y = (long)(l * sin(radian));
        printf("(%ld, %ld), ", x, y);
    }
    printf("\n");
#endif
}


int main(int argc, char *argv[])
{
    enum {
        CAPTURE_TIMES = 10,
    };
    urg_t urg;
    long *data = NULL;
    long time_stamp;
    int n;
    int i;

    if (open_urg_sensor(&urg, argc, argv) < 0) {
        return 1;
    }

    data = (long *)malloc(urg_max_data_size(&urg) * sizeof(data[0]));
    if (!data) {
        perror("urg_max_index()");
        return 1;
    }

    // \~japanese f[^æŸ
#if 0
    // \~japanese f[^ÌæŸÍÍðÏX·éê
    urg_set_scanning_parameter(&urg,
                               urg_deg2step(&urg, -90),
                               urg_deg2step(&urg, +90), 0);
#endif

    urg_start_measurement(&urg, URG_DISTANCE, CAPTURE_TIMES, 0);
    for (i = 0; i < CAPTURE_TIMES; ++i) {
        n = urg_get_distance(&urg, data, &time_stamp);
        if (n <= 0) {
            printf("urg_get_distance: %s\n", urg_error(&urg));
            free(data);
            urg_close(&urg);
            return 1;
        }
        print_data(&urg, data, n, time_stamp);
    }

    // \~japanese Øf
    free(data);
    urg_close(&urg);

#if defined(URG_MSC)
    getchar();
#endif
    return 0;
}
