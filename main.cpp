#include "Capturer.h"
#include "Processor.h"

static volatile int running = 1;

void gps_worker(int gps_fd);
int gps_init(char* serial_path);
void gps_readline(int fd, char* buffer);
int gps_is_gll(char* buffer);
struct coord gps_parse_gll(char* gll_string);
void log_coord_string(struct coord currCoord, char* buffer);

void monitorStop(Capturer* cap) {
    // Open a window to detect key presses
    cv::namedWindow("Stopper Window", cv::WINDOW_AUTOSIZE);

    if (cv::waitKey(0) == 27) {
        std::cout << "--- Stopping capture ---" << std::endl;
        cap->stopCapture();
        running = false;
    }
}

void gps_worker(int gps_fd)
{
    /* create buffer for gps messages */
    char buffer[82];


    /* buffer for coordinate read */
    struct coord message_coord;


    int sync_time = 1;
    int valid_counter = 0;


    //////////////////////////////////////////
    char raw_log_buffer[64];


    FILE *raw_log = fopen("raw_log.txt", "a+");
    //////////////////////////////////////////


    /* infinite while loop for polling gps messages */
    while(running)
    {
        /* read gps message */
        gps_readline(gps_fd, buffer);


        /* check if message is gll */
        if (gps_is_gll(buffer))
        {
            /* parse gll message into a coordinate */
            message_coord = gps_parse_gll(buffer);


            /* check if coordinate is valid */
            if (message_coord.valid == 1)
            {
                //////////////////////////////////////////
                /* log coordinate into file */
                log_coord_string(message_coord, raw_log_buffer);
                fprintf(raw_log, "%s\n", raw_log_buffer);
                //////////////////////////////////////////


                /* sync time if second coordinate is received */
                if (sync_time && valid_counter == 1)
                {
                    /* get system real time */
                    struct timeval tv;
                    gettimeofday(&tv, NULL);
                    //double time_real = tv.tv_sec + tv.tv_usec / 1000000.0;


                    /* calculate time offset between real and gps time */
                    //gps_args->time_offset = time_real - message_coord.timestamp;


                    /* do not sync time again */
                    sync_time = 0;
                }
                /* update coordinate history */
                //gps_args->oldest = gps_args->previous;
                //gps_args->previous = gps_args->current;
                //gps_args->current = message_coord;


                valid_counter++;
            }
        }
    }
    //////////////////////////////////////////
    fclose(raw_log);
    //////////////////////////////////////////
}

int gps_init(char* serial_path)
{
    /* open serial port */
    int fd = open(serial_path, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) {
        fprintf(stderr, "opening serial port %s failed: %s\n", serial_path, strerror(errno));
        return fd;
    }


    /* set up serial port */
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    options.c_iflag |= IGNCR;
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag |= CRTSCTS;
    tcsetattr(fd, TCSAFLUSH, &options);


    /* return file descriptor */
    return fd;
}

void gps_readline(int fd, char* buffer)
{
    int i;
    char c;

    /* wait for '$' character */
    do
    {
        read(fd, &c, 1);
    }
    while(c != '$');


    /* buffer string until '\n' character is encountered */
    buffer[0] = '$';
    read(fd, &c, 1);
    for (i = 1; c != '\n' && i < 82; i++)
    {
        buffer[i] = c;
        read(fd, &c, 1);
    }


    /* null terminate the string */
    buffer[i+1] = '\0';
}

int gps_is_gll(char* buffer)
{
    if (strncmp(buffer, "$GPGLL", 6) == 0)
        return 1;
    return 0;
}

struct coord gps_parse_gll(char* gll_string)
{
    char buffer[16];
    struct coord parsed;


    /* parse latitude */
    strncpy(buffer, gll_string+7, 2);
    buffer[2] = '\0';
    int lat_whole = atoi(buffer);
    strncpy(buffer, gll_string+9, 8);
    buffer[8] = '\0';
    double lat_frac = atof(buffer);
    parsed.lat = lat_whole + lat_frac / 60.0;
    if (gll_string[18] == 'S')
        parsed.lat = -parsed.lat;


    /* parse longitude */
    strncpy(buffer, gll_string+20, 3);
    buffer[3] = '\0';
    int lon_whole = atoi(buffer);
    strncpy(buffer, gll_string+23, 8);
    buffer[8] = '\0';
    double lon_frac = atof(buffer);
    parsed.lon = lon_whole + lon_frac / 60.0;
    if (gll_string[32] == 'W')
        parsed.lon = -parsed.lon;


    /* parse timestamp */
    strncpy(buffer, gll_string+34, 2);
    buffer[2] = '\0';
    int hours = atoi(buffer);
    strncpy(buffer, gll_string+36, 2);
    buffer[2] = '\0';
    int minutes = atoi(buffer);
    strncpy(buffer, gll_string+38, 5);
    buffer[5] = '\0';
    double seconds = atof(buffer);
    double timestamp = hours * 3600.0 + minutes * 60.0 + seconds;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int unix_epoch_midnight = (tv.tv_sec / 86400) * 86400;
    parsed.timestamp = timestamp + unix_epoch_midnight;


    /* pass validity flag */
    strncpy(buffer, gll_string+44, 1);
    buffer[1] = '\0';
    if (buffer[0] == 'A')
        parsed.valid = 1;
    else
        parsed.valid = 0;

    /* return result */
    return parsed;
}


void log_coord_string(struct coord currCoord, char* buffer) {
    sprintf(buffer, "%.10f,%.10f,%.10f", currCoord.timestamp, currCoord.lat, currCoord.lon);
}

int main(int argc, char* argv[]) {
    // Start capturing
    Capturer cap("/dev/video1", "recorded_video.avi", "frame_times.csv");
    std::thread stopper_thread(monitorStop, &cap);

    // Start the thread to update GPS lat/long whenever the GPS gets an update
    int gps_fd = gps_init("/dev/ttyACM0");
    std::thread gps_thread(gps_worker, gps_fd);

    std::cout << "--- Starting capture ---" << std::endl;
    cap.startCapture();
    stopper_thread.join();
    gps_thread.join();

    // Start processing
    /*
    Processor proc("recorded_video.avi", "processed_video.avi", "frame_times.csv", "raw_log.txt");
    proc.processData();
    */
    return 0;
}