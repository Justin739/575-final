#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/ioctl.h>


int main(int argc, const char * argv[]) {

	char serial_path[120];
	strcpy(serial_path, "/dev/ttyACM0");


	int ser;
	ser = open(serial_path, O_RDONLY | O_NOCTTY | O_NONBLOCK);
	if (ser == -1) {
		fprintf(stderr, "opening serial port %s failed: %s\n", serial_path, strerror(errno));
		printf("hint: is the power on and the usb plugged in?\n");
		exit(1);
	}

	//fcntl(ser, F_SETFL, 0);

	struct termios options;
	tcgetattr(ser, &options);
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);
	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag |= CRTSCTS;
	tcsetattr(ser, TCSAFLUSH, &options);

	char rx_buffer[4096];
	int rx_buffer_bytes;
	char line_buffer[1024];

	while(1)
	{
		usleep(100000);

		ioctl(ser, FIONREAD, &rx_buffer_bytes);
		read(ser, rx_buffer, rx_buffer_bytes);
		char c;
		int start = 0;
		int end = 0;
		int i;
		for (i = 0; i < rx_buffer_bytes; i++)
		{
			c = rx_buffer[i];
			if (c == '\n')
			{
				end = i;
				int j;
				for (j = start; j < end; j++);
				memcpy(line_buffer, &rx_buffer[start], end - start);
				line_buffer[end-start-1] = '\0';
				start = end + 1;

				//printf("%s\n", line_buffer);

				// process line buffer
				if (strncmp(line_buffer, "$GPGLL", 6) == 0) {
					// Position data: position fix, time of position fix, and status
					// $GPGLL,4202.38085,N,09338.50822,W,004402.00,A,D*72
					// 0 	Message ID $GPGLL
					// 1 	Latitude in dd mm,mmmm format (0-7 decimal places)
					// 2 	Direction of latitude N: North S: South
					// 3 	Longitude in ddd mm,mmmm format (0-7 decimal places)
					// 4 	Direction of longitude E: East W: West
					// 5 	UTC of position in hhmmss.ss format
					// 6 	Fixed text "A" shows that data is valid
					// 7 	The checksum data, always begins with *
					char extractor[16];
					extractor[0] = line_buffer[7];
					extractor[1] = line_buffer[8];
					extractor[2] = '\0';
					int lat_dec = atoi(extractor);

					extractor[0] = line_buffer[9];
					extractor[1] = line_buffer[10];
					extractor[2] = '\0';
					int lat_min_whole = atoi(extractor);

					extractor[0] = line_buffer[12];
					extractor[1] = line_buffer[13];
					extractor[2] = line_buffer[14];
					extractor[3] = line_buffer[15];
					extractor[4] = line_buffer[16];
					extractor[5] = '\0';
					int lat_min_fraction = atoi(extractor);

					float lat = lat_dec + (lat_min_whole + lat_min_fraction / 100000.0) / 60.0;
					if (line_buffer[18] != 'N')
					{
						lat *= -1.0;
					}

					extractor[0] = line_buffer[20];
					extractor[1] = line_buffer[21];
					extractor[2] = line_buffer[22];
					extractor[3] = '\0';
					int lon_dec = atoi(extractor);

					extractor[0] = line_buffer[23];
					extractor[1] = line_buffer[24];
					extractor[2] = '\0';
					int lon_min_whole = atoi(extractor);

					extractor[0] = line_buffer[26];
					extractor[1] = line_buffer[27];
					extractor[2] = line_buffer[28];
					extractor[3] = line_buffer[29];
					extractor[4] = line_buffer[30];
					extractor[5] = '\0';
					int lon_min_fraction = atoi(extractor);

					float lon = lon_dec + (lon_min_whole + lon_min_fraction / 100000.0) / 60.0;
					if (line_buffer[32] != 'E')
					{
						lon *= -1.0;
					}

					// hhmmss.ss
					extractor[0] = line_buffer[34];
					extractor[1] = line_buffer[35];
					extractor[2] = '\0';
					int hrs = atoi(extractor);

					extractor[0] = line_buffer[36];
					extractor[1] = line_buffer[37];
					extractor[2] = '\0';
					int min = atoi(extractor);

					extractor[0] = line_buffer[38];
					extractor[1] = line_buffer[39];
					extractor[2] = '\0';
					int sec_whole = atoi(extractor);

					extractor[0] = line_buffer[41];
					extractor[1] = line_buffer[42];
					extractor[2] = '\0';
					int sec_fraction = atoi(extractor);

					float sec = sec_whole + sec_fraction / 100.0;

					double time = sec + min * 60.0 + hrs * 3600.0;

					printf("%f:\t%f,%f\n", time, lat, lon);
					//printf("%s\n", line_buffer);
				}
			}

		}

	}



	close(ser);
	return 0;
}
