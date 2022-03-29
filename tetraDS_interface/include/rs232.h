#define COM1	"/dev/ttyS0"
#define COM2	"/dev/ttyS1"
#define TTYUSB0	"/dev/ttyUSB0"
#define TTYUSB1	"/dev/ttyUSB1"
#define TTYUSB2	"/dev/ttyUSB2"
#define TTYUSB3	"/dev/ttyUSB3"
#define TTYUSB4	"/dev/ttyUSB3"

int open_port(const char *device);
void close_port(int fd);
int set_terminal(int fd, int baudrate, int time);

