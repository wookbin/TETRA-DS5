#include "rs232.h"

// Protocol Packet
#define STX				0x02
#define ETX             0x03
#define ACK             0x06
#define NAK             0x15
#define RST             0x12

#define	FLAG_OK			0x30 // Processing Success
#define	FLAG_ERROR		0x31 // Protocol Error
#define FLAG_FAIL		0x32 // Processing Fail
#define	FLAG_NO			0x33 // No Processing
#define	FLAG_END		0x34 // End of Packet
#define	FLAG_PASS		0x35 // Pass through
#define	FLAG_COMPLETE	0x36 // Motion Complete

unsigned char make_lrc(unsigned char data[], int num);
unsigned long mypow(int i, int j);
int int2str(unsigned char data[], int num);
void str2int(unsigned char data[], unsigned int dist[]);
void display_error_message(unsigned char flag);
int get_response(int fd, unsigned char data[]);
void decimal2binary(int decimal, int binary[]);
int get_response2(int fd, unsigned char data[]);
