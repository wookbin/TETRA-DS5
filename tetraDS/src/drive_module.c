
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "drive_module.h"
#include "rs232_common.h"


int m_check_X = 0;
int m_check_Y = 0;
unsigned int buf_x =0;
unsigned int buf_y =0;
int Left_Vel[8];
int Right_Vel[8];

unsigned int m_Befor_buf_x = 0;
unsigned int m_Befor_buf_y = 0;

unsigned char Left_data[2];
unsigned char Right_data[2];

int drvm_init(const char *device, int time_out)
{
	int fd;

	if((fd = open_port(device)) < 0)
	{
		return -1;
	}

	set_terminal(fd, 115200, (int)(time_out/100));

	return fd;
}

void drvm_deinit(int fd)
{
	close(fd);
}


int drvm_set_servo(int fd, int mode)
{
	int ret;
	unsigned char packet_buf[256] = {STX, 'D', 'B'};
	
	if(!fd) return -1;

	if(mode == 1) {//on
		packet_buf[3] = '1';
		packet_buf[4] = '1';
	}
	else {
		packet_buf[3] = '0';
		packet_buf[4] = '0';
	}
	packet_buf[5] = ETX;
	packet_buf[6] = make_lrc(&packet_buf[1], 5);

	ret = write(fd, packet_buf, 7);
	//fflush(fd);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);
	//fflush(fd);
	return ret;
}

int drvm_set_velocity_mode(int fd)
{
	int ret;
	unsigned char packet_buf[256] = {STX, 'C', 'Z', '1', '1', ETX};
	
	if(!fd) return -1;

	packet_buf[6] = make_lrc(&packet_buf[1], 5);

	ret = write(fd, packet_buf, 7);
	//fflush(fd);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);
	//fflush(fd);
	return ret;
}

int drvm_set_position_mode(int fd)
{
	int ret;
	unsigned char packet_buf[256] = {STX, 'C', 'Z', '0', '0', ETX};
	
	if(!fd) return -1;

	packet_buf[6] = make_lrc(&packet_buf[1], 5);

	ret = write(fd, packet_buf, 7);
	//fflush(fd);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);
	//fflush(fd);
	return ret;
}


int drvm_set_drive_err_reset(int fd)
{
	int ret;
	unsigned char packet_buf[256] = {STX, 'C', 'G', ETX};
	
	if(!fd) return -1;

	packet_buf[4] = make_lrc(&packet_buf[1], 3);

	ret = write(fd, packet_buf, 5);
	//fflush(fd);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);
	//fflush(fd);
	return ret;
}

int drvm_set_odometry(int fd, int x, int y, int theta)
{
	int ret;
	int index;
	unsigned char packet_buf[256] = {STX, 'C', 'X'};

	if(!fd) return -1;

	index = 3;
	ret = int2str(&packet_buf[index], x);
	if(x == 0)
	{
		ret = 1;
	}
	index += ret;
	packet_buf[index] = ';';
	index++;

	ret = int2str(&packet_buf[index], y);
	if(y == 0)
	{
		ret = 1;
	}
	index += ret;
	packet_buf[index] = ';';
	index++;
	ret = int2str(&packet_buf[index], theta);
	if(theta == 0)
	{
		ret = 1;
	}
	index += ret;

	packet_buf[index] = ETX;
	index++;

	packet_buf[index] = make_lrc(&packet_buf[1], index-1);
	index++;

	ret = write(fd, packet_buf, index);
	//fflush(fd);
	if(ret <= 0) return -2;
	
	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);
	//fflush(fd);

	return ret;
}

int drvm_set_velocity(int fd, int left, int right)
{
	int ret;
	int index;
	unsigned char packet_buf[255] = {STX, 'B', 'E'};

	if(!fd) return -1;

	index = 3;
	ret = int2str(&packet_buf[index], left);
	if(left == 0)
	{
		ret = 1;
	}
	index += ret;
	packet_buf[index] = ';';
	index++;

	ret = int2str(&packet_buf[index], right);
	if(right == 0)
	{
		ret = 1;
	}
	index += ret;

	packet_buf[index] = ETX;
	index++;

	packet_buf[index] = make_lrc(&packet_buf[1], index-1);
	index++;

	ret = write(fd, packet_buf, index);
	//fflush(fd);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);
	//fflush(fd);

	return ret;
}

int drvm_set_position(int fd, int type, int pass, int data)
{
	int ret;
	int index;
	unsigned char packet_buf[255] = {STX, 'B', 'D'};

	if(!fd) return -1;

	int2str(&packet_buf[3], type);
	int2str(&packet_buf[4], pass);
	index = 5;
	ret = int2str(&packet_buf[index], data);
	if(data == 0)
	{
		ret = 1;
	}
	index += ret;

	packet_buf[index] = ETX;
	index++;

	packet_buf[index] = make_lrc(&packet_buf[1], index-1);
	index++;

	ret = write(fd, packet_buf, index);
	//fflush(fd);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);


	return ret;
}

int drvm_read_bumper_emg(int fd, int *bumper, int *emg_state, int *left_error_code, int *right_error_code)
{
	int ret;
	unsigned char packet_buf[255] = {STX, 'A', 'A', ETX};
	int bumper_val1, bumper_val2;
	int emg_val;
	int binary[16] = {0};

	if(!fd) return -1;

	packet_buf[4] = make_lrc(&packet_buf[1], 3);

	ret = write(fd, packet_buf, 5);
	//fflush(fd);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);
	if(ret !=0 ) return ret;

	//Error Code add...
	memset(binary, 0, sizeof(int)*16);
	*left_error_code = packet_buf[3];
	memset(binary, 0, sizeof(int)*16);
	*right_error_code = packet_buf[5];

	memset(binary, 0, sizeof(int)*16);
	bumper_val1 = packet_buf[7] & 0x0f;
	
	memset(binary, 0, sizeof(int)*16);
	bumper_val2 = packet_buf[6] & 0x0f;

	*bumper = (bumper_val1) + (bumper_val2 << 4);

	memset(binary, 0, sizeof(int)*16);
	emg_val = packet_buf[8];
	decimal2binary(emg_val, binary);
	*emg_state = binary[0];

	return ret;
}

int drvm_read_drive_err(int fd, unsigned char *left_wheel_err, unsigned char *right_wheel_err)
{
	int ret;
	unsigned char packet_buf[256] = {STX, 'A', 'A', ETX};
	int binary[16] = {0};

	if(!fd) return -1;

	packet_buf[4] = make_lrc(&packet_buf[1], 3);

	ret = write(fd, packet_buf, 5);
	//fflush(fd);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);
	if(ret != 0 ) return ret;

	*left_wheel_err = packet_buf[3];
	*right_wheel_err = packet_buf[5];
	return ret;
}

int drvm_read_encoder(int fd, int *left, int *right)
{
	unsigned char packet_buf[256] = {STX, 'A', 'C','0', ETX};
	int ret;
	char tmp[11];

	if(!fd) return -1;

	packet_buf[5] = make_lrc(&packet_buf[1], 4);

	ret = write(fd, packet_buf, 6);
	//fflush(fd);
	if(ret <= 0) return -2;

	ret = get_response(fd, packet_buf);
	if(ret != 0 ) return ret;

	memcpy(tmp, &packet_buf[2], 10);
	tmp[11] = '\0';
	*left = atoi(tmp);
	memcpy(tmp, &packet_buf[12], 10);
	tmp[11] = '\0';
	*right = atoi(tmp);

	return ret;
}

int drvm_read_odometry(int fd, double *Xpos_mm, double *Ypos_mm, double *deg)
{
	unsigned char packet_buf[256] = {STX, 'A','C','1', ETX};
	int ret;
	char tmp[11];

	if(!fd) return -1;

	packet_buf[5] = make_lrc(&packet_buf[1], 4);

	ret = write(fd, packet_buf, 6);
	//fflush(fd);
	if(ret <= 0) return -2;

	ret = get_response(fd, packet_buf);
	if(ret != 0 ) return ret;

	memcpy(tmp, &packet_buf[2], 10);
	tmp[11] = '\0';
	*Xpos_mm = atof(tmp);

	memcpy(tmp, &packet_buf[12], 10);
	tmp[11] = '\0';
	*Ypos_mm = atof(tmp);

	memcpy(tmp, &packet_buf[22], 10);
	tmp[11] = '\0';
	*deg = atof(tmp);

	// for(int i=0; i<35; i++)
	// {
	// 	printf("packet_buf[%d]: %x \n", i, packet_buf[i]);
	// }
	// printf("------------------- \n");

	return ret;
}

int drvm_read_parameter(int fd, int param, int *value)
{
	unsigned char packet_buf[256] = {STX, 'X', 'L'};
	int ret;
	int index = 3;
	unsigned int val[255] = {0};

	if(!fd) return -1;

	ret = int2str(&packet_buf[index], param);
	index += ret;
	
	packet_buf[index] = ETX;
	index++;
	packet_buf[index] = make_lrc(&packet_buf[1], index-1);
	index++;
	ret = write(fd, packet_buf, index);

	ret = get_response(fd, packet_buf);
	if(ret != 0 ) return ret;

	str2int(packet_buf, val); 
	*value = val[0];

	return ret;
}


int drvm_set_parameter(int fd, int param, int value)
{
	unsigned char packet_buf[256] = {STX, 'X', 'B'};
	unsigned char packet_buf2[256] = {STX, 'X', 'D', ETX};
	int ret;
	int index = 3;

	if(!fd) return -1;

	if(param <= 9)
	{
		packet_buf[index] = '0';
		index ++;
	}

	ret = int2str(&packet_buf[index], param);
	index += ret;
	ret = int2str(&packet_buf[index], value);
	if(value == 0)
	{
		ret = 1;
	}
	index += ret;
	
	packet_buf[index] = ETX;
	index++;
	packet_buf[index] = make_lrc(&packet_buf[1], index-1);
	index++;
	ret = write(fd, packet_buf, index);
	ret = get_response(fd, packet_buf);
	if(ret) return -1;
	
	packet_buf2[4] = make_lrc(&packet_buf2[1], 3);
	ret = write(fd, packet_buf2, 5);
	//fflush(fd);

	ret = get_response(fd, packet_buf2);
	if(packet_buf2[1] == 0x34) ret = 0;

	//fflush(fd);

	if(ret) return -1;

	return 0;
}

int drvm_set_Charge_On(int fd)
{
	int ret;
	unsigned char packet_buf[256] = {STX, 'P', 'E'};
	
	if(!fd) return -1;

	packet_buf[3] = '2';
	packet_buf[4] = '5';
	packet_buf[5] = '5';
	
	packet_buf[6] = ETX;
	packet_buf[7] = make_lrc(&packet_buf[1], 6);

	ret = write(fd, packet_buf, 8);
	//fflush(fd);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);
	//fflush(fd);
	return ret;
}

int drvm_set_Charge_Off(int fd)
{
	int ret;
	unsigned char packet_buf[256] = {STX, 'P', 'E'};
	
	if(!fd) return -1;

	packet_buf[3] = '1';
	packet_buf[4] = '2';
	packet_buf[5] = '7';
	
	packet_buf[6] = ETX;
	packet_buf[7] = make_lrc(&packet_buf[1], 6);

	ret = write(fd, packet_buf, 8);
	//fflush(fd);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);
	//fflush(fd);
	return ret;
}

int drvm_set_velocity2(int fd, int left, int right, int *x, int *y, int *theta, int *bumper, int *emg)
{
	int ret;
	int index;
	int lrc_val;
	unsigned char packet_buf[255] = {STX, 'B','V'};
	if(left < 0)
	{
		//printf("[%04x] %02x %02x \n",left* -1, ((left*-1) >> 8) | 0x80, ((left*-1) & 0xff));
		Left_data[0] = ((left*-1) >> 8) | 0x80;
		Left_data[1] = ((left*-1) & 0xff);
		//printf("[L]:%02x %02x \n", Left_data[0], Left_data[1]);
	}
	else
	{
		//printf("[%04x] %02x %02x \n",left, (left >> 8), (left) & 0xff);
		Left_data[0] = left >> 8;
		Left_data[1] = left & 0xff;
		//printf("[L]:%02x %02x \n", Left_data[0], Left_data[1]);
	}

	if(right < 0)
	{
		//printf("[%04x] %02x %02x \n",right* -1, ((right*-1) >> 8) | 0x80, ((right*-1) & 0xff));
		Right_data[0] = ((right*-1) >> 8) | 0x80;
		Right_data[1] = ((right*-1) & 0xff);
		//printf("[R]:%02x %02x \n", Right_data[0], Right_data[1]);
	}
	else
	{
		//printf("[%04x] %02x %02x \n",right, (right >> 8), (right) & 0xff);
		Right_data[0] = right >> 8;
		Right_data[1] = right & 0xff;
		//printf("[R]:%02x %02x \n", Right_data[0], Right_data[1]);
	}

	if(!fd) return -1;

	packet_buf[3] = Left_data[0];
	packet_buf[4] = Left_data[1];
	packet_buf[5] = Right_data[0];
	packet_buf[6] = Right_data[1];
	packet_buf[7] = ETX;
	packet_buf[8] = make_lrc(&packet_buf[1], 7);
	ret = write(fd, packet_buf, 9);

	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);

	ret = get_response2(fd, packet_buf);
	
	if(packet_buf[1] == 0x30 || packet_buf[1] == 0x32)
	{
		// for(int j=0; j<16; j++)
		// {
		// 	printf("%02x ", packet_buf[j]);
		// }
		// printf("\n");

		m_check_X = (packet_buf[2] >> 7);
		buf_x = (packet_buf[5] & 0xff) | ((packet_buf[4] << 8) & 0xff00) | ((packet_buf[3] << 16) & 0xff0000) | ((packet_buf[2] << 24) & 0x7f000000);
		if(m_check_X == 0)
		{
			*x = 1 * buf_x;
		}
		else
		{
			*x = -1 * buf_x;
		}

		m_check_Y = (packet_buf[6] >> 7);
		buf_y = (packet_buf[9] & 0xff) | ((packet_buf[8] << 8) & 0xff00) | ((packet_buf[7] << 16) & 0xff0000) | ((packet_buf[6] << 24) & 0x7f000000);
		if(m_check_Y == 0)
		{
			*y = 1 * buf_y;
		}
		else
		{
			*y = -1 * buf_y;
		}

		*theta = (packet_buf[11] & 0xff) | ((packet_buf[10] << 8) & 0xff00);
		*bumper = (packet_buf[12] & 0xff);
		// *emg = (packet_buf[13] & 0xff);
		//printf("emg: %02x \n", packet_buf[13] & 0xff);
		if(packet_buf[1] != 0x30)
		{
			*emg = 1;
		}
		else
		{
			*emg = 0;
		}
	}
	else
	{
		//printf("Packet Error !!!!!!! \n");
		return -2;
	}
	

	return ret;
}
