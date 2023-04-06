#include <stdio.h>
#include <math.h>
#include <string.h>
#include "power_module.h"
#include "rs232_common.h"

DATA m_data;

int power_init(const char *device, int time_out)
{
	int fd;

	if((fd = open_port(device)) < 0)
	{
		return -1;
	}

	set_terminal(fd, 115200, (int)(time_out/100));

	memset(&m_data, 0, sizeof(DATA)); //initialize struct

	return fd;
}

void power_deinit(int fd)
{
	close(fd);
}

int power_set_light(int fd, int id, int brightness)
{
	int ret;
	int index;
	unsigned char packet_buf[255] = {STX, 'L', 'I'};

	if(!fd) return -1;

	index = 3;
	ret = int2str(&packet_buf[index], id);
	if(id == 0)
	{
		ret = 1;
	}
	index += ret;
	packet_buf[index] = ';';
	index++;

	ret = int2str(&packet_buf[index], brightness);
	if(brightness == 0)
	{
		ret = 1;
	}
	index += ret;

	packet_buf[index] = ETX;
	index++;

	packet_buf[index] = make_lrc(&packet_buf[1], index-1);
	index++;

	ret = write(fd, packet_buf, index);
	if(ret <= 0) return -2;
	
	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);

	return ret;
}

int power_set_Single_OutputPort(int fd, int id, int value)
{
	int ret;
	int index;
	unsigned char packet_buf[255] = {STX, 'P', 'o'};

	if(!fd) return -1;

	index = 3;
	ret = int2str(&packet_buf[index], id);
	if(id == 0)
	{
		ret = 1;
	}
	index += ret;
	packet_buf[index] = ';';
	index++;

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
	if(ret <= 0) return -2;
	
	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);

	return ret;
}

int power_set_Single_Enable(int fd, int id, int value)
{
	int ret;
	int index;
	unsigned char packet_buf[255] = {STX, 'P', 'e'};

	if(!fd) return -1;

	index = 3;
	ret = int2str(&packet_buf[index], id);
	if(id == 0)
	{
		ret = 1;
	}
	index += ret;
	packet_buf[index] = ';';
	index++;

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
	if(ret <= 0) return -2;
	
	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);

	return ret;
}

int power_toggle_on(int fd, int id)
{
	int ret;
	int index;
	unsigned char packet_buf[255] = {STX, 'L', 'T'};

	if(!fd) return -1;

	index = 3;
	ret = int2str(&packet_buf[index], id);
	if(id == 0)
	{
		ret = 1;
	}
	index += ret;
	
	packet_buf[index] = ETX;
	index++;

	packet_buf[index] = make_lrc(&packet_buf[1], index-1);
	index++;

	ret = write(fd, packet_buf, index);
	if(ret <= 0) return -2;
	
	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);

	return ret;
}

int power_set_light_toggle(int fd, int de_index, int lightAcc1, int H_brightness, 
							int lightAcc2, int L_brightness)
{
	int ret;
	int index;
	unsigned char packet_buf[255] = {STX, 'L', 'D'};

	if(!fd) return -1;

	index = 3;
	ret = int2str(&packet_buf[index], de_index);
	if(de_index == 0)
	{
		ret = 1;
	}
	index += ret;
	packet_buf[index] = ';';
	index++;

	ret = int2str(&packet_buf[index], lightAcc1);
	if(lightAcc1 == 0)
	{
		ret = 1;
	}
	index += ret;
	packet_buf[index] = ';';
	index++;

	ret = int2str(&packet_buf[index], H_brightness);
	if(H_brightness == 0)
	{
		ret = 1;
	}
	index += ret;
	packet_buf[index] = ';';
	index++;

	ret = int2str(&packet_buf[index], lightAcc2);
	if(lightAcc2 == 0)
	{
		ret = 1;
	}
	index += ret;
	packet_buf[index] = ';';
	index++;

	ret = int2str(&packet_buf[index], L_brightness);
	if(L_brightness == 0)
	{
		ret = 1;
	}
	index += ret;
	packet_buf[index] = ETX;
	index++;

	packet_buf[index] = make_lrc(&packet_buf[1], index-1);
	index++;

	ret = write(fd, packet_buf, index);
	if(ret <= 0) return -2;
	
	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);

	return ret;
}

int power_read_bumper(int fd, int *bumper)
{
	int ret;
	unsigned char packet_buf[255] = {STX, 'P', 'B', ETX};
	int bumper_val;
	int binary[16] = {0};

	if(!fd) return -1;
	packet_buf[4] = make_lrc(&packet_buf[1], 3);
	ret = write(fd, packet_buf, 5);

	if(ret <= 0) return -2;
	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);
	if(ret !=0 ) return ret;

	memset(binary, 0, sizeof(int)*16);
	bumper_val = packet_buf[3];
	decimal2binary(bumper_val, binary);
	bumper[0] = binary[0];
	bumper[1] = binary[1];
	bumper[2] = binary[2];
	bumper[3] = binary[3];
	bumper[4] = binary[4];
	bumper[5] = binary[5];
	bumper[6] = binary[6];
	bumper[7] = binary[7];

	return ret;
}

int power_read_Battery(int fd, double *dbattery, double *dVoltage, double *dCurrent, int *mode_status, int *Input, int *Output)
{
	unsigned char packet_buf[255] = {STX, 'P','V', ETX};
	int ret;
	int  Input_binary[8] = {0,};
	int Output_binary[8] = {0,};

	if(!fd) return -1;

	packet_buf[4] = make_lrc(&packet_buf[1], 3);

	ret = write(fd, packet_buf, 5);
	//fflush(fd);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response2(fd, packet_buf);
	if(packet_buf[1] == 0x02) //Packet Error Pass..
	{
		return -1;
	}
	
	//Battery Level
	int ibattery = (packet_buf[4] & 0xff) | ((packet_buf[3] << 8) & 0xff00);
	*dbattery = ibattery;

	int iVoltage = (packet_buf[6] & 0xff) | ((packet_buf[5] << 8) & 0xff00);
	*dVoltage = iVoltage / 10.0;

	int iCurrent = (packet_buf[8] & 0xff) | ((packet_buf[7] << 8) & 0xff00);
	*dCurrent = iCurrent / 10.0;
	
	// Charging Mode Status
	*mode_status = (packet_buf[10] & 0xff) | ((packet_buf[9] << 8) & 0xff00);

	//GPIO _add 2021.10.22 wbjin
	int iInput = (packet_buf[12] & 0xff) | ((packet_buf[11] << 8) & 0xff00);
	decimal2binary(iInput, Input_binary);
	int iOutput = (packet_buf[14] & 0xff) | ((packet_buf[13] << 8) & 0xff00);
	decimal2binary(iOutput, Output_binary);
	for(int i=0; i<8; i++)
	{
		Input[i]  = Input_binary[i];
		Output[i] = Output_binary[i];
	}

	return ret;
}

int power_read_tetra(int fd, double *dbattery, double *dVoltage, double *dCurrent, int *mode_status, int *Input, int *Output,double *Ultrasonic)
{
	unsigned char packet_buf[255] = {STX, 'P','Q', ETX};
	int ret;
	int  Input_binary[8] = {0,};
	int Output_binary[8] = {0,};

	if(!fd) return -1;

	packet_buf[4] = make_lrc(&packet_buf[1], 3);

	ret = write(fd, packet_buf, 5);
	//fflush(fd);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response2(fd, packet_buf);
	if(packet_buf[1] == 0x02) //Packet Error Pass..
	{
		return -1;
	}
	
	//Battery Level
	int ibattery = (packet_buf[4] & 0xff) | ((packet_buf[3] << 8) & 0xff00);
	*dbattery = ibattery;

	int iVoltage = (packet_buf[6] & 0xff) | ((packet_buf[5] << 8) & 0xff00);
	*dVoltage = iVoltage / 10.0;

	int iCurrent = (packet_buf[8] & 0xff) | ((packet_buf[7] << 8) & 0xff00);
	*dCurrent = iCurrent / 10.0;
	
	// Charging Mode Status
	*mode_status = (packet_buf[10] & 0xff) | ((packet_buf[9] << 8) & 0xff00);

	//GPIO _add 2021.10.22 wbjin
	int iInput = (packet_buf[12] & 0xff) | ((packet_buf[11] << 8) & 0xff00);
	decimal2binary(iInput, Input_binary);
	int iOutput = (packet_buf[14] & 0xff) | ((packet_buf[13] << 8) & 0xff00);
	decimal2binary(iOutput, Output_binary);
	for(int i=0; i<8; i++)
	{
		Input[i]  = Input_binary[i];
		Output[i] = Output_binary[i];
	}
	int _m_Sonar_1 = (packet_buf[16] & 0xff) | ((packet_buf[15] << 8) & 0xff00);
	Ultrasonic[1] = _m_Sonar_1 / 1000.0;
	int _m_Sonar_2 = (packet_buf[18] & 0xff) | ((packet_buf[17] << 8) & 0xff00);
	Ultrasonic[2] = _m_Sonar_2 / 1000.0;
	int _m_Sonar_3 = (packet_buf[20] & 0xff) | ((packet_buf[19] << 8) & 0xff00);
	Ultrasonic[3] = _m_Sonar_3 / 1000.0;
	int _m_Sonar_0= (packet_buf[22] & 0xff) | ((packet_buf[21] << 8) & 0xff00);
	Ultrasonic[0] = _m_Sonar_0 / 1000.0;


	return ret;
}

int  power_read_Ultrasonic(int fd,  double *Ultrasonic)
{
	unsigned char packet_buf[255] = {STX, 'P','N', ETX};
	int ret;
	if(!fd) return -1;
	packet_buf[4] = make_lrc(&packet_buf[1], 3);
	ret = write(fd, packet_buf, 5);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response2(fd, packet_buf);
	if(packet_buf[1] == 0x02) //Packet Error Pass..
	{
		return -1;
	}

	int _m_Sonar_1 = (packet_buf[6] & 0xff) | ((packet_buf[5] << 8) & 0xff00);
	Ultrasonic[1] = _m_Sonar_1 / 1000.0;
	int _m_Sonar_2 = (packet_buf[8] & 0xff) | ((packet_buf[7] << 8) & 0xff00);
	Ultrasonic[2] = _m_Sonar_2 / 1000.0;
	int _m_Sonar_3 = (packet_buf[10] & 0xff) | ((packet_buf[9] << 8) & 0xff00);
	Ultrasonic[3] = _m_Sonar_3 / 1000.0;
	int _m_Sonar_0= (packet_buf[12] & 0xff) | ((packet_buf[11] << 8) & 0xff00);
	Ultrasonic[0] = _m_Sonar_0 / 1000.0;

	return ret;
}


int power_set_Ultrasonic(int fd, int mode)
{
	int ret;
	unsigned char packet_buf[255] = {STX, 'P', 'S'};
	
	if(!fd) return -1;

	if(mode == 1)//On
	{
		packet_buf[3] = '1';
	}
	else//Off
	{
		packet_buf[3] = '0';
	}
	packet_buf[4] = ETX;
	packet_buf[5] = make_lrc(&packet_buf[1], 4);

	ret = write(fd, packet_buf, 6);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);
	return ret;
}


int  power_data_read(int fd, int *idata_0, int *idata_1, int *idata_2, int *idata_3,int *idata_4, int *idata_5, int *idata_6, int *idata_7, 
                          int *idata_8, int *idata_9, int *idata_10, int *idata_11)
{
	unsigned char packet_buf[255] = {STX, 'P','D', ETX};
	int ret;
	if(!fd) return -1;
	packet_buf[4] = make_lrc(&packet_buf[1], 3);
	ret = write(fd, packet_buf, 5);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response2(fd, packet_buf);
	if(packet_buf[1] == 0x02) //Packet Error Pass..
	{
		return -1;
	}

	int m_data0 = (packet_buf[4] & 0xff) | ((packet_buf[3] << 8) & 0xff00);
	*idata_0 = m_data0;
	int m_data1 = (packet_buf[6] & 0xff) | ((packet_buf[5] << 8) & 0xff00);
	*idata_1 = m_data1;
	int m_data2 = (packet_buf[8] & 0xff) | ((packet_buf[7] << 8) & 0xff00);
	*idata_2 = m_data2;
	int m_data3 = (packet_buf[10] & 0xff) | ((packet_buf[9] << 8) & 0xff00);
	*idata_3 = m_data3;
	int m_data4 = (packet_buf[12] & 0xff) | ((packet_buf[11] << 8) & 0xff00);
	*idata_4 = m_data4;
	int m_data5 = (packet_buf[14] & 0xff) | ((packet_buf[13] << 8) & 0xff00);
	*idata_5 = m_data5;
	int m_data6 = (packet_buf[16] & 0xff) | ((packet_buf[15] << 8) & 0xff00);
	*idata_6 = m_data6;
	int m_data7 = (packet_buf[18] & 0xff) | ((packet_buf[17] << 8) & 0xff00);
	*idata_7 = m_data7;
	int m_data8 = (packet_buf[20] & 0xff) | ((packet_buf[19] << 8) & 0xff00);
	*idata_8 = m_data8;
	int m_data9 = (packet_buf[22] & 0xff) | ((packet_buf[21] << 8) & 0xff00);
	*idata_9 = m_data9;
	int m_data10 = (packet_buf[24] & 0xff) | ((packet_buf[23] << 8) & 0xff00);
	*idata_10 = m_data10;
	int m_data11 = (packet_buf[26] & 0xff) | ((packet_buf[25] << 8) & 0xff00);
	*idata_11 = m_data11;
	return ret;
}

int  power_adc_read(int fd, int *iADB_0, int *iADB_1, int *iADB_2, int *iADB_3,
							int *iADB_4, int *iADB_5, int *iADB_6, int *iADB_7 )
{
	unsigned char packet_buf[255] = {STX, 'P','A', ETX};
	int ret;
	if(!fd) return -1;
	packet_buf[4] = make_lrc(&packet_buf[1], 3);
	ret = write(fd, packet_buf, 5);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response2(fd, packet_buf);
	if(packet_buf[1] == 0x02) //Packet Error Pass..
	{
		return -1;
	}

	*iADB_0 = (packet_buf[4] & 0xff) | ((packet_buf[3] << 8) & 0xff00);
	*iADB_1 = (packet_buf[6] & 0xff) | ((packet_buf[5] << 8) & 0xff00);
	*iADB_2 = (packet_buf[8] & 0xff) | ((packet_buf[7] << 8) & 0xff00);
	*iADB_3 = (packet_buf[10] & 0xff) | ((packet_buf[9] << 8) & 0xff00);
	*iADB_4 = (packet_buf[12] & 0xff) | ((packet_buf[11] << 8) & 0xff00);
	*iADB_5 = (packet_buf[14] & 0xff) | ((packet_buf[13] << 8) & 0xff00);
	*iADB_6 = (packet_buf[16] & 0xff) | ((packet_buf[15] << 8) & 0xff00);
	*iADB_7 = (packet_buf[18] & 0xff) | ((packet_buf[17] << 8) & 0xff00);

	return ret;
}

int  power_version_read(int fd, int *idata_0, int *idata_1, int *idata_2, int *idata_3,int *idata_4, int *idata_5, int *idata_6, int *idata_7) 
{
	unsigned char packet_buf[255] = {STX, 'V','R', ETX};
	int ret;
	if(!fd) return -1;
	packet_buf[4] = make_lrc(&packet_buf[1], 3);
	ret = write(fd, packet_buf, 5);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response2(fd, packet_buf);
	if(packet_buf[1] == 0x02) //Packet Error Pass..
	{
		return -1;
	}

	int m_data0 = (packet_buf[4] & 0xff) | ((packet_buf[3] << 8) & 0xff00);
	*idata_0 = m_data0;
	int m_data1 = (packet_buf[6] & 0xff) | ((packet_buf[5] << 8) & 0xff00);
	*idata_1 = m_data1;
	int m_data2 = (packet_buf[8] & 0xff) | ((packet_buf[7] << 8) & 0xff00);
	*idata_2 = m_data2;
	int m_data3 = (packet_buf[10] & 0xff) | ((packet_buf[9] << 8) & 0xff00);
	*idata_3 = m_data3;
	int m_data4 = (packet_buf[12] & 0xff) | ((packet_buf[11] << 8) & 0xff00);
	*idata_4 = m_data4;
	int m_data5 = (packet_buf[14] & 0xff) | ((packet_buf[13] << 8) & 0xff00);
	*idata_5 = m_data5;
	int m_data6 = (packet_buf[16] & 0xff) | ((packet_buf[15] << 8) & 0xff00);
	*idata_6 = m_data6;
	int m_data7 = (packet_buf[18] & 0xff) | ((packet_buf[17] << 8) & 0xff00);
	*idata_7 = m_data7;
	
	return ret;
}

int  power_sonar_read(int fd, int *idata_0, int *idata_1, int *idata_2, int *idata_3,int *idata_4, int *idata_5) 
{
	unsigned char packet_buf[255] = {STX, 'P','N', ETX};
	int ret;
	if(!fd) return -1;
	packet_buf[4] = make_lrc(&packet_buf[1], 3);
	ret = write(fd, packet_buf, 5);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response2(fd, packet_buf);
	if(packet_buf[1] == 0x02) //Packet Error Pass..
	{
		return -1;
	}

	int m_data0 = (packet_buf[4] & 0xff) | ((packet_buf[3] << 8) & 0xff00);
	*idata_0 = m_data0;
	int m_data1 = (packet_buf[6] & 0xff) | ((packet_buf[5] << 8) & 0xff00);
	*idata_1 = m_data1;
	int m_data2 = (packet_buf[8] & 0xff) | ((packet_buf[7] << 8) & 0xff00);
	*idata_2 = m_data2;
	int m_data3 = (packet_buf[10] & 0xff) | ((packet_buf[9] << 8) & 0xff00);
	*idata_3 = m_data3;
	int m_data4 = (packet_buf[12] & 0xff) | ((packet_buf[11] << 8) & 0xff00);
	*idata_4 = m_data4;
	int m_data5 = (packet_buf[14] & 0xff) | ((packet_buf[13] << 8) & 0xff00);
	*idata_5 = m_data5;
		
	return ret;
}

int power_flash_write(int fd)
{
	int ret;
	unsigned char packet_buf[255] = {STX, 'P', 'F', ETX};
	if(!fd) return -1;

	packet_buf[4] = make_lrc(&packet_buf[1], 3);

	ret = write(fd, packet_buf, 5);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);

	return ret;
}

int  power_parameter_write(int fd, int id, int data)
{
	int ret;
	int index;
	unsigned char packet_buf[255] = {STX, 'P', 'W'};
	unsigned char packet_buf2[255] = {STX, 'P', 'F', ETX};

	if(!fd) return -1;

	index = 3;
	ret = int2str(&packet_buf[index], id);
	if(id == 0)
	{
		ret = 1;
	}
	index += ret;
	packet_buf[index] = ';';
	index++;

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
	if(ret <= 0) return -2;
	
	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);
	if(ret) return -1;

	packet_buf2[4] = make_lrc(&packet_buf2[1], 3);
	ret = write(fd, packet_buf2, 5);
	if(ret <= 0) return -2;

	
	memset(packet_buf2, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf2);
	if(packet_buf2[1] == 0x30) ret = 0;

	if(ret) return -1;
	return ret;
}

int  conveyor_parameter_write(int fd, int id, int data)
{
	int ret;
	int index;
	unsigned char packet_buf[255] = {STX, 'C', 'W'};
	unsigned char packet_buf2[255] = {STX, 'C', 'F', ETX};

	if(!fd) return -1;

	index = 3;
	ret = int2str(&packet_buf[index], id);
	if(id == 0)
	{
		ret = 1;
	}
	index += ret;
	packet_buf[index] = ';';
	index++;

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
	    
	if(ret <= 0) return -2;
		
	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);
	
	    
    if(ret) return -1;

	packet_buf2[4] = make_lrc(&packet_buf2[1], 3);
	ret = write(fd, packet_buf2, 5);
	if(ret <= 0) return -2;

	
	memset(packet_buf2, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf2);
	if(packet_buf2[1] == 0x30) ret = 0;

	if(ret) return -1;

	return ret;
}

int  power_parameter_read(int fd, int *idata_0, int *idata_1, int *idata_2, int *idata_3,int *idata_4, int *idata_5, int *idata_6, int *idata_7, 
                          int *idata_8, int *idata_9, int *idata_10, int *idata_11, int *idata_12, int *idata_13, int *idata_14, int *idata_15, 
                          int *idata_16,int *idata_17, int *idata_18, int *idata_19, int *idata_20, int *idata_21, int *idata_22, int *idata_23, 
                          int *idata_24, int *idata_25, int *idata_26, int *idata_27, int *idata_28, int *idata_29, int *idata_30, int *idata_31)
{
	unsigned char packet_buf[255] = {STX, 'P','R', ETX};
	int ret;
	if(!fd) return -1;
	packet_buf[4] = make_lrc(&packet_buf[1], 3);
	ret = write(fd, packet_buf, 5);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response2(fd, packet_buf);
	if(packet_buf[1] == 0x02) //Packet Error Pass..
	{
		return -1;
	}

	int m_data0 = (packet_buf[4] & 0xff) | ((packet_buf[3] << 8) & 0xff00);
	*idata_0 = m_data0;
	int m_data1 = (packet_buf[6] & 0xff) | ((packet_buf[5] << 8) & 0xff00);
	*idata_1 = m_data1;
	int m_data2 = (packet_buf[8] & 0xff) | ((packet_buf[7] << 8) & 0xff00);
	*idata_2 = m_data2;
	int m_data3 = (packet_buf[10] & 0xff) | ((packet_buf[9] << 8) & 0xff00);
	*idata_3 = m_data3;
	int m_data4 = (packet_buf[12] & 0xff) | ((packet_buf[11] << 8) & 0xff00);
	*idata_4 = m_data4;
	int m_data5 = (packet_buf[14] & 0xff) | ((packet_buf[13] << 8) & 0xff00);
	*idata_5 = m_data5;
	int m_data6 = (packet_buf[16] & 0xff) | ((packet_buf[15] << 8) & 0xff00);
	*idata_6 = m_data6;
	int m_data7 = (packet_buf[18] & 0xff) | ((packet_buf[17] << 8) & 0xff00);
	*idata_7 = m_data7;
	int m_data8 = (packet_buf[20] & 0xff) | ((packet_buf[19] << 8) & 0xff00);
	*idata_8 = m_data8;
	int m_data9 = (packet_buf[22] & 0xff) | ((packet_buf[21] << 8) & 0xff00);
	*idata_9 = m_data9;
	int m_data10 = (packet_buf[24] & 0xff) | ((packet_buf[23] << 8) & 0xff00);
	*idata_10 = m_data10;
	int m_data11 = (packet_buf[26] & 0xff) | ((packet_buf[25] << 8) & 0xff00);
	*idata_11 = m_data11;
	int m_data12 = (packet_buf[28] & 0xff) | ((packet_buf[27] << 8) & 0xff00);
	*idata_12 = m_data12;
	int m_data13 = (packet_buf[30] & 0xff) | ((packet_buf[29] << 8) & 0xff00);
	*idata_13 = m_data13;
	int m_data14 = (packet_buf[32] & 0xff) | ((packet_buf[31] << 8) & 0xff00);
	*idata_14 = m_data14;
	int m_data15 = (packet_buf[34] & 0xff) | ((packet_buf[33] << 8) & 0xff00);
	*idata_15 = m_data15;
	int m_data16 = (packet_buf[36] & 0xff) | ((packet_buf[35] << 8) & 0xff00);
	*idata_16 = m_data16;
	int m_data17 = (packet_buf[38] & 0xff) | ((packet_buf[37] << 8) & 0xff00);
	*idata_17 = m_data17;
	int m_data18 = (packet_buf[40] & 0xff) | ((packet_buf[39] << 8) & 0xff00);
	*idata_18 = m_data18;
	int m_data19 = (packet_buf[42] & 0xff) | ((packet_buf[41] << 8) & 0xff00);
	*idata_19 = m_data19;
	int m_data20 = (packet_buf[44] & 0xff) | ((packet_buf[43] << 8) & 0xff00);
	*idata_20 = m_data20;
	int m_data21 = (packet_buf[46] & 0xff) | ((packet_buf[45] << 8) & 0xff00);
	*idata_21 = m_data21;
	int m_data22 = (packet_buf[48] & 0xff) | ((packet_buf[47] << 8) & 0xff00);
	*idata_22 = m_data22;
	int m_data23 = (packet_buf[50] & 0xff) | ((packet_buf[49] << 8) & 0xff00);
	*idata_23 = m_data23;
	int m_data24 = (packet_buf[52] & 0xff) | ((packet_buf[51] << 8) & 0xff00);
	*idata_24 = m_data24;
	int m_data25 = (packet_buf[54] & 0xff) | ((packet_buf[53] << 8) & 0xff00);
	*idata_25 = m_data25;
	int m_data26 = (packet_buf[56] & 0xff) | ((packet_buf[55] << 8) & 0xff00);
	*idata_26 = m_data26;
	int m_data27 = (packet_buf[58] & 0xff) | ((packet_buf[57] << 8) & 0xff00);
	*idata_27 = m_data27;
	int m_data28 = (packet_buf[60] & 0xff) | ((packet_buf[59] << 8) & 0xff00);
	*idata_28 = m_data28;
	int m_data29 = (packet_buf[62] & 0xff) | ((packet_buf[61] << 8) & 0xff00);
	*idata_29 = m_data29;
	int m_data30 = (packet_buf[64] & 0xff) | ((packet_buf[63] << 8) & 0xff00);
	*idata_30 = m_data30;
	int m_data31 = (packet_buf[66] & 0xff) | ((packet_buf[65] << 8) & 0xff00);
	*idata_31 = m_data31;
	
	return ret;
}

int  conveyor_parameter_read(int fd, int *idata_0, int *idata_1, int *idata_2, int *idata_3,int *idata_4, int *idata_5, int *idata_6, int *idata_7, 
                          int *idata_8, int *idata_9, int *idata_10, int *idata_11, int *idata_12, int *idata_13, int *idata_14, int *idata_15, 
                          int *idata_16,int *idata_17, int *idata_18, int *idata_19,int *idata_20)
{
	unsigned char packet_buf[255] = {STX, 'C','R', ETX};
	int ret;
	if(!fd) return -1;
	packet_buf[4] = make_lrc(&packet_buf[1], 3);
	ret = write(fd, packet_buf, 5);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response2(fd, packet_buf);
	if(packet_buf[1] == 0x02) //Packet Error Pass..
	{
		return -1;
	}

	int m_data0 = (packet_buf[4] & 0xff) | ((packet_buf[3] << 8) & 0xff00);
	*idata_0 = m_data0;
	int m_data1 = (packet_buf[6] & 0xff) | ((packet_buf[5] << 8) & 0xff00);
	*idata_1 = m_data1;
	int m_data2 = (packet_buf[8] & 0xff) | ((packet_buf[7] << 8) & 0xff00);
	*idata_2 = m_data2;
	int m_data3 = (packet_buf[10] & 0xff) | ((packet_buf[9] << 8) & 0xff00);
	*idata_3 = m_data3;
	int m_data4 = (packet_buf[12] & 0xff) | ((packet_buf[11] << 8) & 0xff00);
	*idata_4 = m_data4;
	int m_data5 = (packet_buf[14] & 0xff) | ((packet_buf[13] << 8) & 0xff00);
	*idata_5 = m_data5;
	int m_data6 = (packet_buf[16] & 0xff) | ((packet_buf[15] << 8) & 0xff00);
	*idata_6 = m_data6;
	int m_data7 = (packet_buf[18] & 0xff) | ((packet_buf[17] << 8) & 0xff00);
	*idata_7 = m_data7;
	int m_data8 = (packet_buf[20] & 0xff) | ((packet_buf[19] << 8) & 0xff00);
	*idata_8 = m_data8;
	int m_data9 = (packet_buf[22] & 0xff) | ((packet_buf[21] << 8) & 0xff00);
	*idata_9 = m_data9;
	int m_data10 = (packet_buf[24] & 0xff) | ((packet_buf[23] << 8) & 0xff00);
	*idata_10 = m_data10;
	int m_data11 = (packet_buf[26] & 0xff) | ((packet_buf[25] << 8) & 0xff00);
	*idata_11 = m_data11;
	int m_data12 = (packet_buf[28] & 0xff) | ((packet_buf[27] << 8) & 0xff00);
	*idata_12 = m_data12;
	int m_data13 = (packet_buf[30] & 0xff) | ((packet_buf[29] << 8) & 0xff00);
	*idata_13 = m_data13;
	int m_data14 = (packet_buf[32] & 0xff) | ((packet_buf[31] << 8) & 0xff00);
	*idata_14 = m_data14;
	int m_data15 = (packet_buf[34] & 0xff) | ((packet_buf[33] << 8) & 0xff00);
	*idata_15 = m_data15;
	int m_data16 = (packet_buf[36] & 0xff) | ((packet_buf[35] << 8) & 0xff00);
	*idata_16 = m_data16;
	int m_data17 = (packet_buf[38] & 0xff) | ((packet_buf[37] << 8) & 0xff00);
	*idata_17 = m_data17;
	int m_data18 = (packet_buf[40] & 0xff) | ((packet_buf[39] << 8) & 0xff00);
	*idata_18 = m_data18;
	int m_data19 = (packet_buf[42] & 0xff) | ((packet_buf[41] << 8) & 0xff00);
	*idata_19 = m_data19;
	int m_data20 = (packet_buf[44] & 0xff) | ((packet_buf[43] << 8) & 0xff00);
	*idata_20 = m_data20;

	return ret;
}

int  conveyor_data_read(int fd, int *idata_0, int *idata_1, int *idata_2, int *idata_3,int *idata_4, int *idata_5, int *idata_6, int *idata_7, 
                          int *idata_8, int *idata_9, int *idata_10, int *idata_11, int *idata_12, int *idata_13)
{
	unsigned char packet_buf[255] = {STX, 'C','D', ETX};
	int ret;
	if(!fd) return -1;
	packet_buf[4] = make_lrc(&packet_buf[1], 3);
	ret = write(fd, packet_buf, 5);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response2(fd, packet_buf);
	if(packet_buf[1] == 0x02) //Packet Error Pass..
	{
		return -1;
	}

	int m_data0 = (packet_buf[4] & 0xff) | ((packet_buf[3] << 8) & 0xff00);
	*idata_0 = m_data0;
	int m_data1 = (packet_buf[6] & 0xff) | ((packet_buf[5] << 8) & 0xff00);
	*idata_1 = m_data1;
	int m_data2 = (packet_buf[8] & 0xff) | ((packet_buf[7] << 8) & 0xff00);
	*idata_2 = m_data2;
	int m_data3 = (packet_buf[10] & 0xff) | ((packet_buf[9] << 8) & 0xff00);
	*idata_3 = m_data3;
	int m_data4 = (packet_buf[12] & 0xff) | ((packet_buf[11] << 8) & 0xff00);
	*idata_4 = m_data4;
	int m_data5 = (packet_buf[14] & 0xff) | ((packet_buf[13] << 8) & 0xff00);
	*idata_5 = m_data5;
	int m_data6 = (packet_buf[16] & 0xff) | ((packet_buf[15] << 8) & 0xff00);
	*idata_6 = m_data6;
	int m_data7 = (packet_buf[18] & 0xff) | ((packet_buf[17] << 8) & 0xff00);
	*idata_7 = m_data7;
	int m_data8 = (packet_buf[20] & 0xff) | ((packet_buf[19] << 8) & 0xff00);
	*idata_8 = m_data8;
	int m_data9 = (packet_buf[22] & 0xff) | ((packet_buf[21] << 8) & 0xff00);
	*idata_9 = m_data9;
	int m_data10 = (packet_buf[24] & 0xff) | ((packet_buf[23] << 8) & 0xff00);
	*idata_10 = m_data10;
	int m_data11 = (packet_buf[26] & 0xff) | ((packet_buf[25] << 8) & 0xff00);
	*idata_11 = m_data11;
	int m_data12 = (packet_buf[28] & 0xff) | ((packet_buf[27] << 8) & 0xff00);
	*idata_12 = m_data12;
	int m_data13 = (packet_buf[30] & 0xff) | ((packet_buf[29] << 8) & 0xff00);
	*idata_13 = m_data13;
		
	return ret;
}


int power_charging_ready(int fd, int on)
{
	int ret;
	unsigned char packet_buf[255] = {STX, 'P', 'C'};
	
	if(!fd) return -1;

	if(on == 1)//On
	{
		packet_buf[3] = '1';
	}
	else//Off
	{
		packet_buf[3] = '0';
	}
	packet_buf[4] = ETX;
	packet_buf[5] = make_lrc(&packet_buf[1], 4);

	ret = write(fd, packet_buf, 6);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);
	return ret;
}

///Add Function //////////////////////////////////////////////////////////////////////////////
int power_read_Integral_Voltage(int fd, int *V_value)
{
	unsigned char packet_buf[1024] = {STX, 'I','V', ETX};
	int ret;
	int index = 0;
	int lrc_val=0;

	if(!fd) return -1;
	packet_buf[4] = make_lrc(&packet_buf[1], 3);
	ret = write(fd, packet_buf, 5);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*1024);
	ret = get_response2(fd, packet_buf);
	if(ret <= 0) return -3;

	for(int i=0, j=0; i< 500; i++, j+= 2)
	{
		V_value[i] = (packet_buf[4+j] & 0xff) | ((packet_buf[3+j] << 8) & 0xff00);
		//printf("V_value[%d]: %d \n",i,V_value[i]);
	}


	return ret;
}

int power_read_Integral_Current(int fd, int *I_value)
{
	unsigned char packet_buf[1024] = {STX, 'I','C', ETX};
	int ret;

	if(!fd) return -1;
	packet_buf[4] = make_lrc(&packet_buf[1], 3);
	ret = write(fd, packet_buf, 5);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*1024);
	ret = get_response2(fd, packet_buf);
	if(packet_buf[1] == 0x02) //Packet Error Pass..
	{
		return -1;
	}

	for(int i=0, j=0; i< 500; i++, j+= 2)
	{
		I_value[i] = (packet_buf[4+j] & 0xff) | ((packet_buf[3+j] << 8) & 0xff00);
		//printf("I_value[%d]: %d \n",i,I_value[i]);
	}


	return ret;
}


// Function to convert binary to decimal
int binaryToDecimal(long binarynum)
{
    int decimalnum = 0, temp = 0, remainder;
    while (binarynum!=0)
    {
        remainder = binarynum % 10;
        binarynum = binarynum / 10;
        decimalnum = decimalnum + remainder*pow(2,temp);
        temp++;
    }
    return decimalnum;
}

///GPIO//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Digital Output Control//
int power_set_OutputPort(int fd, int port0, int port1, int port2, int port3, int port4, int port5, int port6, int port7)
{
	int ret;
	unsigned char packet_buf[255] = { STX, 'P', 'O' };
	char charValue[10];
	sprintf(charValue, "%d%d%d%d%d%d%d%d", port7,port6,port5,port4,port3,port2,port1,port0);

	packet_buf[3] = (char)binaryToDecimal(atol(charValue))+'0';
	packet_buf[4] = ETX;
	packet_buf[5] = make_lrc(&packet_buf[1], 4);

	ret = write(fd, packet_buf, 6);
	if (ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char) * 255);
	ret = get_response(fd, packet_buf);

	return ret;
}

///POWER ENABLE//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//POWER Control//
int power_set_Enable(int fd, int port0, int port1, int port2, int port3, int port4, int port5, int port6, int port7)
{
	int ret;
	unsigned char packet_buf[255] = { STX, 'P', 'E' };
	char charValue[10];
	sprintf(charValue, "%d%d%d%d%d%d%d%d", port7,port6,port5,port4,port3,port2,port1,port0);

	packet_buf[3] = (char)binaryToDecimal(atol(charValue))+'0';
	packet_buf[4] = ETX;
	packet_buf[5] = make_lrc(&packet_buf[1], 4);

	ret = write(fd, packet_buf, 6);
	if (ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char) * 255);
	ret = get_response(fd, packet_buf);

	return ret;
}

int power_wheel_enable(int fd, int mode)
{
	int ret;
	unsigned char packet_buf[255] = {STX, 'P', 'M'};
	
	if(!fd) return -1;

	if(mode == 1) //Start 
	{
		packet_buf[3] = '1';
	}
	else //Stop
	{
		packet_buf[3] = '0';
	}
	packet_buf[4] = ETX;
	packet_buf[5] = make_lrc(&packet_buf[1], 4);

	ret = write(fd, packet_buf, 6);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);
	//printf("packet_buf[1]: %02x \n", packet_buf[1]);
	return ret;
}


//Digital Input & Output Read//
int power_read_GPIO(int fd,  int *Input, int *Output)
{
	int  Input_binary[8] = {0,};
	int Output_binary[8] = {0,};

	unsigned char packet_buf2[255] = {STX, 'P','I', ETX};
	int ret;
	if(!fd) return -1;
	packet_buf2[4] = make_lrc(&packet_buf2[1], 3);
	ret = write(fd, packet_buf2, 5);
	if(ret <= 0) return -2;

	memset(packet_buf2, 0, sizeof(unsigned char)*255);
	ret = get_response2(fd, packet_buf2);
	if(packet_buf2[1] == 0x02) //Packet Error Pass..
	{
		return -1;
	}


	int iInput = (packet_buf2[4] & 0xff) | ((packet_buf2[3] << 8) & 0xff00);
	decimal2binary(iInput, Input_binary);
	int iOutput = (packet_buf2[6] & 0xff) | ((packet_buf2[5] << 8) & 0xff00);
	decimal2binary(iOutput, Output_binary);

	for(int i=0; i<8; i++)
	{
		Input[i]  = Input_binary[i];
		Output[i] = Output_binary[i];
	}


	return ret;
}

///CONVEYOR//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Conveyor Function//
int power_conveyor_movement(int fd, int mode)
{
	int ret;
	unsigned char packet_buf[255] = {STX, 'C', 'N'};
	
	if(!fd) return -1;

	if(mode == 1) //Start 
	{
		packet_buf[3] = '1';
	}
	else //Stop
	{
		packet_buf[3] = '0';
	}
	packet_buf[4] = ETX;
	packet_buf[5] = make_lrc(&packet_buf[1], 4);

	ret = write(fd, packet_buf, 6);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);
	//printf("packet_buf[1]: %02x \n", packet_buf[1]);
	return ret;
}

int  power_read_loadcell(int fd,  double *weight)
{
	unsigned char packet_buf[255] = {STX, 'C','L', ETX};
	int ret;
	if(!fd) return -1;

	packet_buf[4] = make_lrc(&packet_buf[1], 3);

	ret = write(fd, packet_buf, 5);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response2(fd, packet_buf);
	if(packet_buf[1] == 0x02) //Packet Error Pass..
	{
		return -1;
	}

	
	int _mWeight = (packet_buf[4] & 0xff) | ((packet_buf[3] << 8) & 0xff00);
	*weight = _mWeight / 10.0;
	

	return ret;
}

int power_loadcell_callibration(int fd)
{
	int ret;
	unsigned char packet_buf[255] = {STX, 'C', 'A', ETX};
	if(!fd) return -1;

	packet_buf[4] = make_lrc(&packet_buf[1], 3);

	ret = write(fd, packet_buf, 5);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);

	return ret;
}

int power_read_conveyor_sensor(int fd,  int *Input)
{
	unsigned char packet_buf[255] = {STX, 'C','P', ETX};
	int ret;
	if(!fd) return -1;
	packet_buf[4] = make_lrc(&packet_buf[1], 3);
	ret = write(fd, packet_buf, 5);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);


	int iInput = (packet_buf[4] & 0xff) | ((packet_buf[3] << 8) & 0xff00);
	*Input = iInput;

	return ret;
}

int power_conveyor_manual_movement(int fd, int on)
{
	int ret;
	unsigned char packet_buf[255] = {STX, 'C', 'C'};
	
	if(!fd) return -1;

	if(on == 1) //CW 
	{
		packet_buf[3] = '1';
	}
	else if(on == 2)//CCW
	{
		packet_buf[3] = '2';
	}
	else //Stop
	{
		packet_buf[3] = '0';
	}
	packet_buf[4] = ETX;
	packet_buf[5] = make_lrc(&packet_buf[1], 4);

	ret = write(fd, packet_buf, 6);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf);
	return ret;
}

int power_read_conveyor_movement(int fd, int* result)
{
	unsigned char packet_buf[255] = { STX, 'C','S', ETX };
	int ret;
	if (!fd) return -1;
	packet_buf[4] = make_lrc(&packet_buf[1], 3);
	ret = write(fd, packet_buf, 5);
	if (ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char) * 255);
	ret = get_response(fd, packet_buf);


	int iInput = packet_buf[4] & 0x07;
	*result = iInput;

	return ret;
}
