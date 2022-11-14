#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "rs232_common.h"

unsigned long mypow(int i, int j)
{
	unsigned long result = 1;
	int k;

	for(k = 0; k < j; k++)
		result *= i;

	return result;
}

void str2int(unsigned char data[], unsigned int dist[])
{
	int i, j, k, index;
	char tmp[10];

	i = j = k = index = 0;
	while(1)
	{
		if('0' <= data[i] && data[i] <= '9')
		{
			tmp[j++] = data[i];
		}
		else if(data[i] == ';' || data[i] == ETX)
		{
			for(k = 0; k < j; k++)
			{
				dist[index] += (tmp[k] - '0') * mypow(10, j - k - 1);
			}
			j = 0;
			index++;
		}
		
		if(data[i] == ETX)
		{
			break;
		}
		i++;
	}
}

int int2str(unsigned char data[], int num)
{
	int index = 0, i, j;
	unsigned char flag = 0;

	if(num < 0)
	{
		data[index++] = '-';
		num = -1 * num;
	}
	data[index] = '0';

	for(i = 9; i >= 0; i--)
	{
		j = num / mypow(10, i);
		num -= j * mypow(10, i);
		if(j > 0 || flag == 1)
		{
			data[index++] = (unsigned char)(j + '0');
			flag = 1;
		}
	}

	return index;
}

void decimal2binary(int decimal, int binary[])
{
	int i=0;

	while(decimal > 0) {
        binary[i] = decimal%2;
        decimal = decimal/2;
        i++;
    }
}

unsigned char make_lrc(unsigned char data[], int num)
{
	unsigned char tmp = 0;
	int i;

	for(i = 0; i < num; i++)
		tmp ^= data[i];

	return tmp;
}

void display_error_message(unsigned char flag)
{
	switch(flag)
	{
		case FLAG_ERROR :
			printf("Protocol Error\n");
			break;
		case FLAG_FAIL :
			printf("Proccessing Fail\n");
			break;
		case FLAG_NO :
			printf("NO Proccessing\n");
			break;
		case FLAG_END :
			printf("End of Packet\n");
			break;
		case FLAG_PASS :
			printf("Pass through\n");
			break;
		case FLAG_COMPLETE :
			printf("Motion Complete\n");
			break;
		default :
			printf("Unknown Error\n");
			break;
	}
}

int get_response(int fd, unsigned char data[])
{
	int index;
	int ret = 0;

	do
	{
		ret = read(fd, data, 1);
		if(ret <= 0) return -1;
	} while(!(data[0] == STX));
	index = 1;
	
	do
	{
		ret = read(fd, &data[index], 1);
		index++;
		if(ret <= 0) return -1;
	} while(data[index - 2] != ETX);
	
	if(data[1] != FLAG_OK)
		return -1;
	else if(data[index-1] != make_lrc(&data[1], index - 2)) {
		return -1;
	}
	else {
		return 0;
	}
}

int get_response2(int fd, unsigned char data[])
{
	int index = 0;
	int ret = 0;
	int lrc_val = 0;
	int total_byte = 0;
	unsigned char tmp = 0;
	// receive STX
	do {
		if( read(fd, data, 1) < 1) 
		{
			printf("return -10 \n");
			return -10;
		}
	} while(data[0] != STX);	

	// receive data total byte check
	read(fd, &data[1], 1);
	read(fd, &data[2], 1);

	total_byte = (data[2] & 0xff) | ((data[1] << 8)& 0xff00);
	index = 3;
	do {
		ret = read(fd, &data[index], 1);
		index++;
		if(!ret) 
		{
			return -11;
		}

	} 
	while(index < total_byte + 5);

	//calc lrc...
	for(int j=0; j<total_byte; j++)
	{
		tmp ^= data[j+3];
	}
	//lrc check
	if(data[index-1] != tmp)
	{
		printf(" lrc check error!!! \n");
		for(int i=0; i<index; i++)
		{
			printf("data[%d]: %02x\n",i,data[i] );
		}
		printf("LRC : %02x\n",tmp);
		return -1;
	}
}
