
#include <stdio.h>
#include <malloc.h>
#include "rs232.h"
#include "dssp_rs232_power_module.h"
#include "power_module.h"

int _power = 0;

int dssp_rs232_power_module_create(const char *device, int time_out)
{
	if(_power != 0) {
		power_deinit(_power);
		_power = 0;
	}

	_power = power_init(device, time_out);
	
	if(_power == 0) return -1;
	else return 0;
}

void dssp_rs232_power_module_destroy()
{
	if(_power != 0) {
		power_deinit(_power);
		_power = 0;
	}
}
	
int  dssp_rs232_power_module_read_bumper(int *bumper_data)
{
	int ret;

	ret = power_read_bumper(_power, bumper_data);

	return ret;
}

int  dssp_rs232_power_module_read_battery(double *dbattery, double *dVoltage, double *dCurrent, int *mode_status, int *Input, int *Output)
{
	int ret;

	ret = power_read_Battery(_power, dbattery, dVoltage, dCurrent, mode_status, Input, Output);

	return ret;
}

int  dssp_rs232_power_module_read_tetra(double *dbattery, double *dVoltage, double *dCurrent, int *mode_status, int *Input, int *Output, double *Ultrasonic)
{
	int ret;

	ret = power_read_tetra(_power, dbattery, dVoltage, dCurrent, mode_status, Input, Output, Ultrasonic);

	return ret;
}

int  dssp_rs232_power_module_set_Ultrasonic(int mode)
{
	int ret;

	if(mode == 1) //On
	{
		ret = power_set_Ultrasonic(_power, 1);
	}
	else // Off
	{
		ret = power_set_Ultrasonic(_power, 0);
	}

	return ret;
}

int  dssp_rs232_power_module_read_Ultrasonic(double * Ultrasonic)
{
	int ret;

	ret = power_read_Ultrasonic(_power, Ultrasonic);

	return ret;
}



int  dssp_rs232_power_module_set_light(int id, int brightness)
{
	int ret;

	ret = power_set_light(_power, id, brightness);
	
	return ret;
}

int  dssp_rs232_power_module_toggle_on(int id)
{
	int ret;

	ret = power_toggle_on(_power, id);
	
	return ret;
}

int  dssp_rs232_power_module_set_light_toggle(int index, int lightAcc1, int H_brightness, int lightAcc2, int L_brightness)
{
	int ret;

	ret = power_set_light_toggle(_power, index, lightAcc1, H_brightness, lightAcc2, L_brightness);
	
	return ret;
}

int  dssp_rs232_power_module_flash_write()
{
	int ret;

	ret = power_flash_write(_power);
	
	return ret;
}

int  dssp_rs232_power_module_parameter_write(int id, int data)
{
	int ret;

	ret = power_parameter_write(_power, id, data);
	
	return ret;
}

int  dssp_rs232_conveyor_module_parameter_write(int id, int data)
{
	int ret;

	ret = conveyor_parameter_write(_power, id, data);
	
	return ret;
}

int  dssp_rs232_power_module_parameter_read(int *idata_0, int *idata_1, int *idata_2, int *idata_3,int *idata_4, int *idata_5, int *idata_6, 
                                            int *idata_7, int *idata_8, int *idata_9, int *idata_10, int *idata_11, int *idata_12,int *idata_13,											 
											int *idata_14, int *idata_15, int *idata_16, int *idata_17, int *idata_18,
											int *idata_19, int *idata_20, int *idata_21, int *idata_22, int *idata_23, int *idata_24,int *idata_25,
											int *idata_26, int *idata_27, int *idata_28, int *idata_29, int *idata_30,int *idata_31)
{
	int ret;

	ret = power_parameter_read(_power, idata_0, idata_1, idata_2, idata_3, idata_4, idata_5, idata_6, idata_7, idata_8, idata_9, idata_10, idata_11, idata_12,idata_13, idata_14, 
	idata_15, idata_16, idata_17, idata_18, idata_19, idata_20, idata_21, idata_22, idata_23, idata_24, idata_25, idata_26, idata_27, idata_28, idata_29, idata_30, idata_31);
	
	return ret;
}

int  dssp_rs232_conveyor_module_parameter_read(int *idata_0, int *idata_1, int *idata_2, int *idata_3,int *idata_4, int *idata_5, int *idata_6, 
                                            int *idata_7, int *idata_8, int *idata_9, int *idata_10, int *idata_11, int *idata_12,int *idata_13,											 
											int *idata_14, int *idata_15, int *idata_16, int *idata_17, int *idata_18, int *idata_19,int *idata_20)
{
	int ret;

	ret = conveyor_parameter_read(_power, idata_0, idata_1, idata_2, idata_3, idata_4, idata_5, idata_6, idata_7, idata_8, idata_9, idata_10, idata_11, idata_12,idata_13, idata_14, 
	idata_15, idata_16, idata_17, idata_18, idata_19, idata_20);
	
	return ret;
}

int  dssp_rs232_conveyor_module_data_read(int *idata_0, int *idata_1, int *idata_2, int *idata_3,int *idata_4, int *idata_5, int *idata_6, 
                                            int *idata_7, int *idata_8, int *idata_9, int *idata_10, int *idata_11, int *idata_12,int *idata_13)
{
	int ret;

	ret = conveyor_data_read(_power, idata_0, idata_1, idata_2, idata_3, idata_4, idata_5, idata_6, idata_7, idata_8, idata_9, idata_10, idata_11, idata_12, idata_13);
	
	return ret;
}

int  dssp_rs232_power_module_data_read(int *idata_0, int *idata_1, int *idata_2, int *idata_3,int *idata_4, int *idata_5, int *idata_6, 
                                            int *idata_7, int *idata_8, int *idata_9, int *idata_10, int *idata_11)
{
	int ret;

	ret = power_data_read(_power, idata_0, idata_1, idata_2, idata_3, idata_4, idata_5, idata_6, idata_7, idata_8, idata_9, idata_10, idata_11);
	
	return ret;
}

int  dssp_rs232_power_module_version_read(int *idata_0, int *idata_1, int *idata_2, int *idata_3,int *idata_4, int *idata_5, int *idata_6, int *idata_7) 
{
	int ret;

	ret = power_version_read(_power, idata_0, idata_1, idata_2, idata_3, idata_4, idata_5, idata_6, idata_7);
	
	return ret;
}

int  dssp_rs232_power_module_sonar_read(int *idata_0, int *idata_1, int *idata_2, int *idata_3,int *idata_4, int *idata_5) 
{
	int ret;

	ret = power_sonar_read(_power, idata_0, idata_1, idata_2, idata_3, idata_4, idata_5);
	
	return ret;
}

int  dssp_rs232_power_module_adc_read(int *iADB_0, int *iADB_1, int *iADB_2, int *iADB_3, int *iADB_4, int *iADB_5, int *iADB_6, int *iADB_7)
{
	int ret;

	ret = power_adc_read(_power, iADB_0, iADB_1, iADB_2, iADB_3, iADB_4, iADB_5, iADB_6, iADB_7);

	return ret;
}

int  dssp_rs232_power_module_set_charging_ready(int on)
{
	int ret;

	if(on == 1) //On
	{
		ret = power_charging_ready(_power, 1);
	}
	else // Off
	{
		ret = power_charging_ready(_power, 0);
	}

	return ret;
}

int  dssp_rs232_power_module_read_Voltage(int *Voltage_data)
{
	int ret;

	ret = power_read_Integral_Voltage(_power, Voltage_data);

	return ret;
}

int  dssp_rs232_power_module_read_Current(int *Current_data)
{
	int ret;

	ret = power_read_Integral_Current(_power, Current_data);

	return ret;
}
     
int  dssp_rs232_power_module_set_OutputPort(int port0, int port1, int port2, int port3, int port4, int port5, int port6, int port7)
{
	int ret;

	ret = power_set_OutputPort(_power, port0,port1,port2,port3,port4,port5,port6,port7);

	return ret;
}

int  dssp_rs232_power_module_set_Single_OutputPort(int id, int value)
{
	int ret;

	ret = power_set_Single_OutputPort(_power, id, value);

	return ret;
}

int  dssp_rs232_power_module_set_Enable(int port0, int port1, int port2, int port3, int port4, int port5, int port6, int port7)
{
	int ret;

	ret = power_set_Enable(_power, port0,port1,port2,port3,port4,port5,port6,port7);

	return ret;
}

int  dssp_rs232_power_module_set_Single_Enable(int id, int value)
{
	int ret;

	ret = power_set_Single_Enable(_power, id, value);

	return ret;
}


int  dssp_rs232_power_module_get_GPIO(int *Input, int *Output)
{
	int ret;

	ret = power_read_GPIO(_power, Input, Output);

	return ret;
}

//Conveyor_Function//
int  dssp_rs232_power_module_conveyor_movement(int start)
{
	int ret;

	if(start == 1) //start
	{
		ret = power_conveyor_movement(_power, 1);
	}
	else // Stop
	{
		ret = power_conveyor_movement(_power, 0);
	}

	return ret;
}

int  dssp_rs232_power_module_wheel_enable(int on)
{
	int ret;

	if(on == 1) //start
	{
		ret = power_wheel_enable(_power, 1);
	}
	else // Stop
	{
		ret = power_wheel_enable(_power, 0);
	}

	return ret;
}

int  dssp_rs232_power_module_read_loadcell(double *weight)
{
	int ret;

	ret = power_read_loadcell(_power, weight);

	return ret;
}

int  dssp_rs232_power_module_loadcell_callibration()
{
	int ret;

	ret = power_loadcell_callibration(_power);
	
	return ret;
}

int  dssp_rs232_power_module_read_conveyor_sensor(int *Input)
{
	int ret;

	ret = power_read_conveyor_sensor(_power, Input);

	return ret;
}

int  dssp_rs232_power_module_conveyor_manual_movement(int on)
{
	int ret;

	ret = power_conveyor_manual_movement(_power, on);

	return ret;
}

int  dssp_rs232_power_module_read_conveyor_movement(int* result)
{
	int ret;

	ret = power_read_conveyor_movement(_power, result);

	return ret;
}