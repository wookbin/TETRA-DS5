
#include <stdio.h>
#include <malloc.h>
#include "rs232.h"
#include "dssp_rs232_drive_module.h"
#include "drive_module.h"

int _drv = 0;

int dssp_rs232_drv_module_create(const char *device, int time_out)
{
	if(_drv != 0) {
		drvm_deinit(_drv);
		_drv = 0;
	}

	_drv = drvm_init(device, time_out);
	
	if(_drv == 0) return -1;
	else return 0;
}

void dssp_rs232_drv_module_destroy()
{
	if(_drv != 0) {
		drvm_deinit(_drv);
		_drv = 0;
	}
}

int  dssp_rs232_drv_module_read_encoder(int *encoder_l, int *encoder_r)
{
	int ret;

	ret = drvm_read_encoder(_drv, encoder_l, encoder_r);

	return ret;
}

int  dssp_rs232_drv_module_read_odometry(double *Xpos_mm, double *Ypos_mm, double *deg)
{
	int ret;

	ret = drvm_read_odometry(_drv, Xpos_mm, Ypos_mm, deg);

	return ret;
}
	
int  dssp_rs232_drv_module_read_bumper_emg(int *bumper_data, int *emg_state, int *left_error_code, int *right_error_code)
{
	int ret;

	ret = drvm_read_bumper_emg(_drv, bumper_data, emg_state, left_error_code, right_error_code);

	return ret;
}

int  dssp_rs232_drv_module_read_drive_err(unsigned char *left_wheel_err, unsigned char *right_wheel_err)
{
	int ret;

	ret = drvm_read_drive_err(_drv, left_wheel_err, right_wheel_err);

	return ret;
}

int  dssp_rs232_drv_module_set_servo(int mode)
{
	int ret;

	//ret = drvm_set_velocity_mode(_drv);
	ret = drvm_set_servo(_drv, mode);
	if(!ret) return ret;

	return ret;
}

int  dssp_rs232_drv_module_set_drive_err_reset()
{
	int ret;

	ret = drvm_set_drive_err_reset(_drv);

	return ret;
}

int  dssp_rs232_drv_module_set_velocity(int velocity_l, int velocity_r)
{
	int ret;

	ret = drvm_set_velocity(_drv, velocity_l, velocity_r);

	return ret;
}

int  dssp_rs232_drv_module_set_position(int type, int pass, int data)
{
	int ret;

	ret = drvm_set_position(_drv, type, pass, data);

	return ret;
}

int  dssp_rs232_drv_module_set_parameter(int param, int value)
{
	int ret;

	ret = drvm_set_parameter(_drv, param, value);
	printf("param num: %d, data: %d \n", param, value);

	return ret;
}

int  dssp_rs232_drv_module_read_parameter(int param, int *value)
{
	int ret;

	ret = drvm_read_parameter(_drv, param, value);

	return ret;
}

int  dssp_rs232_drv_module_reset_odometry()
{
	int ret;

	ret = drvm_set_odometry(_drv, 0, 0, 0);

	return ret;

}

int  dssp_rs232_drv_module_set_velocitymode()
{
	int ret;

	ret = drvm_set_velocity_mode(_drv);

	return ret;

}

int  dssp_rs232_drv_module_set_positionmode()
{
	int ret;

	ret = drvm_set_position_mode(_drv);

	return ret;

}

int  dssp_rs232_drv_module_set_charge(int mode)
{
	int ret;

	if(mode == 0)
	{
		ret = drvm_set_Charge_On(_drv);
	}
	else
	{
		ret = drvm_set_Charge_Off(_drv);
	}
	
	if(!ret) return ret;

	return ret;
}

int  dssp_rs232_drv_module_set_velocity2(int velocity_l, int velocity_r, int *Xpos_mm, int *Ypos_mm, int *deg, int *bumper, int *emg)
{
	int ret;

	ret = drvm_set_velocity2(_drv, velocity_l, velocity_r, Xpos_mm, Ypos_mm, deg, bumper, emg);

	return ret;
}
