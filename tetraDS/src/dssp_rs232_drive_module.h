
int  dssp_rs232_drv_module_create(const char *device, int time_out);
void dssp_rs232_drv_module_destroy();
int  dssp_rs232_drv_module_read_encoder(int *encoder_l, int *encoder_r);	
int  dssp_rs232_drv_module_read_bumper_emg(int *bumper_data, int *emg_state, int *left_error_code, int *right_error_code);
int  dssp_rs232_drv_module_read_drive_err(unsigned char *left_wheel_err, unsigned char *right_wheel_err);
int  dssp_rs232_drv_module_read_parameter(int param, int *value);
int  dssp_rs232_drv_module_set_servo(int mode);
int  dssp_rs232_drv_module_set_drive_err_reset();	
int  dssp_rs232_drv_module_set_velocity(int velocity_l, int velocity_r);
int  dssp_rs232_drv_module_set_position(int type, int pass, int data);
int  dssp_rs232_drv_module_set_parameter(int param, int value);
int  dssp_rs232_drv_module_read_odometry(double *Xpos_mm, double *Ypos_mm, double *deg);	
int  dssp_rs232_drv_module_reset_odometry();	
int  dssp_rs232_drv_module_set_velocitymode();
int  dssp_rs232_drv_module_set_positionmode();
int  dssp_rs232_drv_module_set_charge(int mode);
int  dssp_rs232_drv_module_set_velocity2(int velocity_l, int velocity_r, int *Xpos_mm, int *Ypos_mm, int *deg, int *bumper, int *emg);
