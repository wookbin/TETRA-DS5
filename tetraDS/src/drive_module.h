
#include "rs232.h"

/*
*******************************************************************************
*                             FUNCTION PROTOTYPES
*******************************************************************************
*/

int drvm_init(const char *port, int time_out);
void drvm_deinit(int fd);
int drvm_set_servo(int fd, int mode);
int drvm_set_velocity_mode(int fd);
int drvm_set_position_mode(int fd);
int drvm_set_drive_err_reset(int fd);
int drvm_set_odometry(int fd, int x, int y, int theta);
int drvm_set_velocity(int fd, int left, int right);
int drvm_set_position(int fd, int type, int pass, int data);
int drvm_set_parameter(int fd, int param, int value);
int drvm_read_bumper_emg(int fd, int *bumper, int *emg_state, int *left_error_code, int *right_error_code);
int drvm_read_drive_err(int fd, unsigned char *left_wheel_err, unsigned char *right_wheel_err);
int drvm_read_encoder(int fd, int *left, int *right);
int drvm_read_parameter(int fd, int param, int *value);
int drvm_read_odometry(int fd, double *Xpos_mm, double *Ypos_mm, double *deg);
int drvm_set_Charge_On(int fd);
int drvm_set_Charge_Off(int fd);
int drvm_set_velocity2(int fd, int left, int right, int *x, int *y, int *theta, int *bumper, int *emg);
