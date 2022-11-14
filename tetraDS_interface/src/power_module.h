#include "rs232.h"

typedef struct DATA
{
    double _dBattery_voltage;
    double _dSystem_current;
    double _dCharge_current;
    double _dCharge_signal;
    double _dCharge_voltage;
    double _dPower1_voltage;
    double _dPower2_voltage;
    double _dPower3_voltage;
    int    _iSBumper_0;
    int    _iSBumper_1;
    int    _iSBumper_2;
    int    _iSBumper_3;
    int    _iSBumper_4;
    int    _iSBumper_5;
    int    _iSBumper_6;
    int    _iSBumper_7;
    double _dUltrasonic_0_distance;
    double _dUltrasonic_1_distance;
    double _dUltrasonic_2_distance;
    double _dUltrasonic_3_distance;
    double _dUltrasonic_4_distance;
    double _dUltrasonic_5_distance;
    double _dUltrasonic_6_distance;
    double _dUltrasonic_7_distance;	
}DATA;

int  power_init(const char *port, int time_out);
void power_deinit(int fd);
int  power_read_Battery(int fd, double *dbattery, double *dVoltage, double *dCurrent, int *mode_status, int *Input, int *Output);
int  power_read_tetra(int fd, double *dbattery, double *dVoltage, double *dCurrent, int *mode_status, int *Input, int *Output, double *Ultrasonic);
int  power_read_bumper(int fd, int *bumper);
int  power_set_Ultrasonic(int fd, int mode);
int  power_read_Ultrasonic(int fd, double *Ultrasonic);
int  power_read_data_all(int fd);
int  power_adc_read(int fd, int *iADB_0, int *iADB_1, int *iADB_2, int *iADB_3, int *iADB_4, int *iADB_5, int *iADB_6, int *iADB_7);
int  power_set_light(int fd, int id, int brightness);
int  power_toggle_on(int fd, int id);
int  power_set_light_toggle(int fd, int de_index, int lightAcc1, int H_brightness, int lightAcc2, int L_brightness);
int  power_flash_write(int fd);
int  power_parameter_write(int fd, int id, int data);
int  conveyor_parameter_write(int fd, int id, int data);
int  power_parameter_read(int fd, int *idata_0, int *idata_1, int *idata_2, int *idata_3,int *idata_4, int *idata_5, int *idata_6, int *idata_7, 
                          int *idata_8, int *idata_9, int *idata_10, int *idata_11, int *idata_12, int *idata_13, int *idata_14, int *idata_15, 
                          int *idata_16,int *idata_17, int *idata_18, int *idata_19, int *idata_20, int *idata_21, int *idata_22, int *idata_23, 
                          int *idata_24, int *idata_25, int *idata_26, int *idata_27, int *idata_28, int *idata_29, int *idata_30, int *idata_31);
int  conveyor_parameter_read(int fd, int *idata_0, int *idata_1, int *idata_2, int *idata_3,int *idata_4, int *idata_5, int *idata_6, int *idata_7, 
                          int *idata_8, int *idata_9, int *idata_10, int *idata_11, int *idata_12, int *idata_13, int *idata_14, int *idata_15, 
                          int *idata_16,int *idata_17, int *idata_18, int *idata_19,int *idata_20); 
int  conveyor_data_read(int fd, int *idata_0, int *idata_1, int *idata_2, int *idata_3,int *idata_4, int *idata_5, int *idata_6, int *idata_7, 
                          int *idata_8, int *idata_9, int *idata_10, int *idata_11, int *idata_12, int *idata_13);
int  power_data_read(int fd, int *idata_0, int *idata_1, int *idata_2, int *idata_3,int *idata_4, int *idata_5, int *idata_6, int *idata_7, 
                          int *idata_8, int *idata_9,int *idata_10, int *idata_11);
int  power_version_read(int fd, int *idata_0, int *idata_1, int *idata_2, int *idata_3,int *idata_4, int *idata_5, int *idata_6, int *idata_7); 
int  power_sonar_read(int fd, int *idata_0, int *idata_1, int *idata_2, int *idata_3,int *idata_4, int *idata_5);                                                                               
int  power_charging_ready(int fd, int on);
//Power
int  power_read_Integral_Voltage(int fd, int *V_value);
int  power_read_Integral_Current(int fd, int *I_value);
//GPIO//
int  power_set_OutputPort(int fd, int port0, int port1, int port2, int port3, int port4, int port5, int port6, int port7);
int  power_set_Single_OutputPort(int fd, int id, int value);
int  power_set_Enable(int fd, int port0, int port1, int port2, int port3, int port4, int port5, int port6, int port7);
int  power_set_Single_Enable(int fd, int id, int value);
int  power_read_GPIO(int fd, int *Input, int *Output);
//Conveyor
int  power_conveyor_movement(int fd, int mode);
int  power_wheel_enable(int fd, int mode);
int  power_read_loadcell(int fd,  double *weight);
int  power_loadcell_callibration(int fd);
int  power_read_conveyor_sensor(int fd,  int *Input);
int  power_conveyor_manual_movement(int fd, int on);
int  power_read_conveyor_movement(int fd, int* result);