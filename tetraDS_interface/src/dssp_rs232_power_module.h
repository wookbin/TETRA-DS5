int  dssp_rs232_power_module_create(const char *device, int time_out);
void dssp_rs232_power_module_destroy();
int  dssp_rs232_power_module_read_battery(double *dbattery, double *dVoltage, double *dCurrent, int *mode_status, int *Input, int *Output);
int  dssp_rs232_power_module_read_tetra(double *dbattery, double *dVoltage, double *dCurrent, int *mode_status, int *Input, int *Output, double *Ultrasonic);
int  dssp_rs232_power_module_read_bumper(int *bumper_data);
int  dssp_rs232_power_module_set_Ultrasonic(int mode);
int  dssp_rs232_power_module_read_Ultrasonic(double * Ultrasonic);
int  dssp_rs232_power_module_adc_read(int *iADB_0, int *iADB_1, int *iADB_2, int *iADB_3, int *iADB_4, int *iADB_5, int *iADB_6, int *iADB_7);
int  dssp_rs232_power_module_set_light(int id, int brightness);
int  dssp_rs232_power_module_toggle_on(int id);
int  dssp_rs232_power_module_set_light_toggle(int index, int lightAcc1, int H_brightness, int lightAcc2, int L_brightness);
int  dssp_rs232_power_module_flash_write();
int  dssp_rs232_power_module_parameter_write(int id, int data);
int  dssp_rs232_conveyor_module_parameter_write(int id, int data);
int  dssp_rs232_power_module_parameter_read(int *idata_0, int *idata_1, int *idata_2, int *idata_3, int *idata_4, int *idata_5, int *idata_6, 
                                            int *idata_7, int *idata_8, int *idata_9, int *idata_10, int *idata_11, int *idata_12, int *idata_13,											 
                                            int *idata_14, int *idata_15, int *idata_16, int *idata_17, int *idata_18, int *idata_19, int *idata_20, 
                                            int *idata_21, int *idata_22, int *idata_23, int *idata_24,int *idata_25, int *idata_26, int *idata_27, 
                                            int *idata_28, int *idata_29, int *idata_30,int *idata_31);
int  dssp_rs232_conveyor_module_parameter_read(int *idata_0, int *idata_1, int *idata_2, int *idata_3, int *idata_4, int *idata_5, int *idata_6, 
                                            int *idata_7, int *idata_8, int *idata_9, int *idata_10, int *idata_11, int *idata_12, int *idata_13,											 
                                            int *idata_14, int *idata_15, int *idata_16, int *idata_17, int *idata_18, int *idata_19,int *idata_20);   
int  dssp_rs232_conveyor_module_data_read(int *idata_0, int *idata_1, int *idata_2, int *idata_3, int *idata_4, int *idata_5, int *idata_6, 
                                            int *idata_7, int *idata_8, int *idata_9, int *idata_10, int *idata_11, int *idata_12, int *idata_13);
int  dssp_rs232_power_module_data_read(int *idata_0, int *idata_1, int *idata_2, int *idata_3, int *idata_4, int *idata_5, int *idata_6, 
                                            int *idata_7, int *idata_8, int *idata_9, int *idata_10, int *idata_11);    
int  dssp_rs232_power_module_version_read(int *idata_0, int *idata_1, int *idata_2, int *idata_3,int *idata_4, int *idata_5, int *idata_6, int *idata_7);
int  dssp_rs232_power_module_sonar_read(int *idata_0, int *idata_1, int *idata_2, int *idata_3,int *idata_4, int *idata_5);                                                                                                                               
int  dssp_rs232_power_module_set_charging_ready(int on);
//sampling read
int  dssp_rs232_power_module_read_Voltage(int *Voltage_data);
int  dssp_rs232_power_module_read_Current(int *Current_data);
//GPIO
int  dssp_rs232_power_module_set_OutputPort(int port0, int port1, int port2, int port3, int port4, int port5, int port6, int port7);
int  dssp_rs232_power_module_set_Single_OutputPort(int id, int value);
int  dssp_rs232_power_module_get_GPIO(int *Input, int *Output);
//POWER ENABLE
int  dssp_rs232_power_module_set_Enable(int port0, int port1, int port2, int port3, int port4, int port5, int port6, int port7);
int  dssp_rs232_power_module_set_Single_Enable(int id, int value);
int  dssp_rs232_power_module_wheel_enable(int on);
//Conveyor
int  dssp_rs232_power_module_conveyor_movement(int start);
int  dssp_rs232_power_module_read_loadcell(double *weight);
int  dssp_rs232_power_module_loadcell_callibration();
int  dssp_rs232_power_module_read_conveyor_sensor(int *Input);
int  dssp_rs232_power_module_conveyor_manual_movement(int on);
int  dssp_rs232_power_module_read_conveyor_movement(int* result);