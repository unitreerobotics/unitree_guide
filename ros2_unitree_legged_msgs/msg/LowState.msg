
uint8[2] head
uint8 level_flag
uint8 frame_reserve

uint32[2] sn
uint32[2] version
uint16 band_width
IMU imu
MotorState[20] motor_state
BmsState bms
int16[4] foot_force	
int16[4] foot_force_est	
uint32 tick						
uint8[40] wireless_remote 
uint32 reserve

uint32 crc
