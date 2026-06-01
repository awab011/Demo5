std_msgs/Header header # Ros standard message header

#Robot localization 

float32 x
float32 y
float32 z
float32 yaw # lidar IMU

# KFS Grid map 
#  ________________
#  | 1  | 2  | 3  |  # 1 being the entry point same as the 2026 appendix
#  | 4  | 5  | 6  | 
#  | 7  | 8  | 9  | 
#  | 10 | 11 | 12 |  
#  ________________
# 0: empty, 1: R1, 2: R2, 9: Fake 

int8[12] forest

# Current block R2 is on 
int8 current_block

# R1 status
float32 r1_x
float32 r1_y

string r1_position
string r1_command

#System status
bool lidar_ok
bool camera_ok
bool end