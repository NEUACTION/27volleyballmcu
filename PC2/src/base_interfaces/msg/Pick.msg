#开始发送信息
bool start  
#视觉程序自己的手动开关
bool depth
#
float32 distance
#有几个平面
int32 plane_num

#平面1的质心坐标和夹角（平面法向量与相机光轴）
float32 plane1_centroid_x
float32 plane1_centroid_y
float32 plane1_centroid_z
float32 plane1_angle
int32 plane1_pointnum  #平面点的个数

float32 plane2_centroid_x
float32 plane2_centroid_y
float32 plane2_centroid_z
float32 plane2_angle
int32 plane2_pointnum