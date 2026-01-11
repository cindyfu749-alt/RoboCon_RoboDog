
#include "bsp_imu.h"
#include "pid.h"
//imu_Rx_data[0]加速度--imu_Rx_data[0][0]==
//imu_Rx_data[1]角速度--imu_Rx_data[0][1]==X
//imu_Rx_data[2]角度----imu_Rx_data[0][2]==Y
//imu_Rx_data[3]磁场----imu_Rx_data[0][3]==Z
float imu_Rx_data[4][4]= {0};
	
//pid-IMU-陀螺仪-X-方向---------------------------------------------------------
extern const fp32 imu_Z_PID_data[3] ;//P,I,D值
//pid-IMU-陀螺仪-X-方向
extern float imu_Z_PID_OUT;
/*
 * 函数名：IMU
 * 描述  ：CAN的GPIO 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void imu_Rx_data_fun(uint8_t id)
{
	switch(id)
	{
		case 0x51 :
			
						   break;
		case 0x52 :
						   break;
		case 0x53 :
						   break;
		case 0x54 :
						   break;

	}
}
void IMU_PID_Init(void)
{
	//--------------------------------------------------- pid最大输出
	//pid-IMU-陀螺仪-X-方向
	PID_init(&imu_Z_PID, PID_POSITION,imu_Z_PID_data, 3.744, 1);
}
//IMU-陀螺仪-X-方向--pid计算
void IMU_pid_count(float set_Z)
{
	//IMU-陀螺仪-X-pid-1-计算-----------------反馈--------目标值---
	imu_Z_PID_OUT = PID_calc(&imu_Z_PID, imu_Rx_data[2][3],set_Z);
  
}

/**************************END OF FILE************************************/
