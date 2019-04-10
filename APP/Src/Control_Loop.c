#include "Control_Loop.h"
#include "Remote_Ctrl.h"
#include "Motor_Ctrl.h"
#include "Can_Ctrl.h"
#include "Pc_Uart.h"
#include "Shoot_Ctrl.h"
#include "imu.h"

#include "delay.h"
#include "mpu6500driver_middleware.h"
#include "mpu6500reg.h"

#include "sys.h"


#define Yaw_Limit(x, num)	( (x) < (num) ? (1) : (0) )

float pitchPos;
float yawPos;
float BodanSpeed;

int fireFlag = 0;  //鼠标发射标志位
uint16_t mpuFifoCount = 0;  //mpu6500 FIFO 寄存器缓存数据个数

/*********************************/
void sysControl(void)
{
	if(RemoteCtrlFlag)
	{
		if(!initFlag)  //IMU反馈闭环模式
		{
			/************* Yaw Axis Pos Change ************/
			motorYaw.veloCtrl.rawVel = JYgyro.gyroVeloz;
			motorYaw.posCtrl.rawPosLast = motorYaw.posCtrl.rawPos;
			motorYaw.posCtrl.rawPos = JYgyro.gyroAnglez - motorYaw.posCtrl.posBias;
			
			if((motorYaw.posCtrl.rawPos - motorYaw.posCtrl.rawPosLast) > 200)  //Yaw轴角度相对位置 因为IMU Yaw轴会有漂移，长时间使用可能会转过360°，发生跳变
			{
				motorYaw.posCtrl.round--;
			}
			else if((motorYaw.posCtrl.rawPos - motorYaw.posCtrl.rawPosLast) < -200)
			{
				motorYaw.posCtrl.round++;
			}
			motorYaw.posCtrl.relaPos = motorYaw.posCtrl.rawPos + 360*motorYaw.posCtrl.round;
			
			yawPos = CH0Radio * (RemoteCtrlData.remote.ch0 - 1024) - RemoteCtrlData.mouse.x*0.01;//遥控器和鼠标控制
			
			//云台运动超出电机编码器范围 计算偏差 加在参考值上来达到限位的目的
			motorYaw.posCtrl.motorBias = (Yaw_Limit(6000, motorYaw.posCtrl.motorPos)*(motorYaw.posCtrl.motorPos - 6000) \
										+ Yaw_Limit(motorYaw.posCtrl.motorPos, 3600)*(motorYaw.posCtrl.motorPos - 3600))/10;
			
			//电机在IMU模式下 遥控值限位
			Motor_AbsPos(&(motorYaw.posCtrl), yawPos, 380, -60);
			
			/************* Pitch Axis Pos Change ************/
//			motorPitch.veloCtrl.rawVel = -JYgyro.gyroVeloy;
			motorPitch.posCtrl.rawPos = JYgyro.gyroAngley;
			
			pitchPos = CH1Radio * (RemoteCtrlData.remote.ch1 - 1024) - RemoteCtrlData.mouse.y*0.01;
			//电机在IMU模式下 遥控值限位
			Motor_AbsPos(&(motorPitch.posCtrl), pitchPos, 185, 145);
			
			/*************** 视觉辅助 *************/
			if(RemoteCtrlData.remote.s1 == 3)  //视觉辅助开关
			{
//				motorYaw.posCtrl.motorBias = -yawInc;
//				TD_Calculate(&tdYawPc, -yawInc);
				Motor_AbsPos(&(motorYaw.posCtrl), -yawInc/90, 350, 5);
//				Motor_AbsPos(&(motorPitch.posCtrl), -pitchInc/120, 182, 145);
			}
			
		}
		else  //电机编码器闭环模式
		{
			/************* Yaw Axis Pos Change ************/
			yawPos = CH0Radio * (RemoteCtrlData.remote.figWheel - 1024);
			Motor_IncPos(&(motorYaw.posCtrl), yawPos, 7000, 3500);

			/************* Pitch Axis Pos Change ************/
			pitchPos = CH1Radio * (RemoteCtrlData.remote.ch1 - 1024);
			Motor_IncPos(&(motorPitch.posCtrl), pitchPos, 150000, -100000);
		}

		/*************** Bodan Speed Set ***************/
		BodanSpeed = CH3Radio * (RemoteCtrlData.remote.ch3 - 1024);
		Motor_SetVel(&(motorBodan.veloCtrl), BodanSpeed);
		
		{  //鼠标左键发射弹丸
			if(RemoteCtrlData.mouse.press_l == 1)
			{
				fireFlag = 1;
			}
			
			if(fireFlag)
			{
				Motor_SetVel(&(motorBodan.veloCtrl), 4500);
				fireFlag ++;
			}
			if(fireFlag >= 80)
			{
				fireFlag = 0;
			}
		}

		Gimbal_Control();  //电机控制函数
		
	}

//	mpuFifoCount = mpu6500_read_single_reg(MPU_FIFO_COUNTH);
//	delay_us(100);
//	mpuFifoCount = ((mpuFifoCount<<8) | mpu6500_read_single_reg(MPU_FIFO_COUNTL));
	
	//	CtrlDebug(motorPitch.posCtrl.rawPos, motorPitch.posCtrl.output, td2.v1, td2.v2, 0, 0, 0, 0, 0, 0);  //发送调试信息到串口
	
}
