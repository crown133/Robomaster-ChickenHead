#include "Control_Loop.h"
#include "Remote_Ctrl.h"
#include "Motor_Ctrl.h"
#include "Can_Ctrl.h"
#include "Pc_Uart.h"
#include "Shoot_Ctrl.h"
#include "usart.h"

#include "imu.h"

#include "delay.h"
//#include "mpu6500driver_middleware.h"
//#include "mpu6500reg.h"

#include "sys.h"

#define Yaw_Limit(x, num)	( (x) < (num) ? (1) : (0) )

float pitchPos;
float yawPos;
float BodanSpeed;

float kf0 = 0.008, kf1 = 0.002, kf2 = 0.005, kf3 = 0.0008;
float sum = 0.005, sum1 = -0.009;

float kpv = 0.001, kiv = 0.002, kdv = 50, kpp = -3, kip = 0, kdp = -50;

kalman_filter_t yaw_kalman_filter, pitch_kalman_filter;
kalman_filter_t yaw_velo_kf;

unsigned int sendFlag1 = 0;  //图形绘制

extern void moto_angle_send(void);

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
			TD_Calculate(&td1_velo, motorYaw.veloCtrl.rawVel);
			motorYaw.veloCtrl.filrawVel =  td1_velo.v1;//yaw_velo_kf.filtered_value[0];

//			motorYaw.veloCtrl.rawVel = JYgyro.gyroVeloz;  //
			motorYaw.posCtrl.rawPosLast = motorYaw.posCtrl.rawPos;
//			motorYaw.posCtrl.rawPos = JYgyro.gyroAnglez - motorYaw.posCtrl.posBias;
			motorYaw.posCtrl.rawPos = JYgyro.gyroAnglez - motorYaw.posCtrl.posBias;
			
			if((motorYaw.posCtrl.rawPos - motorYaw.posCtrl.rawPosLast) > 300)  //Yaw轴角度相对位置 因为IMU Yaw轴会有漂移，长时间使用可能会转过360°，发生跳变
			{
				motorYaw.posCtrl.round--;
			}
			else if((motorYaw.posCtrl.rawPos - motorYaw.posCtrl.rawPosLast) < -300)
			{
				motorYaw.posCtrl.round++;
			}
			motorYaw.posCtrl.relaPos = motorYaw.posCtrl.rawPos + 360*motorYaw.posCtrl.round;
			
			yawPos = CH0Radio * (RemoteCtrlData.remote.ch0 - 1024) - RemoteCtrlData.mouse.x * 0.002;//遥控器和鼠标控制

			
			/*云台运动超出电机编码器范围 计算偏差 加在参考值上来达到限位的目的*/
//			motorYaw.posCtrl.motorBias = (Yaw_Limit(6000, motorYaw.posCtrl.motorPos)*(motorYaw.posCtrl.motorPos - 6000) \
//										+ Yaw_Limit(motorYaw.posCtrl.motorPos, 3600)*(motorYaw.posCtrl.motorPos - 3600))/15;
			
			Motor_AbsPos(&(motorYaw.posCtrl), yawPos, 60, -100);	//电机在IMU模式下 遥控值限位

			
			/************* Pitch Axis Pos Change ************/
			motorPitch.posCtrl.rawPos = JYgyro.gyroAngley;

//			if((mouseR != 0) || (RemoteCtrlData.remote.s2 != 1))  //防止供弹链路内有子弹，云台俯仰压缩链路
			pitchPos = CH1Radio * (RemoteCtrlData.remote.ch1 - 1024) - RemoteCtrlData.mouse.y * 0.001;
			
			Motor_AbsPos(&(motorPitch.posCtrl), pitchPos, 12, -40);//电机在IMU模式下 遥控值限位
			
			/*************** 视觉相关 *************/
			if(RemoteCtrlData.remote.s1 == 2 || RemoteCtrlData.mouse.press_r == 1)  //视觉辅助开关
			{	
				sendFlag1 = 0;
				TD_Calculate(&tdYawPc, yawInc);
				TD_Calculate(&tdPitchPc, pitchInc);
//				TD4_track4(&trackerYawInc, yawInc, 1.0f/200);
//				ESO_AngularRate_run(&eso2, yawInc, 1.0f/200);
//				eso2.z2 = 0;
//				ADRC_LESO(&eso1, yawInc);
				
//				kalman_filter_calc(&yaw_kalman_filter, yawInc, 0);
//				kalman_filter_calc(&pitch_kalman_filter, pitchInc, 0);
//				Motor_AbsPos(&motorYaw.posCtrl, -yaw_kalman_filter.filtered_value[0] * kf0 - tdYawPc.v2 * kf1 + motorYaw.veloCtrl.filrawVel * sum, 360, -360);
//				Motor_AbsPos(&motorPitch.posCtrl, -pitch_kalman_filter.filtered_value[0] * kf2 - tdPitchPc.v2 * kf3, 12, -40);
				Motor_AbsPos(&motorYaw.posCtrl, -yawInc * kf0 - tdYawPc.v2 * kf1 + motorYaw.veloCtrl.filrawVel * sum, 60, -100);
				Motor_AbsPos(&motorPitch.posCtrl, -pitchInc * kf2 - tdPitchPc.v2 * kf3 + motorPitch.veloCtrl.rawVel * sum1, 12, -40);

			}
			
			{	//视觉模式识别切换 0攻击地面单位  1攻击基地
				if(RemoteCtrlData.key.A == 1)  
				{
					UART7_TX_BUF[2] = 1;
					HAL_UART_Transmit(&huart7, UART7_TX_BUF, 3, 100);
					RemoteCtrlData.key.A = 0;
				}
				if(RemoteCtrlData.key.S == 1)  //视觉模式识别切换 0攻击地面单位  1攻击基地
				{
					UART7_TX_BUF[2] = 0;
					HAL_UART_Transmit(&huart7, UART7_TX_BUF, 3, 100);
					RemoteCtrlData.key.S = 0;
				}
			}	
		}
		else  //电机编码器闭环模式
		{
			/************* Yaw Axis Pos Change ************/
			yawPos = CH0Radio * (RemoteCtrlData.remote.figWheel - 1024);
			Motor_IncPos(&(motorYaw.posCtrl), yawPos, 7000, 2000);

			/************* Pitch Axis Pos Change ************/
			pitchPos = CH1Radio * (RemoteCtrlData.remote.ch1 - 1024);
			Motor_IncPos(&(motorPitch.posCtrl), pitchPos, 7000, 500);
		}

		/*************** Bodan Speed Set ***************/
//		if((mouseR != 0) || (RemoteCtrlData.remote.s2 != 1))  //防止未开启摩擦轮就打蛋，堵塞供弹链路
		BodanSpeed = CH3Radio * (RemoteCtrlData.remote.ch3 - 1024);
		
		Motor_SetVel(&(motorBodan.veloCtrl), BodanSpeed);
		BodanSpeed = 0;
		{  //鼠标左键发射弹丸
			if((RemoteCtrlData.mouse.press_l == 1) && ((mouseR != 0) || (RemoteCtrlData.remote.s2 != 1)))
			{
				fireFlag = 1;
			}
			
			if(fireFlag)
			{
				Motor_SetVel(&(motorBodan.veloCtrl), -7200);
				fireFlag ++;
			}
			if(fireFlag >= 50)
			{
				Motor_SetVel(&(motorBodan.veloCtrl), 0);
				fireFlag = 0;
			}
		}
		
		Gimbal_Control();  //电机控制函数
	}
	else
	{
		CAN_CMD_GIMBAL(0, 0, 0, 0);
	}
	
	//CtrlDebug(motorPitch.posCtrl.rawPos, motorPitch.posCtrl.output, td2.v1, td2.v2, 0, 0, 0, 0, 0, 0);  //发送调试信息到串口
	
}
