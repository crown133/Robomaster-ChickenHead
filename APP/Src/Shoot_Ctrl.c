#include "Shoot_Ctrl.h"
#include "Remote_Ctrl.h"
#include "tim.h"
#include "sys.h"
#include "delay.h"


#define BeThree(x) ((x)>2 ? 0:(x))  //鼠标右键摩擦轮换挡
int mouseR = 0, mouseFlag = 0;

uint16_t setVelo = 1000;


float halAVelo[3], halBVelo[3];

float shootVeloA[6] = {0}, shootVeloB[6] = {0};  //最后一位是平均值
int countA = 0, countB = 0;    //均值滤波计数用


/********** 摩擦轮pid参数初始化 ********/
void MocaWheelInit(VeloPidCtrl_t *motor, float p, float i, float d, float outmax, float outmin)
{
	motor->kp = p;
	motor->ki = i;
	motor->kd = d;
	motor->outputMax = outmax;
	motor->outputMin = outmin;
}

/*********** 摩擦轮控制函数 ************/
void ShootWheel_Control(void)
{

//	if(RemoteCtrlData.remote.s2 == FireFire)
//	{
//		setVelo = 1300;
//		fire1 = 3;
//		if(fire1 != fire2)
//		{
//			fireFlag = 1;
//		}
//		fire2 = fire1;
//	}
//	else if(RemoteCtrlData.remote.s2 == FireWeak)
//	{
//		setVelo = 1250;
//		fire1 = 2;
//		if(fire1 != fire2)
//		{
//			fireFlag = 1;
//		}
//		fire2 = fire1;
//	}
//	else
//	{
//		fire1 = 1;
//		if(fire1 != fire2)
//		{
//			fireFlag = 1;
//		}
//		fire2 = fire1;
//		setVelo = 1000;
//	}
//	if(fireFlag)
//	{
//		for(int i = 1000; i <= setVelo; i++)
//		{
//			MocaWheelA = i;
//			MocaWheelB = i;
//			HAL_Delay(1);
//		}
//		fireFlag = 0;
//	}

//	if(RemoteCtrlData.mouse.press_r == 1) //鼠标右键摩擦轮换挡
//	{
//		mouseR++;
//		mouseR = BeThree(mouseR);
//		RemoteCtrlData.mouse.press_r = 0;
//	}

	{  //电脑Q W E切换摩擦轮转速
		if(RemoteCtrlData.key.Q == 1)
		{
			mouseR = 0;
		}
		if(RemoteCtrlData.key.W == 1)
		{
			mouseR = 1;
		}
		if(RemoteCtrlData.key.E == 1)
		{
			mouseR = 2;
		}
	}

	if(((RemoteCtrlData.remote.s2 == FireFire) || (mouseR == 2)))// && (setVelo >= 1245))
	{
		if(setVelo < 1400 )// && (velo_delay1 == 0)) 
		{
			setVelo += 1;
			delay_ms(1);
		}
		
//		velo_delay1++;
//		
//		if(velo_delay1 > velo_delay0)
//		{
//			velo_delay1 = 0;
//		}
		
		PGout(13) = 1;
		
	}
	else if(RemoteCtrlData.remote.s2 == FireWeak || (mouseR == 1))
	{
		if(setVelo < 1250)// && (velo_delay1 == 0))
		{
			setVelo += 1;
			delay_ms(1);
		}
//		velo_delay1++;
//		
//		if(velo_delay1 > velo_delay0)
//		{
//			velo_delay1 = 0;
//		}
		if(setVelo > 1250)
		{
			setVelo -= 10;
		}
		PGout(13) = 1;
	}
	else
	{
		setVelo = 1000;
		PGout(13) = 1;
	}
		MocaWheelA = setVelo;
		MocaWheelB = setVelo;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM5)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			halAVelo[1] = halAVelo[0];
			halAVelo[0] = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_1);//获取上升沿时间点
			HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
			
			if(0 == halAVelo[1])
			{
				halAVelo[2] = 0;  //数组第三位存差值
			}
			else
			{
				if(halAVelo[0] > halAVelo[1])
				{
					halAVelo[2] = halAVelo[0] - halAVelo[1];
				}
				else
				{
					halAVelo[2] = halAVelo[0] + 0xffff - halAVelo[1] + 1;
				}
				shootVeloA[countA++] = 1000000 / halAVelo[2] / 7 * 60;  //周期与实际转速转换公式，7对极，故一圈7个周期(单位 r/min)
				shootVeloA[5] = (shootVeloA[0] + shootVeloA[1] + shootVeloA[2] + shootVeloA[3] + shootVeloA[4]) / 5;
				if(countA == 5)
				{
					countA = 0;
				}
			}
		}
		
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			halBVelo[1] = halBVelo[0];
			halBVelo[0] = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_2);//获取上升沿时间点
			HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
			
			if(0 == halBVelo[1])
			{
				halBVelo[2] = 0;  //数组第三位存差值
			}
			else
			{
				if(halBVelo[0] > halBVelo[1])
					{
						halBVelo[2] = halBVelo[0] - halBVelo[1];
					}
					else
					{
						halBVelo[2] = halBVelo[0] + 0xffff - halBVelo[1] + 1;
					}
				shootVeloB[countB++] = 1000000 / halBVelo[2] / 7 * 60; //周期与实际转速转换公式，7对极，故一圈7个周期
				shootVeloB[5] = (shootVeloB[0] + shootVeloB[1] + shootVeloB[2] + shootVeloB[3] + shootVeloB[4]) / 5;
				if(countB == 5)
				{
					countB = 0;
				}
			}
		}
	}
}

