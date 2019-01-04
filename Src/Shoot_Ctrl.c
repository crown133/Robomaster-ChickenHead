#include "Shoot_Ctrl.h"
#include "Remote_Ctrl.h"
#include "tim.h"

int setVelo;
int fire1, fire2, fireFlag = 0;

float halAVelo[3], halBVelo[3];

float shootVeloA, shootVeloB;

void ShootWheel_Control(void)
{
	if(RemoteCtrlData.remote.s1 == FireFire)
	{
		setVelo = 1500;
		fire1 = 3;
		if(fire1 != fire2)
		{
			fireFlag = 1;
		}
		fire2 = fire1;
	}
	else if(RemoteCtrlData.remote.s1 == FireWeak)
	{
		setVelo = 1300;
		fire1 = 2;
		if(fire1 != fire2)
		{
			fireFlag = 1;
		}
		fire2 = fire1;
	}
	else
	{
		fire1 = 1;
		if(fire1 != fire2)
		{
			fireFlag = 1;
		}
		fire2 = fire1;
		setVelo = 1000;
	}
	if(fireFlag)
	{
		for(int i = 1000; i <= setVelo; i++)
		{
			MocaWheelA = i;
			MocaWheelB = i;
			HAL_Delay(1);
		}
		fireFlag = 0;
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
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
			shootVeloA = 1000000 / halAVelo[2] / 7 * 60;
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
			shootVeloB = 1000000 / halBVelo[2] / 7 * 60;
		}
	}
}

