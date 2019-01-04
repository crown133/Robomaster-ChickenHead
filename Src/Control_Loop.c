#include "Control_Loop.h"
#include "Remote_Ctrl.h"
#include "Motor_Func.h"
#include "Can_Ctrl.h"
#include "sys.h"
float pitchPos;
float yawPos;
float BodanSpeed;

void sysControl(void)
{
	if(RemoteCtrlFlag)
	{
		/************* Yaw Axis Pos Change ************/
		yawPos = CH0Radio * (RemoteCtrlData.remote.ch0 - 1024);
		Motor_IncPos(&(motorYaw.posCtrl), yawPos, 5000, 1700);
		
		/************* Pitch Axis Pos Change ************/
		pitchPos = CH1Radio * (RemoteCtrlData.remote.ch1 - 1024);
		Motor_IncPos(&(motorPitch.posCtrl), pitchPos, 150000, -100000);
		
		/*************** Bodan Speed Set ***************/
		BodanSpeed = CH3Radio * (RemoteCtrlData.remote.ch3 - 1024);
		Motor_SetVel(&(motorBodan.veloCtrl), BodanSpeed);
		
	
	}
	RemoteCtrlFlag = 0;
}
