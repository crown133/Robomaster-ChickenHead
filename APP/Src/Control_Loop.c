#include "Control_Loop.h"
#include "Remote_Ctrl.h"
#include "Motor_Ctrl.h"
#include "Can_Ctrl.h"
#include "Pc_Uart.h"
#include "Shoot_Ctrl.h"

#include "imu.h"

#include "delay.h"
//#include "mpu6500driver_middleware.h"
//#include "mpu6500reg.h"

#include "sys.h"

#define Yaw_Limit(x, num)	( (x) < (num) ? (1) : (0) )

float pitchPos;
float yawPos;
float BodanSpeed;

float kf0 = 150, kf1 = 150;
float sum = 0.008;

kalman_filter_t yaw_kalman_filter, pitch_kalman_filter;
kalman_filter_t yaw_velo_kf;

extern void moto_angle_send(void);

int fireFlag = 0;  //��귢���־λ
uint16_t mpuFifoCount = 0;  //mpu6500 FIFO �Ĵ����������ݸ���

/*********************************/
void sysControl(void)
{
	if(RemoteCtrlFlag)
	{
		if(!initFlag)  //IMU�����ջ�ģʽ
		{
			/************* Yaw Axis Pos Change ************/
			TD_Calculate(&td1_velo, motorYaw.veloCtrl.rawVel);
			motorYaw.veloCtrl.filrawVel =  td1_velo.v1;//yaw_velo_kf.filtered_value[0];

//			motorYaw.veloCtrl.rawVel = JYgyro.gyroVeloz;  //
			motorYaw.posCtrl.rawPosLast = motorYaw.posCtrl.rawPos;
//			motorYaw.posCtrl.rawPos = JYgyro.gyroAnglez - motorYaw.posCtrl.posBias;
			motorYaw.posCtrl.rawPos = JYgyro.gyroAnglez;
			
			if((motorYaw.posCtrl.rawPos - motorYaw.posCtrl.rawPosLast) > 300)  //Yaw��Ƕ����λ�� ��ΪIMU Yaw�����Ư�ƣ���ʱ��ʹ�ÿ��ܻ�ת��360�㣬��������
			{
				motorYaw.posCtrl.round--;
			}
			else if((motorYaw.posCtrl.rawPos - motorYaw.posCtrl.rawPosLast) < -300)
			{
				motorYaw.posCtrl.round++;
			}
			motorYaw.posCtrl.relaPos = motorYaw.posCtrl.rawPos + 360*motorYaw.posCtrl.round;
			
			yawPos = CH0Radio * (RemoteCtrlData.remote.ch0 - 1024) - RemoteCtrlData.mouse.x*0.01;//ң������������

			
			/*��̨�˶����������������Χ ����ƫ�� ���ڲο�ֵ�����ﵽ��λ��Ŀ��*/
//			motorYaw.posCtrl.motorBias = (Yaw_Limit(6000, motorYaw.posCtrl.motorPos)*(motorYaw.posCtrl.motorPos - 6000) \
//										+ Yaw_Limit(motorYaw.posCtrl.motorPos, 3600)*(motorYaw.posCtrl.motorPos - 3600))/15;
			
			Motor_AbsPos(&(motorYaw.posCtrl), yawPos, 200, -200);	//�����IMUģʽ�� ң��ֵ��λ

			
			/************* Pitch Axis Pos Change ************/
			motorPitch.posCtrl.rawPos = JYgyro.gyroAngley;

//			if((mouseR != 0) || (RemoteCtrlData.remote.s2 != 1))  //��ֹ������·�����ӵ�����̨����ѹ����·
			pitchPos = CH1Radio * (RemoteCtrlData.remote.ch1 - 1024) - RemoteCtrlData.mouse.y*0.01;
			
			Motor_AbsPos(&(motorPitch.posCtrl), pitchPos, 20, -35);//�����IMUģʽ�� ң��ֵ��λ
			
			/*************** �Ӿ����� *************/
			if(RemoteCtrlData.remote.s1 == 2)  //�Ӿ���������
			{				
				TD_Calculate(&tdYawPc, yawInc);
				TD_Calculate(&tdPitchPc, pitchInc);
//				TD4_track4(&trackerYawInc, yawInc, 1.0f/200);
//				ESO_AngularRate_run(&eso2, yawInc, 1.0f/200);
//				eso2.z2 = 0;
//				sum = (trackerYawInc.x1/kf0 + eso2.z2 * kf1) * 100; 
//				ADRC_LESO(&eso1, yawInc);
				kalman_filter_calc(&yaw_kalman_filter, yawInc, 0);
				kalman_filter_calc(&pitch_kalman_filter, pitchInc, 0);
				
				Motor_AbsPos(&motorYaw.posCtrl, -yaw_kalman_filter.filtered_value[0]/150 - tdYawPc.v2 * 0.004, 360, -360);
				Motor_AbsPos(&motorPitch.posCtrl, -pitch_kalman_filter.filtered_value[0]/150 - tdPitchPc.v2 * 0.005, 20, -35);
				
				kf0 = pitch_kalman_filter.filtered_value[0];   //�Ƕ�
				
			}

			if(RemoteCtrlData.remote.s1 == 3)
			{
				TD_Calculate(&tdPitchPc, pitchInc);
				kalman_filter_calc(&pitch_kalman_filter, pitchInc, 0);
				Motor_AbsPos(&motorPitch.posCtrl, -pitch_kalman_filter.filtered_value[0]/kf1 - tdPitchPc.v2 * sum, 20, -35);
				kf0 = pitch_kalman_filter.filtered_value[0];   //�Ƕ�
			}
	
		}
		else  //����������ջ�ģʽ
		{
			/************* Yaw Axis Pos Change ************/
			yawPos = CH0Radio * (RemoteCtrlData.remote.figWheel - 1024);
			Motor_IncPos(&(motorYaw.posCtrl), yawPos, 7000, 3500);

			/************* Pitch Axis Pos Change ************/
			pitchPos = CH1Radio * (RemoteCtrlData.remote.ch1 - 1024);
			Motor_IncPos(&(motorPitch.posCtrl), pitchPos, 7000, 500);
		}

		/*************** Bodan Speed Set ***************/
//		if((mouseR != 0) || (RemoteCtrlData.remote.s2 != 1))  //��ֹδ����Ħ���־ʹ򵰣�����������·
		BodanSpeed = CH3Radio * (RemoteCtrlData.remote.ch3 - 1024);
		
		Motor_SetVel(&(motorBodan.veloCtrl), BodanSpeed);
		BodanSpeed = 0;
		{  //���������䵯��
			if((RemoteCtrlData.mouse.press_l == 1) && ((mouseR != 0) || (RemoteCtrlData.remote.s2 != 1)))
			{
				fireFlag = 1;
			}
			
			if(fireFlag)
			{
				Motor_SetVel(&(motorBodan.veloCtrl), -6600);
				fireFlag ++;
			}
			if(fireFlag >= 50)
			{
				Motor_SetVel(&(motorBodan.veloCtrl), 0);
				fireFlag = 0;
			}
		}
		
		Gimbal_Control();  //������ƺ���
	}
	else
	{
		CAN_CMD_GIMBAL(0, 0, 0, 0);
	}
	
	//CtrlDebug(motorPitch.posCtrl.rawPos, motorPitch.posCtrl.output, td2.v1, td2.v2, 0, 0, 0, 0, 0, 0);  //���͵�����Ϣ������
	
}
