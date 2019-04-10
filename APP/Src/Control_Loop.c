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
			motorYaw.veloCtrl.rawVel = JYgyro.gyroVeloz;
			motorYaw.posCtrl.rawPosLast = motorYaw.posCtrl.rawPos;
			motorYaw.posCtrl.rawPos = JYgyro.gyroAnglez - motorYaw.posCtrl.posBias;
			
			if((motorYaw.posCtrl.rawPos - motorYaw.posCtrl.rawPosLast) > 200)  //Yaw��Ƕ����λ�� ��ΪIMU Yaw�����Ư�ƣ���ʱ��ʹ�ÿ��ܻ�ת��360�㣬��������
			{
				motorYaw.posCtrl.round--;
			}
			else if((motorYaw.posCtrl.rawPos - motorYaw.posCtrl.rawPosLast) < -200)
			{
				motorYaw.posCtrl.round++;
			}
			motorYaw.posCtrl.relaPos = motorYaw.posCtrl.rawPos + 360*motorYaw.posCtrl.round;
			
			yawPos = CH0Radio * (RemoteCtrlData.remote.ch0 - 1024) - RemoteCtrlData.mouse.x*0.01;//ң������������
			
			//��̨�˶����������������Χ ����ƫ�� ���ڲο�ֵ�����ﵽ��λ��Ŀ��
			motorYaw.posCtrl.motorBias = (Yaw_Limit(6000, motorYaw.posCtrl.motorPos)*(motorYaw.posCtrl.motorPos - 6000) \
										+ Yaw_Limit(motorYaw.posCtrl.motorPos, 3600)*(motorYaw.posCtrl.motorPos - 3600))/10;
			
			//�����IMUģʽ�� ң��ֵ��λ
			Motor_AbsPos(&(motorYaw.posCtrl), yawPos, 380, -60);
			
			/************* Pitch Axis Pos Change ************/
//			motorPitch.veloCtrl.rawVel = -JYgyro.gyroVeloy;
			motorPitch.posCtrl.rawPos = JYgyro.gyroAngley;
			
			pitchPos = CH1Radio * (RemoteCtrlData.remote.ch1 - 1024) - RemoteCtrlData.mouse.y*0.01;
			//�����IMUģʽ�� ң��ֵ��λ
			Motor_AbsPos(&(motorPitch.posCtrl), pitchPos, 185, 145);
			
			/*************** �Ӿ����� *************/
			if(RemoteCtrlData.remote.s1 == 3)  //�Ӿ���������
			{
//				motorYaw.posCtrl.motorBias = -yawInc;
//				TD_Calculate(&tdYawPc, -yawInc);
				Motor_AbsPos(&(motorYaw.posCtrl), -yawInc/90, 350, 5);
//				Motor_AbsPos(&(motorPitch.posCtrl), -pitchInc/120, 182, 145);
			}
			
		}
		else  //����������ջ�ģʽ
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
		
		{  //���������䵯��
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

		Gimbal_Control();  //������ƺ���
		
	}

//	mpuFifoCount = mpu6500_read_single_reg(MPU_FIFO_COUNTH);
//	delay_us(100);
//	mpuFifoCount = ((mpuFifoCount<<8) | mpu6500_read_single_reg(MPU_FIFO_COUNTL));
	
	//	CtrlDebug(motorPitch.posCtrl.rawPos, motorPitch.posCtrl.output, td2.v1, td2.v2, 0, 0, 0, 0, 0, 0);  //���͵�����Ϣ������
	
}
