#ifndef _ADRC_H_
#define _ADRC_H_

#include "sys.h"

#define abs(x)  (((x)>0)?(x):-(x))  //����ֵ����

#define limit_float(x, min, max)	( (x) < (min) ? (min) : (x) > (max) ? (max) : (x) )//�޷�����

typedef struct
{
/*****���Ź��ɹ��� (TD)*******/
float v1;//����΢����״̬��
float v2;//����΢����״̬��΢����
float r;//ʱ��߶�
float h;//ADRCϵͳ����ʱ��
float N0;//����΢��������ٶȳ���h0=N*h
 
float h0;
float fh;//����΢�ּ��ٶȸ�����
} TD;

typedef struct
{
/*****���Ź��ɹ��� (TD)*******/
float v1;//����΢����״̬��
float v2;//����΢����״̬��΢����
float r0;//ʱ��߶�
float td_h;//ADRC TDϵͳ����ʱ��
 
float h0;
float fh;//����΢�ּ��ٶȸ�����
/*****����״̬�۲��� (ESO)*******/
/******��ϵͳ���y������u�����ٹ���ϵͳ״̬���Ŷ�*****/
float eso_h;
float a1;  //fe �� fal()����
float eso_t1;
float a2;  //fe1 �� fal()����
float eso_t2;
	
float z1;
float z2;
float z3;//���ݿ��ƶ����������������ȡ���Ŷ���Ϣ
float e;//ϵͳ״̬���
float y;//ϵͳ�����
float fe;
float fe1;
float beta_01;
float beta_02;
float beta_03;
float b0;//�Ŷ�����

/**********ϵͳ״̬������*********/
float e0;//״̬��������
float e1;//  v1-z1  ״̬ƫ��
float e2;//  v2-z2  ״̬��΢���� 
float u0;//���������ϵͳ���
float u;//���Ŷ�����������
 
 
/*********��һ�������ʽ*********/
float beta_0;//����
float beta_1;//��������ϲ���
float beta_2;//u0=beta_1*e1+beta_2*e2+(beta_0*e0);
/*********�ڶ��������ʽ*********/
float alpha1;//u0=beta_1*fal(e1,alpha1,delta)+beta_2*fal(e2,alpha2,delta)
float alpha2;//0<alpha1<1<alpha2
float delta;//���Զε����䳤��
/*********�����������ʽ*********/
float h1;//u0=-fhan(e1,e2,r,h1);
/*********�����������ʽ*********/
float r1;
float c;//u0=-fhan(e1,c*e2,r,h1);

	
}ADRC_Data;


extern void ADRC_Yaw_Init(void);  ///////��ֵ��ֵ///////
extern void ADRC_Control(ADRC_Data *adrc, float adrc_expert, float adrc_feedback);


extern void TD_Init(TD *td, float r, float h, float n0);
extern void TD_Calculate(TD *td, float expert);

#endif

