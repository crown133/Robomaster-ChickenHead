#ifndef _ADRC_H_
#define _ADRC_H_

#include "sys.h"

#define abs(x)  (((x)>0)?(x):-(x))  //绝对值函数

#define limit_float(x, min, max)	( (x) < (min) ? (min) : (x) > (max) ? (max) : (x) )//限幅函数

typedef struct
{
/*****安排过渡过程 (TD)*******/
float v1;//跟踪微分期状态量
float v2;//跟踪微分期状态量微分项
float r;//时间尺度
float h;//ADRC系统积分时间
float N0;//跟踪微分器解决速度超调h0=N*h
 
float h0;
float fh;//最速微分加速度跟踪量
} TD;

typedef struct
{
/*****安排过渡过程 (TD)*******/
float v1;//跟踪微分期状态量
float v2;//跟踪微分期状态量微分项
float r0;//时间尺度
float td_h;//ADRC TD系统积分时间
 
float h0;
float fh;//最速微分加速度跟踪量
/*****扩张状态观测器 (ESO)*******/
/******已系统输出y和输入u来跟踪估计系统状态和扰动*****/
float eso_h;
float a1;  //fe 的 fal()参数
float eso_t1;
float a2;  //fe1 的 fal()参数
float eso_t2;
	
float z1;
float z2;
float z3;//根据控制对象输入与输出，提取的扰动信息
float e;//系统状态误差
float y;//系统输出量
float fe;
float fe1;
float beta_01;
float beta_02;
float beta_03;
float b0;//扰动补偿

/**********系统状态误差反馈率*********/
float e0;//状态误差积分项
float e1;//  v1-z1  状态偏差
float e2;//  v2-z2  状态量微分项 
float u0;//非线性组合系统输出
float u;//带扰动补偿后的输出
 
 
/*********第一种组合形式*********/
float beta_0;//线性
float beta_1;//非线性组合参数
float beta_2;//u0=beta_1*e1+beta_2*e2+(beta_0*e0);
/*********第二种组合形式*********/
float alpha1;//u0=beta_1*fal(e1,alpha1,delta)+beta_2*fal(e2,alpha2,delta)
float alpha2;//0<alpha1<1<alpha2
float delta;//线性段的区间长度
/*********第三种组合形式*********/
float h1;//u0=-fhan(e1,e2,r,h1);
/*********第四种组合形式*********/
float r1;
float c;//u0=-fhan(e1,c*e2,r,h1);

	
}ADRC_Data;


extern void ADRC_Yaw_Init(void);  ///////数值赋值///////
extern void ADRC_Control(ADRC_Data *adrc, float adrc_expert, float adrc_feedback);


extern void TD_Init(TD *td, float r, float h, float n0);
extern void TD_Calculate(TD *td, float expert);

#endif

