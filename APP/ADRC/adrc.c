/*
二阶自抗扰控制器,它的参数主要包括

ESO(a1, a2, h, beta_01, beta_02, beta_03)
TD(r)
NLSEF(c, h1)
噪声滤波因子 h0

其中a1, a2是指数函数的幂,当它们为1时,控制器即为线性控制器,否则就为非线性控制器。

	为函数的参数,只有在非线性时才有效。因此,称			为非线性参数。
	
经过大量的仿真可知这些参数的微小变化就会极大地影响其他参数的整定,在

控制器启动时一般不轻易修改这些参数,而其余的参数		,则可以在线修改。

*/

#include "adrc.h"
#include "math.h"
#include "Motor_Ctrl.h"

/****************** 基本函数 *********************
**************************************************
*************************************************/

/***** 快速开方 **********/
static float my_sqrt(float num)
{
    float halfnum = 0.5f * num;
    float y = num;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

/*********** 限幅函数 ************/
static float Constrain_Float(float amt, float low, float high)
{
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/********** sign函数 ***********/
static float sign(float x)
{
	if(x > 0)
		return (1.0f);
	if(x < 0)
		return (-1.0f);
	return 0;
}

/********* fsg 函数 ***********/
static float fsg(float x, float d) // (d)>=0
{
	if(d >= 0)
	{
		if(abs(x) <= d)
			return 1.0f;
		else 
			return 0.0f;
	}
	return 0;
}

/*********** fal 函数 **********/
static float fal(float e, float alpha, float delta)
{ 
	float y = 0.0f;
	if(abs(e) <= delta)
		y = e / (pow(delta, 1.0f-alpha) + 0.00001f);
	else 
		y = (float)pow(abs(e), alpha) * sign(e);
	
	return (y);
}

/********** fhan 函数 **********/

//static float fhan(float x1, float x2, float r, float h)
//{
//	float d, a0, y, a1, a2, a;
//	float fh = 0;
//	
//	d = r*h*h;
//	a0 = h*x2;
//	y = x1 + a0;
//	a1 = my_sqrt(d*d + 8*d*abs(y));
//	a2 = a0 + sign(y)*(a1-d)/2;
//	a = (a0+y-a2)*fsg(y,d) + a2;
//	
//	fh = -r*(a/d - sign(a))*fsg(a,d) - r*sign(a);
//	return (fh);
//}
static float fhan(float x1, float x2, float r, float h)
{
	float d, d0, y, a0, a, fh;
    d = r*h;
    d0 = h*d;
    y = x1 + h*x2;
    a0 = sqrt(d*d +8*r*abs(y));
     
     if(abs(y)>d0)
         a = x2 + (a0 - d)*sign(y)/2;
     else 
         a = x2 + y/h;
     
     if (abs(a)>d)
         fh = -r*sign(a);
     else
         fh = -r*a/d;
	 
     return fh;
}

//跟踪微分器初始化
void TD_Init(TD *td, float r, float h, float n0)
{
	td->r = r;
	td->h = h;
	td->N0 = n0;
}
//跟踪微分器计算
void TD_Calculate(TD *td, float expert)
{
	td->v1 += td->h * td->v2;
	td->v2 += td->h * fhan(td->v1 - expert, td->v2, td->r, td->h * td->N0);
}

//Yaw电机adrc参数初始化
void ADRC_Yaw_Init(void)
{
	ADRC_Yaw.r0 = 3000;  //响应速度
	ADRC_Yaw.td_h = 0.01; //滤波
	ADRC_Yaw.h0 = 0.001;  //跟踪速度
	
/************ ESO ***************/
	ADRC_Yaw.eso_h = 0.01;
	ADRC_Yaw.a1 = 0.5;
	ADRC_Yaw.eso_t1 = 0.008;
	ADRC_Yaw.a2 = 0.25;
	ADRC_Yaw.eso_t2 = 0.01;
	
	ADRC_Yaw.beta_01 = 100;
	ADRC_Yaw.beta_02 = 500;
	ADRC_Yaw.beta_03 = 600;
	
	ADRC_Yaw.e = 0;
	ADRC_Yaw.y = 0;
	ADRC_Yaw.z1 = 0;
	ADRC_Yaw.z2 = 0;
	ADRC_Yaw.z3 = 0;
/************* NLSEF ****************/
	ADRC_Yaw.c = 4;
	ADRC_Yaw.r1 = 200;
	ADRC_Yaw.h1 = 0.05;  //u0=-fhan(e1,c*e2*e2,r,h1);
	
	ADRC_Yaw.b0 = 1;  
}

/*
  adrc_expert 为系统输入，即期望值
*/
static void ADRC_TD(ADRC_Data *adrc, float adrc_expert)
{
//	float d, a0, y, a1, a2, a;
//	float x1;
//	x1 = adrc->v1 - adrc_expert;
//	adrc->h0 = adrc->N0 * adrc->h;
//	
//	d = adrc->r*adrc->h0*adrc->h0;
//	a0 = adrc->h0*adrc->v2;
//	y = x1 + a0;
//	a1 = my_sqrt(d*d + 8*d*abs(y));
//	a2 = a0 + sign(y)*(a1-d)/2;
//	a = (a0+y-a2)*fsg(y,d) + a2;
//	
//	adrc->fh = -adrc->r*(a/d - sign(a))*fsg(a,d) - adrc->r*sign(a);
	
	adrc->v1 += adrc->td_h*adrc->v2;  //更新最速跟踪状态量v1
//	adrc->v2 += adrc->h*adrc->fh;  //更新最速跟踪状态量微分v2;
	adrc->v2 += adrc->td_h*fhan(adrc->v1-adrc_expert, adrc->v2, adrc->r0, adrc->h0);
}


/***************** 扩张状态观测器 **********************
*******************************************************/

/*********** ESO_2N ***************/
static void ADRC_ESO_2N(ADRC_Data *adrc)
{
	adrc->e = adrc->z1 - adrc->y;  //状态误差
	adrc->fe = fal(adrc->e, 0.5, adrc->eso_t1); //非线性函数，提取跟踪状态与当前状态误差
	adrc->fe1 = fal(adrc->e, 0.25, adrc->eso_t2);
	
	/*********** 状态量更新 ***********/
	adrc->z1 += adrc->eso_h*(adrc->z2 - adrc->beta_01*adrc->e + adrc->u*adrc->b0);
	adrc->z2 += adrc->eso_h*(-adrc->beta_02 * adrc->e);
}

/******** ESO_3N **********/
//状态观测器参数beta01=1/h  beta02=1/(3*h^2)  beta03=2/(8^2*h^3) ...
static void ADRC_ESO_3N(ADRC_Data *adrc)
{
	adrc->e = adrc->z1 - adrc->y;  //状态误差
	adrc->fe = fal(adrc->e, adrc->a1, adrc->alpha1); //非线性函数，提取跟踪状态与当前状态误差
	adrc->fe1 = fal(adrc->e, adrc->a2, adrc->alpha2);
	
	/*********** 状态量更新 ***********/
	adrc->z1 += adrc->eso_h*(adrc->z2 - adrc->beta_01*adrc->e);
	adrc->z2 += adrc->eso_h*(adrc->z3 - adrc->beta_02*adrc->fe + adrc->u*adrc->b0);
	adrc->z3 += adrc->eso_h*(-adrc->beta_03*adrc->fe1);//ESO估计状态加速度信号，进行扰动补偿
}
//inline void LESO_Init(ESO* eso,)
//{
//	eso->b0 = 
//	eso->beta_01 = ;
//	eso->beta_02 = ;
//	eso->beta_03 = ;
//	eso->h = ;
//	eso->z1 = eso->z2 = eso->z3 = 0;
//}

inline void ADRC_LESO(ESO* eso, float y)
{
	eso->e = y - eso->z1;
	eso->z3 += eso->h * (eso->beta_03*eso->e);
	eso->z2 += eso->h * (eso->z3 + eso->beta_02*eso->e + eso->b0*eso->u);
	eso->z1 += eso->h * (eso->z2 + eso->beta_01*eso->e);
}

/********* 非线性状态误差反馈 ***********/
static void ADRC_NLSEF2(ADRC_Data *adrc)  //组合方式：u0=beta_1*fal(e1,alpha1,delta)+beta_2*fal(e2,alpha2,delta)
{
	float e2;
	e2 = limit_float(adrc->e2, -200, 200);
	adrc->u0 = adrc->beta_1 * fal(adrc->e1, adrc->alpha1, adrc->delta) 
			  + adrc->beta_2 * fal(e2, adrc->alpha2, adrc->delta);
}
static void ADRC_NLSEF4(ADRC_Data *adrc) //组合方式：u0=-fhan(e1,c*e2*e2,r,h1);
{
	adrc->e1 = adrc->v1 - adrc->z1;
	adrc->e2 = adrc->v2 - adrc->z2;
	
	adrc->u0 = -fhan(adrc->e1, adrc->c*adrc->e2, adrc->r1, adrc->h1);
}

/********************************/

void ADRC_Control(ADRC_Data *adrc, float adrc_expert, float adrc_feedback)
{
/********
 安排过渡过程，输入为期望给定，
 由TD跟踪微分器得到：过渡期望信号v1，过渡期望微分信号v2
******/
	ADRC_TD(adrc, adrc_expert);
 
/************系统输出值为反馈量，状态反馈，ESO扩张状态观测器的输入*********/
	adrc->y = adrc_feedback;
	
/*****
 扩张状态观测器，得到反馈信号的扩张状态：
 1、状态信号z1；
 2、状态速度信号z2；
 3、状态加速度信号z3。
 其中z1、z2用于作为状态反馈与TD微分跟踪器得到的x1,x2做差后，经过非线性函数映射，乘以beta系数后，
 组合得到未加入状态加速度估计扰动补偿的原始控制量u  
*********/
	ADRC_ESO_3N(adrc);//

  /********状态误差反馈率***/
	ADRC_NLSEF4(adrc);
/********线性组合*******/
//  u0=-fhan(e1,e2,r,h1);

  /**********扰动补偿*******/
	adrc->u = adrc->u0 - adrc->z3/adrc->b0;

   //输出限幅
	adrc->u = limit_float(adrc->u0, -10000, 10000);
}







