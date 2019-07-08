#ifndef _REFEREE_COMM_H_
#define _REFEREE_COMM_H_

#include "stm32f4xx_hal.h"
#include "CRC.h"

#define BSP_USART3_DMA_RX_BUF_LEN 128u
#define BSP_USART3_DMA_TX_BUF_LEN 30u

#define FRAME_HEADER_LEN	sizeof(FrameHeader_t)
#define CMD_ID_LEN					2u	//�����볤��
#define CRC16_LEN					2u	//CRC16

/* ����ϵͳ���ݶγ��� */
#define GAME_STATE_LEN					3u	
#define GAME_RESULT_LEN					1u
#define GAME_ROBOTSURVIVORS_LEN			2u	
#define EVENT_DATA_LEN					4u
#define SUPPLY_PROJECTILE_ACTION_LEN	3u
#define SUPPLY_PROJECTILE_BOOKING_LEN	2u
#define GAME_ROBOT_STATE_LEN			15u
#define POWER_HEAT_DATA_LEN				14u
#define GAME_ROBOT_POS_LEN				16u
#define BUFF_MUSK_LEN					1u
#define AERIAL_ROBOT_ENERGY_LEN			3u
#define ROBOT_HURT_LEN					1u
#define SHOOT_DATA_LEN					6u

#define FRAME_HEADER_SOF		0xA5

#define GAME_STATE_CMD_ID					0x0001
#define GAME_RESULT_CMD_ID					0x0002
#define GAME_ROBOTSURVIVORS_CMD_ID			0x0003
#define EVENT_DATA_CMD_ID					0x0101
#define SUPPLY_PROJECTILE_ACTION_CMD_ID		0x0102
#define SUPPLY_PROJECTILE_BOOKING_CMD_ID	0x0103
#define GAME_ROBOT_STATE_CMD_ID				0x0201
#define POWER_HEAT_DATA_CMD_ID				0x0202
#define GAME_ROBOT_POS_CMD_ID				0x0203
#define BUFF_MUSK_CMD_ID					0x0204
#define AERIAL_ROBOT_ENERGY_CMD_ID			0x0205
#define ROBOT_HURT_CMD_ID					0x0206
#define SHOOT_DATA_CMD_ID					0x0207
#define INTERACTIVE_DATA_CMD_ID				0x0301

typedef __packed struct
{
	uint8_t		sof;			//֡��ʼ�ֽ�
	uint16_t	dataLength;		//���ݶ�Data����
	uint8_t		seq;			//�����
	uint8_t		crc8;			//֡ͷCRC8У��
}FrameHeader_t;					//FrameHeader

typedef __packed struct
{
	uint8_t		game_type 		: 4;	//��������
	uint8_t 	game_progress 	: 4;	//��ǰ�����׶�
	uint16_t 	stage_remain_time;		//��ǰ�׶�ʣ��ʱ��
}ext_game_state_t;					//����״̬

typedef __packed struct
{
	uint8_t winner;			//�������
}ext_game_result_t;			//�����������

typedef __packed struct
{
	uint16_t robot_legion;
}ext_game_robot_survivors_t;	//�����˴������

typedef __packed struct
{
	uint32_t event_type;
}ext_event_data_t;				//�����¼�����

typedef __packed struct
{
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
}ext_supply_projectile_action_t;	//����վ����

typedef __packed struct
{
	uint8_t supply_projectile_id;
	uint8_t supply_num;
}ext_supply_projectile_booking_t;	//����վԤԼ�ӵ�

typedef __packed struct
{
	uint8_t 	robot_id;
	uint8_t		robot_level;
	uint16_t 	max_HP;
	uint16_t 	shooter_heat0_cooling_rate;
	uint16_t 	shooter_heat0_cooling_limit;
	uint16_t 	shooter_heat1_cooling_rate;
	uint16_t 	shooter_heat1_cooling_limit;
	uint8_t 	mains_power_gimbal_output :		1;
	uint8_t 	mains_power_chassis_output : 	1;
	uint8_t 	mains_power_shooter_output :	1;
}ext_game_robot_state_t;			//����������״̬

typedef __packed struct
{
	uint16_t	chassis_volt;			//���������ѹ
	uint16_t	chassis_current;		//�����������
	float		chassis_Power;			//�����������
	uint16_t	chassis_power_buffer;	//���̹��ʻ���
	uint16_t	shooter_heat0;			//17mmǹ������
	uint16_t	shooter_heat1;			//42mmǹ������
}ext_power_heat_data_t;					//ʵʱ������������

typedef __packed struct
{
	float x;				//λ��X����ֵ
	float y;				//λ��Y����ֵ
	float z;				//λ��Z����ֵ
	float yaw;				//ǹ�ڳ���Ƕ�ֵ
}ext_game_robot_pos_t;		//������λ��

typedef __packed struct
{
	uint8_t power_rune_buff;	//Buff���ͣ�1��ʾ��Ч
}ext_buff_musk_t;					//Buff��ȡ����

typedef __packed struct
{
	uint8_t energy_point;		//���۵�������
	uint8_t attack_time;		//�ɹ���ʱ��
}aerial_robot_energy_t;			//���л���������״̬

typedef __packed struct
{
    uint8_t	armor_id  	: 4;	//���仯����Ϊװ���˺�ʱ����ʶװ��ID
    uint8_t	hurt_type   : 4;	//Ѫ���仯����
}ext_robot_hurt_t;				//�˺�״̬

typedef __packed struct
{
    uint8_t	bullet_type;      	//�ӵ�����
    uint8_t	bullet_freq;      	//�ӵ���Ƶ
    float	bullet_speed;		//�ӵ�����
}ext_shoot_data_t;              //ʵʱ�����Ϣ

typedef __packed struct
{
	uint16_t data_cmd_id;		//���ݶ�����ID
	uint16_t send_ID;			//������ID
	uint16_t receiver_ID;		//������ID
}ext_student_interactive_header_data_t;		//�������ݶ�ͷ

typedef __packed struct
{ 
	float data1;				//�Զ�������1
	float data2;				//�Զ�������2
	float data3;				//�Զ�������3
	uint8_t mask;				//�Զ�������4
}client_custom_data_t;			//�ͻ����Զ�������

typedef __packed struct
{
	ext_game_state_t					GameState_t;
	
	ext_game_result_t					GameResult_t;
	
	ext_game_robot_survivors_t			GameRobotSurvivors_t;
	
	ext_event_data_t					EventData_t;
	
	ext_supply_projectile_action_t		SupplyProjectileAction_t;
	
	ext_supply_projectile_booking_t		SupplyProjectileBooking_t;
	
	ext_game_robot_state_t				GameRobotState_t;
	
	ext_power_heat_data_t				PowerHeatData_t;
	
	ext_game_robot_pos_t				GameRobotPos_t;
	
	ext_buff_musk_t						BuffMusk_t;
	
	aerial_robot_energy_t				AerialRobotEnergy_t;
	
	ext_robot_hurt_t					RobotHurt_t;
	
	ext_shoot_data_t					ShootData_t;
	
}ext_referee_data_t;

extern ext_referee_data_t RefereeData_t;

extern uint8_t USART3_DMA_RX_BUF[BSP_USART3_DMA_RX_BUF_LEN];
extern uint8_t USART3_DMA_TX_BUF[BSP_USART3_DMA_TX_BUF_LEN];

void Referee_Data_Receive_Start(void);

void Referee_Data_Receive(void);

void Referee_Decode(uint8_t *pData);

#endif
