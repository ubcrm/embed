#ifndef USER_LIB_H
#define USER_LIB_H
#include "main.h"

typedef __packed struct
{
    fp32 input;        //��������
    fp32 out;          //�������
    fp32 min_value;    //�޷���Сֵ
    fp32 max_value;    //�޷����ֵ
    fp32 frame_period; //ʱ����
} ramp_function_source_t;

typedef __packed struct
{
    fp32 input;        //��������
    fp32 out;          //�˲����������
    fp32 num[1];       //�˲�����
    fp32 frame_period; //�˲���ʱ���� ��λ s
} first_order_filter_type_t;
//���ٿ���
extern fp32 invSqrt(fp32 num);

//б��������ʼ��
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);

//б����������
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);
//��������
extern void abs_limit(fp32 *num, fp32 Limit);
//�жϷ���λ
extern fp32 sign(fp32 value);
//��������
extern fp32 fp32_deadzone(fp32 Value, fp32 minValue, fp32 maxValue);
//int26����
extern int16_t int16_deadzone(int16_t Value, int16_t minValue, int16_t maxValue);
//�޷�����
extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
//�޷�����
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//ѭ���޷�����
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
//�Ƕ� ���޷� 180 ~ -180
extern fp32 theta_format(fp32 Ang);

/*
* Returns an angle between -pi and pi
* Requires: an angle in radians
* Returns: an equivalent angle in radians between -pi and pi
*/
float get_domain_angle(float alpha);

/*
* Gets smallest angle required to travel from alpha to beta
* requires: two angles in radians
* returns: angle in radians with magnitude < 2*pi
*/
float get_relative_angle(float alpha, float beta);

/**
 * Maps float in a specified range to an int in a new range as a linear mapping function
 */
int linear_map_int_to_int(int val, int val_min, int val_max, int out_min, int out_max);

//���ȸ�ʽ��Ϊ-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)
#define average(x,y) ((x + y) / 2.0)
#define range(x,y) (y - x)


#endif
