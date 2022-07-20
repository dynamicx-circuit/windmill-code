//
// Created by 58286 on 2022/7/20.
//
#include "motor.h"

#define ABS(x)		((x>0)? (x): (-x))

extern CAN_HandleTypeDef hcan;

uint8_t can_rx_data[8];
moto_measure_t moto_measure;
pid_t pid_struct;

static CAN_TxHeaderTypeDef can_header;
static uint8_t can_tx_data[8];
static uint8_t can_receive_flag = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  static uint8_t counter = 0;
  if(hcan->Instance == CAN1){
    can_receive_flag = 1;
    CAN_RxHeaderTypeDef rx_header;
    HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,can_rx_data);
  }
}

uint8_t GetMotoMeasure(moto_measure_t *ptr)
{
  if(can_receive_flag == 1){
    can_receive_flag = 0;
  }else return 0;
  ptr->last_angle = ptr->angle;
  ptr->angle = (uint16_t)(can_rx_data[0]<<8 | can_rx_data[1]) ;
  ptr->real_current  = (int16_t)(can_rx_data[2]<<8 | can_rx_data[3]);
  ptr->speed_rpm = ptr->real_current;	//这里是因为两种电调对应位不一样的信息
  ptr->given_current = (int16_t)(can_rx_data[4]<<8 | can_rx_data[5])/-5;
  ptr->hall = can_rx_data[6];
  if(ptr->angle - ptr->last_angle > 4096)
    ptr->round_cnt --;
  else if (ptr->angle - ptr->last_angle < -4096)
    ptr->round_cnt ++;
  ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

void SetMotoCurrent(uint16_t current)
{
  can_header.IDE = CAN_ID_STD;
  can_header.RTR = CAN_RTR_DATA;
  can_header.StdId = 0x200;
  can_header.DLC = 8;

  can_tx_data[0] = current>>8;
  can_tx_data[1] = current&0xFF;
  uint32_t tx_mailbox;

  HAL_CAN_AddTxMessage(&hcan,&can_header,can_tx_data,&tx_mailbox);
}

void abs_limit(float *a, float ABS_MAX){
  if(*a > ABS_MAX)
    *a = ABS_MAX;
  if(*a < -ABS_MAX)
    *a = -ABS_MAX;
}
/*参数初始化--------------------------------------------------------------*/
static void pid_param_init(
    pid_t *pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float 	kp,
    float 	ki,
    float 	kd)
{
  pid->IntegralLimit = intergral_limit;
  pid->MaxOutput = maxout;
  pid->pid_mode = mode;
  pid->p = kp;
  pid->i = ki;
  pid->d = kd;
}
/*中途更改参数设定(调试)------------------------------------------------------------*/
static void pid_reset(pid_t	*pid, float kp, float ki, float kd)
{
  pid->p = kp;
  pid->i = ki;
  pid->d = kd;
}

/**
    *@bref. calculate delta PID and position PID
    *@param[in] set： target
    *@param[in] real	measure
    */
float pid_calc(pid_t* pid, float get, float set){
  pid->get[NOW] = get;
  pid->set[NOW] = set;
  pid->err[NOW] = set - get;	//set - measure
  if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )
    return 0;
  if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
    return 0;

  if(pid->pid_mode == POSITION_PID) //位置式p
  {
    pid->pout = pid->p * pid->err[NOW];
    pid->iout += pid->i * pid->err[NOW];
    pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST] );
    abs_limit(&(pid->iout), pid->IntegralLimit);
    pid->pos_out = pid->pout + pid->iout + pid->dout;
    abs_limit(&(pid->pos_out), pid->MaxOutput);
    pid->last_pos_out = pid->pos_out;	//update last time
  }
  else if(pid->pid_mode == DELTA_PID)//增量式P
  {
    pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
    pid->iout = pid->i * pid->err[NOW];
    pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);

    abs_limit(&(pid->iout), pid->IntegralLimit);
    pid->delta_u = pid->pout + pid->iout + pid->dout;
    pid->delta_out = pid->last_delta_out + pid->delta_u;
    abs_limit(&(pid->delta_out), pid->MaxOutput);
    pid->last_delta_out = pid->delta_out;	//update last time
  }

  pid->err[LLAST] = pid->err[LAST];
  pid->err[LAST] = pid->err[NOW];
  pid->get[LLAST] = pid->get[LAST];
  pid->get[LAST] = pid->get[NOW];
  pid->set[LLAST] = pid->set[LAST];
  pid->set[LAST] = pid->set[NOW];
  return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;
//
}

/*pid总体初始化-----------------------------------------------------------------*/
void PID_struct_init(
    pid_t* pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float 	kp,
    float 	ki,
    float 	kd)
{
  /*init function pointer*/
  pid->f_param_init = pid_param_init;
  pid->f_pid_reset = pid_reset;
//	pid->f_cal_pid = pid_calc;
//	pid->f_cal_sp_pid = pid_sp_calc;	//addition

  /*init pid param */
  pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
}

void MotorInit(void)
{
  CAN_FilterTypeDef can_filter_st;
  can_filter_st.FilterActivation = ENABLE;
  can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_st.FilterIdHigh = (CAN_ID)<<5;
  can_filter_st.FilterIdLow = 0;
  can_filter_st.FilterMaskIdHigh = 0xFFE0;
  can_filter_st.FilterMaskIdLow = 0x0000;
  can_filter_st.FilterBank = 1;
  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
  HAL_CAN_ConfigFilter(&hcan, &can_filter_st);
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  PID_struct_init(&pid_struct,POSITION_PID,16599,16599,1.5f,0.5f,0.0f);
  pid_struct.max_err = 2000.0f;
}

void MotorRun(uint8_t state,uint16_t dt)
{
  static uint8_t set_speed;
  static float t = 0.0f;
  if(GetMotoMeasure(&moto_measure) == 1) return;
  if(state == UNIFORM_VELOCITY){
    set_speed = 190.0f;
    t = 0.0f;
  }else if(state == SIN_VELOCITY){
    set_speed = 0.785f*sinf(1.884f*t)+1.305f;
    set_speed *= 60*M_PI;
    t += 1.0f/(float)dt;
    if(t>3.335f) t = 0.0f;
  }else if(state == MOTOR_STOP){
    set_speed = 0.0f;
    t = 0.0f;
  }
  pid_calc(&pid_struct,(float)moto_measure.speed_rpm,set_speed);
  SetMotoCurrent((uint16_t)pid_struct.pos_out);
}