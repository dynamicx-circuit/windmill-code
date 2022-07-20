//
// Created by 58286 on 2022/7/20.
//

#ifndef WINDMILL_FUNCTION_MOTOR_H_
#define WINDMILL_FUNCTION_MOTOR_H_
#include "main.h"
#include "math.h"

#define CAN_ID 0x201

#define FILTER_BUF_LEN 2

typedef enum{
  UNIFORM_VELOCITY = 0,
  SIN_VELOCITY = 1,
  MOTOR_STOP = 2
}MotorState;

typedef struct{
  int16_t speed_rpm;
  int16_t real_current;
  int16_t given_current;
  uint8_t hall;
  uint16_t angle;				//abs angle range:[0,8191]
  uint16_t last_angle;	//abs angle range:[0,8191]
  uint16_t offset_angle;
  int32_t round_cnt;
  int32_t total_angle;
  uint8_t buf_idx;
  uint16_t angle_buf[FILTER_BUF_LEN];
  uint16_t fited_angle;
  uint32_t msg_cnt;
}moto_measure_t;

enum
{
  LLAST = 0,
  LAST = 1,
  NOW = 2,

  POSITION_PID,
  DELTA_PID,
};
typedef struct __pid_t
{
  float p;
  float i;
  float d;

  float set[3]; //目标值,包含NOW， LAST， LLAST上上次
  float get[3]; //测量值
  float err[3]; //误差

  float pout; //p输出
  float iout; //i输出
  float dout; //d输出

  float pos_out;      //本次位置式输出
  float last_pos_out; //上次输出
  float delta_u;      //本次增量值
  float delta_out;    //本次增量式输出 = last_delta_out + delta_u
  float last_delta_out;

  float max_err;
  float deadband; //err < deadband return
  uint32_t pid_mode;
  uint32_t MaxOutput;     //输出限幅
  uint32_t IntegralLimit; //积分限幅

  void (*f_param_init)(struct __pid_t *pid, //PID参数初始化
                       uint32_t pid_mode,
                       uint32_t maxOutput,
                       uint32_t integralLimit,
                       float p,
                       float i,
                       float d);
  void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d); //pid三个参数修改

} pid_t;

extern pid_t pid_rol;
extern pid_t pid_pit;
extern pid_t pid_yaw;
extern pid_t pid_pit_omg;
extern pid_t pid_yaw_omg;
extern pid_t pid_spd[4];
extern pid_t pid_yaw_alfa;
extern pid_t pid_chassis_angle;
extern pid_t pid_poke;
extern pid_t pid_poke_omg;
extern pid_t pid_imu_tmp;  //imu_temperature
extern pid_t pid_cali_bby; //big buff yaw
extern pid_t pid_cali_bbp;
extern pid_t pid_omg;
extern pid_t pid_pos;

extern uint8_t can_rx_data[8];
extern moto_measure_t moto_measure;

//void GetMotoMeasure(moto_measure_t *ptr);
//void SetMotoCurrent(uint16_t current);
//
//void PID_struct_init(
//    pid_t *pid,
//    uint32_t mode,
//    uint32_t maxout,
//    uint32_t intergral_limit,
//    float kp,
//    float ki,
//    float kd);
//
//float pid_calc(pid_t *pid, float fdb, float ref);
void MotorRun(uint8_t state,uint16_t dt);
void MotorInit(void);

#endif // WINDMILL_FUNCTION_MOTOR_H_
