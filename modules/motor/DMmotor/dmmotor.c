#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"
#include "dm_imu.h"
#include "ins_task.h"
#include "robot_cmd.h"
#include "robot_def.h"

extern imu_t imu;
extern INS_t INS;
extern Chassis_Upload_Data_s chassis_fetch_data;
extern Chassis_Ctrl_Cmd_s chassis_cmd_send;

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];
/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
}

static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);

    measure->last_position = measure->position;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
}

/************************************************************************
* @brief:      	read_motor_ctrl_fbdata: 发送读取电机反馈数据的命令
* @param[in]:   id:    电机can id
* @retval:     	void
* @details:    	读取电机控制反馈的数据
************************************************************************
**/
// void read_can1_motor_ctrl_fbdata(uint16_t id) 
// {
//     // 创建临时CAN实例用于发送数据
//     static CANInstance temp_can1_instance = {0};
    
//     // 初始化CAN实例（仅首次）
//     if(temp_can1_instance.can_handle == NULL) {
//         temp_can1_instance.can_handle = &hfdcan1;  // 使用CAN1，根据实际情况调整
//         temp_can1_instance.tx_id = id;          // 示例发送ID，根据实际情况调整
// #ifdef FDCAN
//         temp_can1_instance.txconf.Identifier = temp_can1_instance.tx_id;
//         temp_can1_instance.txconf.TxFrameType = FDCAN_DATA_FRAME;
//         temp_can1_instance.txconf.DataLength = FDCAN_DLC_BYTES_8; // 发送8字节
//         temp_can1_instance.txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//         temp_can1_instance.txconf.BitRateSwitch = FDCAN_BRS_OFF;
//         temp_can1_instance.txconf.FDFormat = FDCAN_CLASSIC_CAN;
//         temp_can1_instance.txconf.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//         temp_can1_instance.txconf.MessageMarker = 0;
// #endif
//     }
    
//     // 设置要发送的固定数据: 7F FF 7F F0 00 00 07 FF
//     temp_can1_instance.tx_buff[0] = 0xFF;
//     temp_can1_instance.tx_buff[1] = 0xFF;
//     temp_can1_instance.tx_buff[2] = 0xFF;
//     temp_can1_instance.tx_buff[3] = 0xFF;
//     temp_can1_instance.tx_buff[4] = 0xFF;
//     temp_can1_instance.tx_buff[5] = 0xFF;
//     temp_can1_instance.tx_buff[6] = 0xFF;
//     temp_can1_instance.tx_buff[7] = 0xFE;
    
//     // 发送数据
//     CANTransmit(&temp_can1_instance, 1);
// }

// void read_can2_motor_ctrl_fbdata(uint16_t id) 
// {
//     // 创建临时CAN实例用于发送数据
//     static CANInstance temp_can2_instance = {0};
    
//     // 初始化CAN实例（仅首次）
//     if(temp_can2_instance.can_handle == NULL) {
//         temp_can2_instance.can_handle = &hfdcan2;  // 使用CAN1，根据实际情况调整
//         temp_can2_instance.tx_id = id;          // 示例发送ID，根据实际情况调整
// #ifdef FDCAN
//         temp_can2_instance.txconf.Identifier = temp_can2_instance.tx_id;
//         temp_can2_instance.txconf.TxFrameType = FDCAN_DATA_FRAME;
//         temp_can2_instance.txconf.DataLength = FDCAN_DLC_BYTES_8; // 发送8字节
//         temp_can2_instance.txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//         temp_can2_instance.txconf.BitRateSwitch = FDCAN_BRS_OFF;
//         temp_can2_instance.txconf.FDFormat = FDCAN_CLASSIC_CAN;
//         temp_can2_instance.txconf.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//         temp_can2_instance.txconf.MessageMarker = 0;
// #endif
//     }
    
//     // 设置要发送的固定数据: 7F FF 7F F0 00 00 07 FF
//     temp_can2_instance.tx_buff[0] = 0x7F;
//     temp_can2_instance.tx_buff[1] = 0xFF;
//     temp_can2_instance.tx_buff[2] = 0x7F;
//     temp_can2_instance.tx_buff[3] = 0xF0;
//     temp_can2_instance.tx_buff[4] = 0x00;
//     temp_can2_instance.tx_buff[5] = 0x00;
//     temp_can2_instance.tx_buff[6] = 0x07;
//     temp_can2_instance.tx_buff[7] = 0xFF;
    
//     // 发送数据
//     CANTransmit(&temp_can2_instance, 1);
// }

static void DMMotorLostCallback(void *motor_ptr)
{
}
void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));
    
    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);
    if(config->motor_type==set_zero)
    {
        DMMotorCaliEncoder(motor);
    }
    DWT_Delay(0.1);
    dm_motor_instance[idx++] = motor;
    return motor;
}

void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorStop(DMMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

float reset_imu_data_range(float data)
{
    if (data>180){
        data=data-360;}
    else if (data<-180){
        data=data+360;}
    return data;
}

float angle_feedback;
float yaw_1_angle_feedback;
float yaw_2_angle_feedback;
float yaw_1_angle_feedback1;
float yaw_2_angle_feedback1;
float loader_angle_feedback;
float pitch_1_angle_feedback;

float speed_ref;

float set1=0,set2=0,set=0,set3=0,last_set=0;
//set1用于yaw1电机设定值，set2用于yaw2电机设定值，set3用于拨弹盘电机设定值,set用于拨弹盘电机设定值
//last_set用于拨弹盘电机设置零点标志位
float err_feedback;//大小yaw同时动，大yaw给小yaw的前馈
float err_feedback1;//小yaw超限，小yaw给大yaw的前馈
uint8_t begain_fack_flag = 0;
float angle_feedback_difference=0.0f;
float imu_angle_difference=0.0f;
uint8_t protect_flag = 0,protect_flag_count=0;
float loader_set=0.0f;

float motor_position_angle_now=0.0f;
//@Todo: 

float forword_sppeed_feedback=0.0f;
float err;
void DMMotorTask(void const *argument)
{
    DMMotorInstance *motor = (DMMotorInstance *)argument;
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    DMMotor_Send_s motor_send_mailbox;
    if(setting->outer_loop_type==SPEED_LOOP && setting->close_loop_type==ANGLE_AND_SPEED_LOOP){
        motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
        motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
        motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
        motor_send_mailbox.Kp = 0x150; 
        motor_send_mailbox.Kd = 0x19A; 
        motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
        motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
        motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
        motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
        motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
        motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
        motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
        motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);
        CANTransmit(motor->motor_can_instace, 1);
        osDelay(1000);
        begain_fack_flag=1;
        imu_angle_difference=imu.yaw;

    }
    while (1)
    {
        //set = motor->pid_ref;
        // if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        //     set *= -1;

        // 根据闭环类型选择不同的控制策略
        if(setting->close_loop_type==ANGLE_AND_SPEED_LOOP){
             // 反馈位置+电机速度双环控制模式
            if (setting->angle_feedback_source == MOTOR_FEED ) 
            {
                if(setting->outer_loop_type==SPEED_LOOP)
                {
                    angle_feedback = radian_to_degree_180(motor->measure.position);
                    angle_feedback=reset_imu_data_range(angle_feedback+INS.Yaw);
                    motor_position_angle_now=angle_feedback;
                }
                else
                {angle_feedback=motor->measure.position;}
            } 
            else if (setting->angle_feedback_source == BM1088_FEED) 
            {
                angle_feedback = INS.Yaw;
            } 
            else if (setting->angle_feedback_source == IMU_FEED) 
            {
                // 使用IMU的yaw角度作为反馈
                angle_feedback = reset_imu_data_range(imu.yaw-imu_angle_difference);
            }
            
            

            if (setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
                angle_feedback *= -1;
                
            // 计算位置环PID输出（期望速度）
            if(setting->outer_loop_type==ANGLE_LOOP){//yaw2   加了反向
                if(imu_angle_difference!=0.0f){
                    set2=reset_imu_data_range(imu.yaw-imu_angle_difference);
                }
                else{
                    set2=motor_position_angle_now;
                }
               
                speed_ref=-PIDCalculate(&motor->angle_PID, angle_feedback, set2);

                err_feedback=speed_ref;

                speed_ref+=chassis_fetch_data.real_wz;
                chassis_fetch_data.real_wz=0;

                speed_ref-=err_feedback1;
                err_feedback1 = 0.0f;
                
                yaw_2_angle_feedback=speed_ref;
                if(begain_fack_flag==0)
                {speed_ref=0;}

                yaw_2_angle_feedback1=speed_ref;

                motor_send_mailbox.Kd = 0x500; 
            }
            else if (setting->outer_loop_type==SPEED_LOOP)//yaw1
            {
                set1 =reset_imu_data_range(motor->pid_ref-imu_angle_difference);
                speed_ref=PIDCalculate(&motor->angle_PID, angle_feedback, set1);
                speed_ref+=err_feedback;
                err_feedback = 0.0f;
                yaw_1_angle_feedback=speed_ref;
                //保护
                if(motor->measure.position>0.34 && speed_ref>0)
                {
                    err_feedback1=speed_ref;
                    speed_ref=0;
                }
                else if(motor->measure.position<-0.34 && speed_ref<0)
                {
                    err_feedback1=speed_ref;
                    speed_ref=0;
                }
                
                yaw_1_angle_feedback1=speed_ref;

                motor_send_mailbox.Kd = 0x600; 
            }

            // 设置位置和速度目标
            motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
            motor_send_mailbox.velocity_des = float_to_uint(speed_ref, DM_V_MIN, DM_V_MAX, 12);
            motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
            motor_send_mailbox.Kp = 0; 
           
        }

        else if (setting->close_loop_type == ANGLE_LOOP) {
            // 电机位置控制模式
            set = motor->pid_ref;
            
            if(setting->outer_loop_type == ANGLE_LOOP)//pitch电机
            {
                motor_send_mailbox.Kp = 0x0F6; 
                motor_send_mailbox.Kd = 0x4CD;
                pitch_1_angle_feedback=set; 
                motor_send_mailbox.position_des = float_to_uint(set, DM_P_MIN, DM_P_MAX, 16);
                motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
                motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
            }
            else if(setting->outer_loop_type == SPEED_LOOP)//拨弹盘电机
            {
                //得控恢复
                if(motor->stop_flag == MOTOR_STOP)
                {protect_flag==0;}

                //过载保护
                if(motor->measure.torque>8)
                {protect_flag_count++;protect_flag=1;}

                float motor_measure_position=motor->measure.position;
                err=set-motor_measure_position;
                if(err>12.5664)
                {err-=25.1328;}
                else if(err<-12.5664)
                {err+=25.1328;}
                speed_ref=PIDCalculate(&motor->angle_PID, 0, err)*10;
                loader_angle_feedback=speed_ref;


                motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
                motor_send_mailbox.velocity_des = float_to_uint(speed_ref, DM_V_MIN, DM_V_MAX, 12);
                motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);

                
                // motor_send_mailbox.Kp = 0x0F6; 
                // motor_send_mailbox.Kd = 0x4CD; 
                // motor_send_mailbox.Kp = 0x150; 
                motor_send_mailbox.Kp = 0;
                motor_send_mailbox.Kd = 0x500; 
                // if(set<last_set)
                // {DMMotorCaliEncoder(motor);}
                // last_set=set;
            }


           
                // motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
                // motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
                // motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
                // motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
                // motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
                // motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
                // motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
                // motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);
           
                // CANTransmit(motor->motor_can_instace,1);
        }
        else if (setting->close_loop_type == SPEED_LOOP) {
            // // 速度控制模式
            // float speed_feedback = (setting->speed_feedback_source == MOTOR_FEED) ? 
            //                        motor->measure.velocity : *motor->other_speed_feedback_ptr;
            // if (setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            //     speed_feedback *= -1;
                
            // // 计算速度环PID输出（期望扭矩）
            // PIDCalculate(&motor->speed_PID, speed_feedback, set);
            // set = motor->speed_PID.Output;
            
            // // 设置速度目标
            // motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
            // motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
            // motor_send_mailbox.torque_des = float_to_uint(set, DM_T_MIN, DM_T_MAX, 12);
            // motor_send_mailbox.Kp = 0; 
            // motor_send_mailbox.Kd = 0; 
        }
        else {
            // 力矩控制模式
            set3= motor->pid_ref;
            speed_ref=PIDCalculate(&motor->angle_PID, motor->measure.position, set3);
            
            LIMIT_MIN_MAX(set, DM_T_MIN, DM_T_MAX);
            motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
            motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
            motor_send_mailbox.torque_des = float_to_uint(set, DM_T_MIN, DM_T_MAX, 12);
            motor_send_mailbox.Kp = 0;
            motor_send_mailbox.Kd = 0;
        }

        if(motor->stop_flag == MOTOR_STOP || protect_flag==1) {
            motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
            motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
            motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
            motor_send_mailbox.Kp = 0;
            motor_send_mailbox.Kd = 0;
        }
        

        motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
        motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
        motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
        motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
        motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
        motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
        motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
        motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);

        CANTransmit(motor->motor_can_instace,1);

        osDelay(10);
        //  if(motor->stop_flag == MOTOR_STOP || protect_flag==1) {
        //     motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
        //     motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
        //     motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
        //     motor_send_mailbox.Kp = 0;
        //     motor_send_mailbox.Kd = 0;
        // }
    }
}
void DMMotorControlInit()
{
    char dm_task_name[5] = "dm";
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        char dm_id_buff[2] = {0};
        __itoa(i, dm_id_buff, 10);
        strcat(dm_task_name, dm_id_buff);
        osThreadDef(dm_task_name, DMMotorTask, osPriorityNormal, 0, 128);
        dm_task_handle[i] = osThreadCreate(osThread(dm_task_name), dm_motor_instance[i]);
    }
}