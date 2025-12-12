#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"

#include "bmi088.h"
#include "dmmotor.h"
#include "dm_imu.h"
#include "vofa.h"

extern imu_t imu;
extern INS_t INS;

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor, *pitch_motor;
DMMotorInstance *dmmotor_yaw1;
DMMotorInstance *dmmotor_yaw2;
DMMotorInstance *dmmotor_pitch;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static Chassis_Upload_Data_s chassis_real_speed;
static Subscriber_t *chassis_speed_sub; // 底盘反馈信息订阅者
//static float yaw_gyro_feedforward = -0.472267985;

//static float yaw_current_forward_abs = 1600.0f;
static float pitch_current_forward_abs = 1600.0f;

//static float yaw_current_feedforward = 0.f;

// 定义VOFA+ USART实例
//static USARTInstance *vofa_usart_instance;

void GimbalInit()

{
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源,

    //vofa_usart_instance = USARTRegister(&vofa_usart_config);
    // YAW

    // Motor_Init_Config_s yaw_config = {
    //     .can_init_config = {
    //         .can_handle = &hcan2,
    //         .tx_id = 7,
    //     },
    //     .controller_param_init_config = {
    //         .angle_PID = {
    //             .Kp = 0.3,    // 0.55 0.3
    //             .Ki = 0.08,   // 0.05 0.2
    //             .Kd = 0.02,   // 0.02 0.01
    //             .CoefA = 0.5, // 0.5
    //             .CoefB = 0.6, // 0.6
    //             .Output_LPF_RC = 0, // 0
    //             // .DeadBand = 0.02,           // 0.02
    //             .Derivative_LPF_RC = 0.015, // 0.008 0.01
    //             .Improve = PID_Trapezoid_Intergral |PID_ChangingIntegrationRate| PID_Integral_Limit |PID_Derivative_On_Measurement |  PID_OutputFilter |PID_DerivativeFilter,
    //             // .Improve = PID_Trapezoid_Intergral | PID_ChangingIntegrationRate | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
    //             .IntegralLimit = 1,

    //             .MaxOut = 400,
    //         },
    //         .speed_PID = {
    //             .Kp = 8000, // 22000
    //             .Ki = 0,    //
    //             .Kd = 0.002,
    //             .CoefA = 0.8,
    //             .CoefB = 0.1,
    //             .Output_LPF_RC = 0.002, // 0.002
    //             .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_OutputFilter,
    //             .IntegralLimit = 5000,
    //             .MaxOut = 20000,
    //         },
    //         .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
    //         // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
    //         .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
    //         .speed_feedforward_ptr = &(yaw_gyro_feedforward),
    //         .current_feedforward_ptr = &(yaw_current_feedforward),
    //     },
    //     .controller_setting_init_config = {
    //         .angle_feedback_source = OTHER_FEED, 
    //         .speed_feedback_source = OTHER_FEED, 
    //         .outer_loop_type = ANGLE_LOOP, 
    //         .close_loop_type = ANGLE_LOOP | SPEED_LOOP, 
    //         .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
    //         .feedforward_flag = CURRENT_FEEDFORWARD,
    //         // .feedforward_flag = SPEED_FEEDFORWARD,
    //     },
    //     .motor_type = GM6020};
    // PITCH,两个can2
    // Motor_Init_Config_s pitch_config = {
    //     .can_init_config = {
    //         .can_handle = &hcan1,
    //         .tx_id = 6,
    //     },
    //     .controller_param_init_config = {
    //         .angle_PID = {
    //             .Kp = 0.2,            // 0.4
    //             .Ki = 2,             // 0.15
    //             .Kd = 0.006,           // 0.006
    //             .CoefA = 0.5,          // 0.5
    //             .CoefB = 0.6,          // 0.6
    //             .DeadBand = 0.005,     // 0.005
    //             .Output_LPF_RC = 0.01, // 0.01
    //             .Improve = PID_Trapezoid_Intergral | PID_ChangingIntegrationRate | PID_Integral_Limit | PID_OutputFilter | PID_DerivativeFilter,
    //             .IntegralLimit = 1, // 1
    //             .MaxOut = 30,      // 600
    //         },
    //         .speed_PID = {
    //             .Kp = 6500,             // 14000
    //             .Ki = 0,                // 0
    //             .Kd = 0.000,           // 0.0005
    //             .CoefA = 1500,          // 1500
    //             .CoefB = 2000,          // 2000 
    //             .Output_LPF_RC = 0.005, // 0.005
    //             .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_OutputFilter,
    //             .IntegralLimit = 3000, // 3000
    //             .MaxOut = 20000,       // 20000
    //         },
    //         .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
    //         // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
    //         .other_speed_feedback_ptr = (&gimba_IMU_data->Gyro[0]),

    //         .current_feedforward_ptr = &pitch_current_forward_abs,
    //     },
    //     .controller_setting_init_config = {
    //         .angle_feedback_source = OTHER_FEED,
    //         .speed_feedback_source = OTHER_FEED,
    //         .outer_loop_type = ANGLE_LOOP,
    //         .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
    //         .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
    //         .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,

    //         .feedforward_flag = CURRENT_FEEDFORWARD,
    //     },
    //     .motor_type = GM6020,
    // };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动

        //-7.0
        Motor_Init_Config_s dm4310_pitch={
       .can_init_config ={
         .can_handle = &hcan1,
         .tx_id = 6,
         .rx_id = 16
       },
       .controller_param_init_config = {
           
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,   //pitch电机标志
            .close_loop_type = ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,  
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL 
        },
        .motor_type = no_set_zero
    };
    //小yaw
    Motor_Init_Config_s dm4310_yaw1={
       .can_init_config ={
         .can_handle = &hcan1,  
         .tx_id = 7 ,
         .rx_id = 17
       },
       .controller_param_init_config = {
           .angle_PID = {
                .Kp = 0.21,            // 0.17
                .Ki = 0.006,              //  0.008
                .Kd = 0.01,            //0.008
                .CoefA = 0.5,          // 0.5
                .CoefB = 0.6,          // 0.6
                .DeadBand = 0.00001,     // 0.005
                .Output_LPF_RC = 0.01, // 0.01
                .Improve = PID_Trapezoid_Intergral | PID_ChangingIntegrationRate | PID_Integral_Limit | PID_OutputFilter | PID_DerivativeFilter,
                .IntegralLimit = 1, // 1
                .MaxOut = 30,      // 600
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,    //这里改成前馈计算和使用的标志位了，不是原来的意思，ANGLE_LOOP是被计算前馈yaw2，SPEED_LOOP是使用前馈数据yaw1
            .close_loop_type = ANGLE_AND_SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,  
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL 
        },
        .motor_type = no_set_zero
    };

    //大yaw
    Motor_Init_Config_s dm8006_yaw2={
       .can_init_config ={
         .can_handle = &hcan2,
         .tx_id = 8 ,
         .rx_id = 18
       },
       .controller_param_init_config = {
           .angle_PID = {
                .Kp = 0.18,           // 减小Kp从0.2到0.15
                .Ki = 0.1,              // 保持或减小
                .Kd = 0.006,      // 0.006
                .CoefA = 0.5,          // 0.5
                .CoefB = 0.6,          // 0.6
                .DeadBand = 0.0001,     // 0.005
                .Output_LPF_RC = 0.01, // 0.01
                .Improve = PID_Trapezoid_Intergral | PID_ChangingIntegrationRate | PID_Integral_Limit | PID_OutputFilter | PID_DerivativeFilter,
                .IntegralLimit = 1, // 1
                .MaxOut = 45,      // 600
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = BM1088_FEED,
            .speed_feedback_source = BM1088_FEED,
            .outer_loop_type = ANGLE_LOOP,          //这里改成前馈计算和使用的标志位了，不是原来的意思，ANGLE_LOOP是被计算反馈，SPEED_LOOP是使用反馈
            .close_loop_type = ANGLE_AND_SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,  
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL 
        },
        .motor_type = no_set_zero
    };
    dmmotor_yaw1 = DMMotorInit(&dm4310_yaw1);
    dmmotor_yaw2 = DMMotorInit(&dm8006_yaw2);
    dmmotor_pitch = DMMotorInit(&dm4310_pitch);
    //yaw_motor = DJIMotorInit(&yaw_config);
    //pitch_motor = DJIMotorInit(&pitch_config);

    DMMotorSetRef(dmmotor_yaw1, 0.0f);
    DMMotorSetRef(dmmotor_pitch, 0.0f);

    //DMMotorSetRef(dmmotor_yaw2, 0.0f);

    // Motor_Init_Config_s dm8006={
    //    .can_init_config ={
    //      .can_handle = &hcan1,
    //      .tx_id = 0x23
    //    },
    //    .controller_param_init_config = {
    //        .angle_PID = {
    //             // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
    //             .Kp = 4.0, // 10
    //             .Ki = 0,
    //             .Kd = 8.0,
    //             .MaxOut = 20000,
    //         },
    //         .speed_PID = {
    //             .Kp = 2, // 0
    //             .Ki = 0., // 0
    //             .Kd = 0,
    //             .Improve = PID_Integral_Limit | PID_ErrorHandle,
    //             .IntegralLimit = 100,
    //             .MaxOut = 10000,
    //         },
    //     },
    //     .controller_setting_init_config = {
    //         .angle_feedback_source = MOTOR_FEED,
    //         .speed_feedback_source = MOTOR_FEED,
    //         .outer_loop_type = ANGLE_LOOP,
    //         .close_loop_type = ANGLE_LOOP,
    //         .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,  
    //         .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL 
    //     },
    // };
    // dmmotor_big_yaw = DMMotorInit(&dm8006);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    chassis_speed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

}

float degree;


/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    //read_can2_motor_ctrl_fbdata(0x08);
    //read_can1_motor_ctrl_fbdata(0x07);
    //read_can2_motor_ctrl_fbdata(0x08);
    //DMMotorSetRef(dmmotor_yaw1, 0.2f);
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    SubGetMessage(chassis_speed_sub, &chassis_real_speed);


    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        // DJIMotorStop(yaw_motor);
        // DJIMotorStop(pitch_motor);
        DMMotorStop(dmmotor_yaw1);
        DMMotorStop(dmmotor_yaw2);
        DMMotorStop(dmmotor_pitch);
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式

        DMMotorEnable(dmmotor_yaw1);
        DMMotorEnable(dmmotor_yaw2);
        DMMotorEnable(dmmotor_pitch);
        DMMotorSetRef(dmmotor_yaw1, gimbal_cmd_recv.yaw);
        DMMotorSetRef(dmmotor_pitch, gimbal_cmd_recv.pitch);

                           // DJIMotorSetFeedfoward(yaw_motor,SPEED_FEEDFORWARD);
        // DJIMotorEnable(yaw_motor);
        // DJIMotorEnable(pitch_motor);
        // DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        // DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        // DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        // DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        // DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        // DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);


        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        DMMotorEnable(dmmotor_yaw1);
        DMMotorEnable(dmmotor_yaw2);
        DMMotorEnable(dmmotor_pitch);
        DMMotorSetRef(dmmotor_yaw1, gimbal_cmd_recv.yaw);
        DMMotorSetRef(dmmotor_pitch, gimbal_cmd_recv.pitch);
        // DJIMotorSetFeedfoward(yaw_motor, FEEDFORWARD_NONE);
        // DJIMotorEnable(yaw_motor);
        // DJIMotorEnable(pitch_motor);
        // DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        // DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        // DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        // DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        // DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        // DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    default:
        DMMotorStop(dmmotor_yaw1);
        DMMotorStop(dmmotor_yaw2);
        DMMotorStop(dmmotor_pitch);
        break;
    }

    

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = dmmotor_yaw2->measure.position-dmmotor_yaw1->measure.position;
    // 首先转换为角度
    degree = gimbal_feedback_data.yaw_motor_single_round_angle * 180.0f / 3.1415926f;
    
    // 使用fmod函数规范化到 [-180, 180] 范围
    degree = fmodf(degree + 180.0f, 360.0f);
    if (degree < 0) {
        degree += 360.0f;
    }
    degree -= 180.0f;
    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
