#include "shoot.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "servo_motor.h"
#include "cmsis_os.h"
#include "vofa.h"
/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l; // 左摩擦轮
static DJIMotorInstance *friction_r; // 右摩擦轮
static DJIMotorInstance *loader;     // 拨盘电机
DMMotorInstance *dmmotor_loader;

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自gimbal_cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自gimbal_cmd的发射控制信息

// 拨弹盘前馈
static float loader_current_forward = 0;

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;
static float load_angle_set = 0;

float shootData[2]= {0};
float L_speed = 0, R_speed= 0;
int count1 =0;
void ShootInit()
{
    // 左摩擦轮,两个can2
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan3,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 16, // 20
                .Ki = 0, // 1
                .Kd = 0,
                .Derivative_LPF_RC = 0.02,
                .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral | PID_DerivativeFilter,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 1.5, // 0.7
                .Ki = 0,   // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },

        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,

        },
        .motor_type = M3508,
    };
    friction_config.can_init_config.tx_id = 1; // 左摩擦轮
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 8; // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_r = DJIMotorInit(&friction_config);

    // // 拨盘电机
    // Motor_Init_Config_s loader_config = {
    //     .can_init_config = {
    //         .can_handle = &hcan2,
    //         .tx_id = 5,
    //     },
    //     .controller_param_init_config = {
    //         .angle_PID = {
    //             // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
    //             .Kp = 10, // 10
    //             .Ki = 0,
    //             .Kd = 0,
    //             .MaxOut = 20000,
    //         },
    //         .speed_PID = {
    //             .Kp = 0.5, // 0
    //             .Ki = 0.1, // 0
    //             .Kd = 0.002,
    //             .Improve = PID_Integral_Limit | PID_ErrorHandle,
    //             .IntegralLimit = 200,
    //             .MaxOut = 10000,
    //         },
    //         .current_PID = {
    //             .Kp = 1.5, // 0
    //             .Ki = 0,   // 0
    //             .Kd = 0,
    //             .Improve = PID_Integral_Limit,
    //             .IntegralLimit = 5000,
    //             .MaxOut = 10000,
    //         },

    //         .current_feedforward_ptr = &loader_current_forward,
    //     },
    //     .controller_setting_init_config = {
    //         .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
    //         .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
    //         .close_loop_type = ANGLE_AND_SPEED_LOOP,
    //         .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向

    //         .feedforward_flag = CURRENT_FEEDFORWARD,
    //     },
    //     .motor_type = M2006 // 英雄使用m3508
    // };
    // loader = DJIMotorInit(&loader_config);

    Motor_Init_Config_s dm4310_loader={
       .can_init_config ={
         .can_handle = &hcan1,
         .tx_id = 5 ,
         .rx_id = 15
       },
       .controller_param_init_config = {
        //    .angle_PID = {
        //         .Kp = 0.2,           // 减小Kp从0.2到0.15
        //         .Ki = 0.1,              // 保持或减小
        //         .Kd = 0.004,      // 0.006
        //         .CoefA = 0.5,          // 0.5
        //         .CoefB = 0.6,          // 0.6
        //         .DeadBand = 1,     // 0.005
        //         .Output_LPF_RC = 0.01, // 0.01
        //         .Improve = PID_Trapezoid_Intergral | PID_ChangingIntegrationRate | PID_Integral_Limit | PID_OutputFilter | PID_DerivativeFilter,
        //         .IntegralLimit = 1, // 1
        //         .MaxOut = 45,      // 600
        //     },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,          //这里改成前馈计算和使用的标志位了，不是原来的意思，ANGLE_LOOP是被计算反馈，SPEED_LOOP是使用反馈
            .close_loop_type = ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,  
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL 
        },
        .motor_type = set_zero
    };
    dmmotor_loader = DMMotorInit(&dm4310_loader);

    DMMotorSetRef(dmmotor_loader,0.0f);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}


/* 机器人发射机构控制核心任务 */
int count=0;
float angle=0.0f;

void ShootTask()
{   
    
    
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);
    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
         DJIMotorStop(friction_l);
         DJIMotorStop(friction_r);
        // DJIMotorStop(loader);
         DMMotorStop(dmmotor_loader);
        //count=0;
    }
    else // 恢复运行
    {
        DMMotorEnable(dmmotor_loader);
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        if(shoot_cmd_recv.load_mode==LOAD_1_BULLET)
        {
            if(count==shoot_cmd_recv.shoot_rate)
            {
                count=0;angle+=2.110746f;
                if(angle>=5)
                {  
                    angle=2.110746f;
                }
                DMMotorSetRef(dmmotor_loader, angle);
            }
            count++;
        }
        else if(shoot_cmd_recv.load_mode==LOAD_STOP)
        {
            count=shoot_cmd_recv.shoot_rate;
        }
        else if(shoot_cmd_recv.load_mode==LOAD_REVERSE)
        {
            DMMotorSetRef(dmmotor_loader, 0.0f);
        }
        
    
        if (shoot_cmd_recv.friction_mode == FRICTION_ON)
        {
            //根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
            switch (shoot_cmd_recv.bullet_speed)
            {
            case SMALL_AMU_15:
                DJIMotorSetRef(friction_l, 26800);
                DJIMotorSetRef(friction_r, 26800);
                break;
            case SMALL_AMU_18:
                DJIMotorSetRef(friction_l, 30000);
                DJIMotorSetRef(friction_r, 30000);
                break;
            case SMALL_AMU_30:
                DJIMotorSetRef(friction_l, 46500);
                DJIMotorSetRef(friction_r, 46500);
                break;
            default: // 当前为了调试设定的默认值40000,因为还没有加入裁判系统无法读取弹速.
                 // DJIMotorSetRef(friction_l, 26800);
                 // DJIMotorSetRef(friction_r, 26800);
                DJIMotorSetRef(friction_l, 40000);
                DJIMotorSetRef(friction_r, 40000);
                break;
            }
            DJIMotorSetRef(friction_l, 30000);
            DJIMotorSetRef(friction_r, 30000);
        }
        else // 关闭摩擦轮/
        {
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
        }
    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    // if (hibernate_time + dead_time > DWT_GetTimeline_ms())
    //     return;

    // if (shoot_cmd_recv.rest_heat >= 120 || shoot_cmd_recv.bullet_speed == 0)
    // {
    //     // 堵转处理
    //     static uint16_t loader_error_cnt = 0;
    //     if (loader->motor_controller.speed_PID.ERRORHandler.ERRORType == PID_MOTOR_BLOCKED_ERROR)
    //     {
    //         loader_error_cnt++;

    //         if (loader_error_cnt < 3 && loader_error_cnt >= 1)
    //             shoot_cmd_recv.load_mode = LOAD_REVERSE;
    //         // 超过堵转次数，恢复正转
    //     }
    //     else
    //     {
    //         loader_error_cnt = 0;
    //     }

    //     switch (shoot_cmd_recv.load_mode)
    //     {
    //     case LOAD_STOP:
    //         DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
    //         DJIMotorSetRef(loader, 0);
    //         break;
    //     // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    //     case LOAD_1_BULLET: // 激活能量机关/干扰对方用,英雄用.
    //         load_angle_set = loader->measure.total_angle + (360.f / NUM_PER_CIRCLE * REDUCTION_RATIO_LOADER);
    //         DJIMotorOuterLoop(loader, ANGLE_LOOP);  // 切换到角度环
    //         DJIMotorSetRef(loader, load_angle_set); // 控制量增加一发弹丸的角度
    //         hibernate_time = DWT_GetTimeline_ms();  // 记录触发指令的时间
    //         dead_time = 500;                        // 完成1发弹丸发射的时间
    //         break;
    //     // 三连发,如果不需要后续可能删除
    //     case LOAD_3_BULLET:
    //         load_angle_set = loader->measure.total_angle + (360.f / NUM_PER_CIRCLE * REDUCTION_RATIO_LOADER * 3);
    //         DJIMotorOuterLoop(loader, ANGLE_LOOP);  // 切换到角度环
    //         DJIMotorSetRef(loader, load_angle_set); // 增加3发
    //         hibernate_time = DWT_GetTimeline_ms();  // 记录触发指令的时间
    //         dead_time = 1800;                       // 完成3发弹丸发射的时间
    //         break;
    //     // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
    //     case LOAD_BURSTFIRE:
    //         DJIMotorOuterLoop(loader, SPEED_LOOP);
    //         DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * (360.f * REDUCTION_RATIO_LOADER / NUM_PER_CIRCLE));
    //         // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
    //         break;
    //     // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
    //     // 也有可能需要从switch-case中独立出来
    //     case LOAD_REVERSE:
    //         load_angle_set = loader->measure.total_angle - (360.f / NUM_PER_CIRCLE * REDUCTION_RATIO_LOADER); // Enable load angle set calculation
    //         DJIMotorOuterLoop(loader, ANGLE_LOOP);                                                          // Change back to angle loop for reverse loading
    //         DJIMotorSetRef(loader, load_angle_set);
    //         hibernate_time = DWT_GetTimeline_ms(); // 记录触发指令的时间
    //         dead_time = 100;                       // 完成1发弹丸发射的时间
    //         // ...
    //         break;
    //     default:
    //         while (1)
    //             ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    //     }
    // // }
    // else
    // {
    //     DJIMotorOuterLoop(loader, SPEED_LOOP);
    //     DJIMotorSetRef(loader, 0);
    // }

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
    }
    L_speed =friction_l->measure.speed_aps , R_speed = friction_r->measure.speed_aps;
    shootData[0] = -L_speed;
    shootData[1] = R_speed;
    vofa_justfloat_output(shootData, 2, &huart10);
}
