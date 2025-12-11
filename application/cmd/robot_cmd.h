#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H


/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 * 
 */
void RobotCMDInit();

/**
 * @brief 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 * 
 */
void RobotCMDTask();

/**
 * @brief 将弧度转换为角度，并将范围限制在 [-180, 180]
 * @param radian 输入的弧度值
 * @return 角度值，范围在 [-180, 180]
 */
float radian_to_degree_180(float radian);



#endif // !ROBOT_CMD_H