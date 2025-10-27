#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>

// 定义卡尔曼滤波器结构体
typedef struct {
    float x[4]; // 状态向量 [x, y, vx, vy]
    float P[4][4]; // 状态协方差矩阵
    float Q[4][4]; // 过程噪声协方差矩阵
    float R[2][2]; // 测量噪声协方差矩阵
    float H[2][4]; // 测量矩阵
} KalmanFilter;

/**
 * @brief 初始化卡尔曼滤波器
 *
 * @param kf 指向 KalmanFilter 结构体的指针
 * @param initial_x 初始x坐标
 * @param initial_y 初始y坐标
 * @param process_noise 过程噪声
 * @param measurement_noise 测量噪声
 */
void kalman_init(KalmanFilter* kf, float initial_x, float initial_y, float process_noise, float measurement_noise);

/**
 * @brief 卡尔曼滤波器预测步骤
 *
 * @param kf 指向 KalmanFilter 结构体的指针
 * @param dt 时间间隔
 */
void kalman_predict(KalmanFilter* kf, float dt);

/**
 * @brief 卡尔曼滤波器更新步骤
 *
 * @param kf 指向 KalmanFilter 结构体的指针
 * @param measurement 测量值 [x, y]
 */
void kalman_update(KalmanFilter* kf, float measurement[2]);

#endif // KALMAN_H
