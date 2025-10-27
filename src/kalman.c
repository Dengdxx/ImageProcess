#include "kalman.h"
#include <string.h>

// --- Optimized Matrix Operations for Kalman Filter (4x4, 4x1, 2x2, 2x1) ---

// 4x4 * 4x1 (vector) -> 4x1
static void mat_mul_4x4_4x1(float res[4], const float A[4][4], const float B[4]) {
    res[0] = A[0][0]*B[0] + A[0][1]*B[1] + A[0][2]*B[2] + A[0][3]*B[3];
    res[1] = A[1][0]*B[0] + A[1][1]*B[1] + A[1][2]*B[2] + A[1][3]*B[3];
    res[2] = A[2][0]*B[0] + A[2][1]*B[1] + A[2][2]*B[2] + A[2][3]*B[3];
    res[3] = A[3][0]*B[0] + A[3][1]*B[1] + A[3][2]*B[2] + A[3][3]*B[3];
}

// 4x4 * 4x4 -> 4x4
static void mat_mul_4x4_4x4(float res[4][4], const float A[4][4], const float B[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            res[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j] + A[i][3]*B[3][j];
        }
    }
}

// 4x4 + 4x4 -> 4x4
static void mat_add_4x4_4x4(float res[4][4], const float A[4][4], const float B[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            res[i][j] = A[i][j] + B[i][j];
        }
    }
}

// 4x4 - 4x4 -> 4x4
static void mat_sub_4x4_4x4(float res[4][4], const float A[4][4], const float B[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            res[i][j] = A[i][j] - B[i][j];
        }
    }
}

// 4x4 Transpose -> 4x4
static void mat_trans_4x4(float res[4][4], const float A[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            res[i][j] = A[j][i];
        }
    }
}

// 2x4 * 4x4 -> 2x4
static void mat_mul_2x4_4x4(float res[2][4], const float A[2][4], const float B[4][4]) {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 4; j++) {
            res[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j] + A[i][3]*B[3][j];
        }
    }
}

// 4x4 * 4x2 -> 4x2
static void mat_mul_4x4_4x2(float res[4][2], const float A[4][4], const float B[4][2]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 2; j++) {
            res[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j] + A[i][3]*B[3][j];
        }
    }
}

// 2x4 * 4x2 -> 2x2
static void mat_mul_2x4_4x2(float res[2][2], const float A[2][4], const float B[4][2]) {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            res[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j] + A[i][3]*B[3][j];
        }
    }
}

// 2x2 + 2x2 -> 2x2
static void mat_add_2x2_2x2(float res[2][2], const float A[2][2], const float B[2][2]) {
    res[0][0] = A[0][0] + B[0][0];
    res[0][1] = A[0][1] + B[0][1];
    res[1][0] = A[1][0] + B[1][0];
    res[1][1] = A[1][1] + B[1][1];
}

// 2x2 Transpose -> 2x2 (same as original for 2x2)
static void mat_trans_2x2(float res[2][2], const float A[2][2]) {
    res[0][0] = A[0][0];
    res[0][1] = A[1][0];
    res[1][0] = A[0][1];
    res[1][1] = A[1][1];
}

// 2x2 Inverse
static int mat_inv_2x2(float res[2][2], const float A[2][2]) {
    float det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    if (det == 0.0f) return -1; // Singular matrix
    float inv_det = 1.0f / det;
    res[0][0] = A[1][1] * inv_det;
    res[0][1] = -A[0][1] * inv_det;
    res[1][0] = -A[1][0] * inv_det;
    res[1][1] = A[0][0] * inv_det;
    return 0;
}

// 4x1 + 4x1 -> 4x1
static void vec_add_4x1(float res[4], const float A[4], const float B[4]) {
    res[0] = A[0] + B[0];
    res[1] = A[1] + B[1];
    res[2] = A[2] + B[2];
    res[3] = A[3] + B[3];
}

// 2x1 - 2x1 -> 2x1
static void vec_sub_2x1(float res[2], const float A[2], const float B[2]) {
    res[0] = A[0] - B[0];
    res[1] = A[1] - B[1];
}

// 4x2 * 2x1 -> 4x1
static void mat_mul_4x2_2x1(float res[4], const float A[4][2], const float B[2]) {
    res[0] = A[0][0]*B[0] + A[0][1]*B[1];
    res[1] = A[1][0]*B[0] + A[1][1]*B[1];
    res[2] = A[2][0]*B[0] + A[2][1]*B[1];
    res[3] = A[3][0]*B[0] + A[3][1]*B[1];
}

// 2x4 * 4x1 -> 2x1
static void mat_mul_2x4_4x1(float res[2], const float A[2][4], const float B[4]) {
    res[0] = A[0][0]*B[0] + A[0][1]*B[1] + A[0][2]*B[2] + A[0][3]*B[3];
    res[1] = A[1][0]*B[0] + A[1][1]*B[1] + A[1][2]*B[2] + A[1][3]*B[3];
}


void kalman_init(KalmanFilter* kf, float initial_x, float initial_y, float process_noise, float measurement_noise) {
    // 初始化状态向量 [x, y, vx, vy]
    kf->x[0] = initial_x;
    kf->x[1] = initial_y;
    kf->x[2] = 0.0f;
    kf->x[3] = 0.0f;

    // 初始化状态协方差矩阵 P (较大的不确定性)
    memset(kf->P, 0, sizeof(kf->P));
    kf->P[0][0] = 1.0f;
    kf->P[1][1] = 1.0f;
    kf->P[2][2] = 1000.0f;
    kf->P[3][3] = 1000.0f;

    // 初始化过程噪声协方差矩阵 Q
    memset(kf->Q, 0, sizeof(kf->Q));
    float q = process_noise;
    kf->Q[0][0] = q;
    kf->Q[1][1] = q;
    kf->Q[2][2] = q;
    kf->Q[3][3] = q;

    // 初始化测量噪声协方差矩阵 R
    memset(kf->R, 0, sizeof(kf->R));
    kf->R[0][0] = measurement_noise;
    kf->R[1][1] = measurement_noise;

    // 初始化测量矩阵 H
    memset(kf->H, 0, sizeof(kf->H));
    kf->H[0][0] = 1.0f;
    kf->H[1][1] = 1.0f;
}

void kalman_predict(KalmanFilter* kf, float dt) {
    // 状态转移矩阵 F
    float F[4][4] = {
        {1.0f, 0.0f, dt,   0.0f},
        {0.0f, 1.0f, 0.0f, dt  },
        {0.0f, 0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}
    };

    // 预测状态: x_hat = F * x
    float x_hat[4];
    mat_mul_4x4_4x1(x_hat, F, kf->x);
    memcpy(kf->x, x_hat, sizeof(x_hat));

    // 预测协方差: P = F * P * F^T + Q
    float F_T[4][4];
    mat_trans_4x4(F_T, F);
    float temp_P[4][4];
    mat_mul_4x4_4x4(temp_P, F, kf->P);
    mat_mul_4x4_4x4(kf->P, temp_P, F_T);
    mat_add_4x4_4x4(kf->P, kf->P, kf->Q);
}

void kalman_update(KalmanFilter* kf, float measurement[2]) {
    // 计算卡尔曼增益 K = P * H^T * (H * P * H^T + R)^-1
    float H_T[4][2];
    // H is 2x4, H_T is 4x2
    H_T[0][0] = kf->H[0][0]; H_T[0][1] = kf->H[1][0];
    H_T[1][0] = kf->H[0][1]; H_T[1][1] = kf->H[1][1];
    H_T[2][0] = kf->H[0][2]; H_T[2][1] = kf->H[1][2];
    H_T[3][0] = kf->H[0][3]; H_T[3][1] = kf->H[1][3];

    float P_HT[4][2];
    mat_mul_4x4_4x2(P_HT, kf->P, H_T);

    float H_P_HT[2][2];
    mat_mul_2x4_4x2(H_P_HT, kf->H, P_HT);

    float S[2][2];
    mat_add_2x2_2x2(S, H_P_HT, kf->R);

    float S_inv[2][2];
    if (mat_inv_2x2(S_inv, S) != 0) {
        // 矩阵不可逆，跳过更新
        return;
    }

    float K[4][2]; // Kalman Gain
    // Corrected: K = P_HT * S_inv (4x2 * 2x2 -> 4x2)
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 2; j++) {
            K[i][j] = P_HT[i][0]*S_inv[0][j] + P_HT[i][1]*S_inv[1][j];
        }
    }

    // 更新状态: x = x + K * (z - H * x)
    float H_x[2];
    mat_mul_2x4_4x1(H_x, kf->H, kf->x);

    float y[2]; // Innovation
    vec_sub_2x1(y, measurement, H_x);

    float K_y[4];
    mat_mul_4x2_2x1(K_y, K, y);

    vec_add_4x1(kf->x, kf->x, K_y);

    kf->x[0] = kf->x[0];
    kf->x[1] = kf->x[1];

    // 更新协方差: P = (I - K * H) * P
    float K_H[4][4];
    // Corrected: K_H = K * H (4x2 * 2x4 -> 4x4)
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            K_H[i][j] = K[i][0]*kf->H[0][j] + K[i][1]*kf->H[1][j];
        }
    }

    float I_KH[4][4];
    float I[4][4] = {{1.0f,0.0f,0.0f,0.0f},{0.0f,1.0f,0.0f,0.0f},{0.0f,0.0f,1.0f,0.0f},{0.0f,0.0f,0.0f,1.0f}};
    mat_sub_4x4_4x4(I_KH, I, K_H);

    float temp_P[4][4];
    memcpy(temp_P, kf->P, sizeof(temp_P));
    mat_mul_4x4_4x4(kf->P, I_KH, temp_P);
}