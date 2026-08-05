#include "imu.h"
#include <math.h>

/*
 * IMU 上层数据处理：在 ICM45686 原始数据之上提供姿态融合和车体平面加速度。
 * 角速度输出单位为 °/s，加速度输出单位为 mg；TASK4 使用前后加速度做摆杆前馈，
 * 滚球位置闭环仍由 K230 视觉数据完成。
 */

/* 角度、弧度换算常数；姿态更新周期与 ICM45686 的 200 Hz 输出率一致。 */
#define DEG_TO_RAD 0.01745329251994329577f
#define RAD_TO_DEG 57.295779513082320876f
#define IMU_DT_S   0.005f

/* 平面加速度上电校准及输出滤波参数。 */
#define PLANAR_CALIBRATION_SAMPLES  200U
#define PLANAR_MIN_VALID_SAMPLES    180U
#define PLANAR_SAMPLE_INTERVAL_MS     5U
#define PLANAR_FILTER_ALPHA           0.10f
#define PLANAR_DEADBAND_MG            20.0f

/* Mahony 姿态四元数，初始状态表示传感器与参考坐标系重合。 */
static float q0 = 1.0f;
static float q1;
static float q2;
static float q3;

/* Mahony 反馈误差的三轴积分项，用于补偿陀螺仪慢性零偏。 */
static float integral_x;
static float integral_y;
static float integral_z;

static bool planar_calibrated;              /* 平面加速度零偏是否已校准。 */
static float front_back_offset_mg;          /* 静止时前后轴零偏，单位 mg。 */
static float left_right_offset_mg;          /* 静止时左右轴零偏，单位 mg。 */
static float front_back_filtered_mg;        /* 前后轴低通滤波结果，单位 mg。 */
static float left_right_filtered_mg;        /* 左右轴低通滤波结果，单位 mg。 */

/* 浮点平方根倒数，用于向量和四元数归一化。 */
static float InvSqrt(float value)
{
    return 1.0f / sqrtf(value);
}

/* 将姿态状态恢复到单位四元数，同时清空误差积分。 */
void IMU_Init(void)
{
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    integral_x = 0.0f;
    integral_y = 0.0f;
    integral_z = 0.0f;
}

/*
 * Mahony 六轴姿态更新：
 * 1. 将加速度归一化并作为重力方向观测值；
 * 2. 用观测重力和四元数预测重力的叉积得到姿态误差；
 * 3. 通过比例、积分反馈修正角速度；
 * 4. 积分四元数并重新归一化。
 */
static void IMU_MahonyUpdate(float gx, float gy, float gz,
                             float ax, float ay, float az)
{
    const float kp = 0.8f;
    const float ki = 0.002f;
    float norm;
    float vx;
    float vy;
    float vz;
    float ex;
    float ey;
    float ez;
    float half_dt = 0.5f * IMU_DT_S;
    float nq0;
    float nq1;
    float nq2;
    float nq3;

    norm = ax * ax + ay * ay + az * az;
    /* 加速度模长无效时无法得到重力方向，本次不更新姿态。 */
    if (norm <= 0.000001f) return;
    norm = InvSqrt(norm);
    ax *= norm; ay *= norm; az *= norm;

    /* 根据当前四元数计算机体系中预测的单位重力方向。 */
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    /* 实测重力与预测重力的叉积即为姿态修正误差。 */
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;
    integral_x += ki * ex * IMU_DT_S;
    integral_y += ki * ey * IMU_DT_S;
    integral_z += ki * ez * IMU_DT_S;
    gx += kp * ex + integral_x;
    gy += kp * ey + integral_y;
    gz += kp * ez + integral_z;

    /* 用修正后的角速度积分四元数，并保证四元数保持单位长度。 */
    nq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_dt;
    nq1 = q1 + ( q0 * gx + q2 * gz - q3 * gy) * half_dt;
    nq2 = q2 + ( q0 * gy - q1 * gz + q3 * gx) * half_dt;
    nq3 = q3 + ( q0 * gz + q1 * gy - q2 * gx) * half_dt;
    norm = InvSqrt(nq0 * nq0 + nq1 * nq1 + nq2 * nq2 + nq3 * nq3);
    q0 = nq0 * norm; q1 = nq1 * norm; q2 = nq2 * norm; q3 = nq3 * norm;
}

/* 读取传感器数据、执行一次姿态融合并输出 yaw/pitch/roll。 */
bool IMU_Update(float ypr_deg[3])
{
    ICM45686_Data data;
    float sin_pitch;

    if ((ypr_deg == NULL) || !ICM45686_Read(&data)) return false;
    IMU_MahonyUpdate(data.gyro_dps[0] * DEG_TO_RAD,
                     data.gyro_dps[1] * DEG_TO_RAD,
                     data.gyro_dps[2] * DEG_TO_RAD,
                     data.accel_mg[0], data.accel_mg[1], data.accel_mg[2]);

    /* 将姿态四元数转换为欧拉角，输出顺序为 yaw、pitch、roll。 */
    ypr_deg[0] = atan2f(2.0f * (q1 * q2 + q0 * q3),
                        1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
    sin_pitch = 2.0f * (q0 * q2 - q3 * q1);
    if (sin_pitch > 1.0f) sin_pitch = 1.0f;
    if (sin_pitch < -1.0f) sin_pitch = -1.0f;
    ypr_deg[1] = asinf(sin_pitch) * RAD_TO_DEG;
    ypr_deg[2] = atan2f(2.0f * (q0 * q1 + q2 * q3),
                        1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;
    return true;
}

/*
 * 车体静止时连续读取约 1 秒，把有效样本平均值作为 X/Y 平面零偏。
 * 200 个计划样本中至少需要 180 个有效样本，否则校准失败。
 */
bool IMU_PlanarCalibrate(void)
{
    ICM45686_Data data;
    float front_back_sum = 0.0f;
    float left_right_sum = 0.0f;
    uint16_t sample;
    uint16_t valid_samples = 0U;

    planar_calibrated = false;
    front_back_offset_mg = 0.0f;
    left_right_offset_mg = 0.0f;
    front_back_filtered_mg = 0.0f;
    left_right_filtered_mg = 0.0f;

    for (sample = 0U; sample < PLANAR_CALIBRATION_SAMPLES; sample++) {
        HAL_Delay(PLANAR_SAMPLE_INTERVAL_MS);
        if (ICM45686_Read(&data)) {
            /* 当前安装映射：ICM X轴=前后，Y轴=左右。 */
            front_back_sum += data.accel_mg[0];
            left_right_sum += data.accel_mg[1];
            valid_samples++;
        }
    }

    if (valid_samples < PLANAR_MIN_VALID_SAMPLES) {
        return false;
    }

    /* 用有效样本的平均值抵消安装倾斜及传感器静态零偏。 */
    front_back_offset_mg = front_back_sum / (float)valid_samples;
    left_right_offset_mg = left_right_sum / (float)valid_samples;
    planar_calibrated = true;
    return true;
}

/* 读取平面加速度，依次完成去零偏、一阶低通滤波和小信号死区。 */
bool IMU_PlanarUpdate(float *front_back_mg, float *left_right_mg)
{
    ICM45686_Data data;
    float front_back_raw;
    float left_right_raw;

    if (!planar_calibrated ||
        (front_back_mg == NULL) || (left_right_mg == NULL) ||
        !ICM45686_Read(&data)) {
        return false;
    }

    front_back_raw = data.accel_mg[0] - front_back_offset_mg;
    left_right_raw = data.accel_mg[1] - left_right_offset_mg;

    /* 一阶低通：alpha 越小越平稳，但响应越慢。 */
    front_back_filtered_mg +=
        PLANAR_FILTER_ALPHA *
        (front_back_raw - front_back_filtered_mg);
    left_right_filtered_mg +=
        PLANAR_FILTER_ALPHA *
        (left_right_raw - left_right_filtered_mg);

    /* 小于死区阈值的残余噪声直接输出为 0，避免静止时数值漂动。 */
    *front_back_mg =
        (fabsf(front_back_filtered_mg) < PLANAR_DEADBAND_MG) ?
        0.0f : front_back_filtered_mg;
    *left_right_mg =
        (fabsf(left_right_filtered_mg) < PLANAR_DEADBAND_MG) ?
        0.0f : left_right_filtered_mg;
    return true;
}
