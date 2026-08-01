#ifndef IMU_H
#define IMU_H

#include "main.h"
#include <stdbool.h>

/* ICM45686 一次采样得到的物理量，数组下标 0/1/2 对应芯片 X/Y/Z 轴。 */
typedef struct {
    float accel_mg[3];    /* 三轴加速度，单位 mg，1 g = 1000 mg。 */
    float gyro_dps[3];    /* 三轴角速度，单位 °/s。 */
    float temperature_c;  /* 芯片内部温度，单位 ℃。 */
} ICM45686_Data;

/* 初始化 ICM45686 SPI 驱动、量程、输出频率及数据就绪中断。 */
bool ICM45686_Init(SPI_HandleTypeDef *hspi);

/* 读取一组加速度、角速度和温度数据，并转换成上述物理单位。 */
bool ICM45686_Read(ICM45686_Data *data);

/* 在 IMU 数据就绪引脚的 EXTI 回调中调用，只设置就绪标志。 */
void ICM45686_DataReadyIRQ(void);

/* 查询数据就绪标志；实际读取成功后该标志会被清除。 */
bool ICM45686_IsDataReady(void);

/* 清零 Mahony 姿态解算所使用的四元数及积分误差。 */
void IMU_Init(void);

/*
 * 读取 IMU 并更新 Mahony 姿态，输出 yaw/pitch/roll，单位均为度。
 * 成功读取并更新时返回 true；参数无效或 SPI 读取失败时返回 false。
 */
bool IMU_Update(float ypr_deg[3]);

/*
 * 静止采样并标定车体平面 X/Y 轴零偏；校准期间车体必须保持静止。
 * 当前安装映射为 ICM X=前后、ICM Y=左右。
 */
bool IMU_PlanarCalibrate(void);

/* 输出已经去零偏、低通滤波及死区处理的前后/左右加速度，单位 mg。 */
bool IMU_PlanarUpdate(float *front_back_mg, float *left_right_mg);

#endif
