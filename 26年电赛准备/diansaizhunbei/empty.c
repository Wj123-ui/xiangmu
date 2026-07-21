/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "A4950.h" 
#include "board.h"
#include "oled.h"
#include "bmp.h"
#include "IMU.h"
#include "Encoder.h"
#include "Timer.h"
#include "PID.h"
#include "xunji.h"
#include "task.h"
volatile bool gIcmDataReady = false;
float icm_ypr[3] = {0};
PID_Controller left_speed_pid;
PID_Controller right_speed_pid;
PID_Controller dir_speed_pid;
PID_Controller angle_pid;
static void ICM_ApplyStartupZero(float ypr[3])//初始化置0
{
    static float zero[3] = {0};
    static uint16_t sampleCount = 0;
    static bool zeroReady = false;

    if (!zeroReady) {
        sampleCount++;
        if (sampleCount >= 100U) {
            zero[0] = ypr[0];
            zero[1] = ypr[1];
            zero[2] = ypr[2];
            zeroReady = true;
        }

        ypr[0] = 0.0f;
        ypr[1] = 0.0f;
        ypr[2] = 0.0f;
        return;
    }

    ypr[0] -= zero[0];
    ypr[1] -= zero[1];
    ypr[2] -= zero[2];

    if (ypr[0] > 180.0f) {
        ypr[0] -= 360.0f;
    } else if (ypr[0] < -180.0f) {
        ypr[0] += 360.0f;
    }
}
void PID_all_init(void)
{
    PID_Init(&left_speed_pid,3,0.05,0,-500,500);
    PID_Init(&right_speed_pid,3,0.05,0,-500,500);
    PID_Init(&dir_speed_pid,0.2,0.05,0,-200,200);
    PID_Init(&angle_pid,0,0,0,-100,100);
}
void PID_all_update(void)
{
    PID_Update(&left_speed_pid,(float)WJ_get_left_speed());
    PID_Update(&right_speed_pid,(float)WJ_get_right_speed());
    PID_Update(&dir_speed_pid,(float)WJ_get_left_speed()-(float)WJ_get_right_speed());
    PID_Update(&angle_pid,(float)icm_ypr[0]);
    WJ_Set_leftspeed(left_speed_pid.output+dir_speed_pid.output);
    WJ_Set_rightspeed(right_speed_pid.output-dir_speed_pid.output);
}
void xunji_set_speed(float a)
{
    PID_SetSetpoint(&left_speed_pid,150+a);
    PID_SetSetpoint(&right_speed_pid,150-a);
    PID_SetSetpoint(&dir_speed_pid,a*2);
}
void xunji_yuandizuozhuan(float a)
{
	/* 两轮等速反向，a<0 时原地左转。 */
    PID_SetSetpoint(&left_speed_pid,a);
    PID_SetSetpoint(&right_speed_pid,-a);
    PID_SetSetpoint(&dir_speed_pid,a*2.0f);
}
int main(void)
{
    uint8_t displayDivider = 0;
    SYSCFG_DL_init();
    DL_GPIO_setPins(ICM_CS_PORT, ICM_CS_ICM_CS_OUT_PIN);
    delay_cycles(40000000);
    OLED_Init();
    delay_cycles(40000000);
    delay_ms(100);
    IMU_init();
    gIcmDataReady = false;
    DL_GPIO_clearInterruptStatus(ICM_INT_PORT, ICM_INT_ICM_DRDY_PIN);
    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(TIMG12_INT_IRQn);
    PID_all_init();
    while (1) 
    {
        //---------------------------------------------------------------------------------------//
        if (gIcmDataReady) {
            gIcmDataReady = false;
            IMU_getYawPitchRoll(icm_ypr);
            ICM_ApplyStartupZero(icm_ypr);
            /* ICM ODR is 200 Hz; refresh telemetry/display every 10 samples. */
            if (++displayDivider >= 10U) {
                displayDivider = 0; 
                OLED_ShowString(0, 0, (uint8_t *)"YAW:", 8, 1);
                OLED_ShowFloat(30, 0, icm_ypr[0], 3, 2, 8, 1);
            }
        }
        //---------------------------------------------------------------------------------------//


        WJ_task_zhiujiao();
         

        //--------------------------------------------------//
        /* 编码器速度每 20 ms 更新一次，PID 也按相同节拍计算。 */
        if(speed_update_ready!=0U)
        {
            speed_update_ready=0U;
            PID_all_update();
        }
        //--------------------------------------------------//


        OLED_ShowSignedNum(0,8,WJ_get_left_speed(),4,8,1);
        OLED_ShowSignedNum(0,16,WJ_get_right_speed(),4,8,1);
        OLED_Refresh(); 
    }
}
