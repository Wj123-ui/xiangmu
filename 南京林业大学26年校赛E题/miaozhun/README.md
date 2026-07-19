# 自动瞄准及绘图装置工程说明

2026 南京林业大学校内电子设计竞赛 E 题  
MCU：STM32F407xx｜云台：双 TMC2209｜屏幕：ILI9341｜视觉：K230

## 1. 工程概述

本工程控制二维步进电机云台和激光笔，完成靶心瞄准、正弦曲线绘制、参数化曲线绘制、矩形轮廓跟踪、数字书写与线条复绘。STM32 负责菜单、运动控制、轨迹生成和 LCD 显示；K230 负责视觉检测并通过 UART4 提供误差或目标点。

当前已完成菜单优化、ILI9341 显示 180° 旋转、Task4 参数编辑与双击启动，以及 Task6 不依赖 K230 的开环数字 0～9 绘制。

重要提示：

- 激光具有伤眼风险，不得照射人眼、镜面或高反射物体。
- 开环绘图没有编码器或限位开关提供绝对位置反馈，运行前必须确认激光位于约定起点。
- Task5、Task7 仍依赖 K230 连续提供目标点。
- 默认靶面距离为 50 cm，改变距离后必须重新标定。

## 2. 硬件与引脚

| 功能 | STM32 引脚 | 说明 |
|---|---|---|
| 按键 1～4 | PE0、PE1、PE2、PE3 | 上拉输入，低电平有效 |
| 激光开关 | PE4 | 高电平开启 |
| LCD SPI1 | PA5 SCK、PA6 MISO、PA7 MOSI | 模式 0，8 位 |
| LCD 控制 | PA4 CS、PC4 DC、PC5 RST | ILI9341 240×320 |
| K230 UART4 | PA0 TX、PA1 RX | 115200 8N1 |
| Yaw TMC2209 | PA2/PA3 UART、PC6 STEP、PB12 DIR、PB13 EN | TIM3_CH1 |
| Pitch TMC2209 | PB10/PB11 UART、PC7 STEP、PB14 DIR、PB15 EN | TIM3_CH2 |

电机按 200 整步/转、1:1 直驱配置，默认 1/16 微步，即 3200 脉冲/转、0.1125°/脉冲。

## 3. 软件结构

    main.c
      ├─ 外设、LCD、TMC2209、UART 初始化
      └─ 主循环：show_eum → eum_task_dispatch

    TIM1 1ms 中断
      ├─ 按键 5ms 采样、20ms 去抖
      └─ 双轴速度斜坡更新

    UART4 中断
      └─ 解析 K230 数据帧

| 文件 | 作用 |
|---|---|
| Core/Src/main.c | 外设初始化和主循环 |
| Core/Src/eum.c | 按键、菜单、LCD 菜单和任务调度 |
| Core/Src/task.c | Task1～Task7、PID 和开环轨迹 |
| Core/Src/control.c | TMC2209、STEP/DIR、VACTUAL 和激光控制 |
| Core/Src/opmv.c | K230 通信协议 |
| Core/Src/ili9341.c | LCD 驱动与图形绘制 |
| Core/Inc/task.h | 几何、时间、Task4/Task6 参数 |

## 4. 菜单操作

主菜单：

- 按键 1 / 2：选择上一项或下一项。
- 按键 3：任务 1、2、3、5、6、7 直接启动；任务 4 进入参数页。
- 任务运行中按键 3：停止并返回主菜单。

Task4 参数页：

1. 未编辑时，按键 1 选择幅值 A，按键 2 选择相位 P。
2. 按键 4 进入编辑，按键 1 增大、按键 2 减小。
3. 再按按键 4 保存并退出编辑。
4. 400 ms 内快速按两次按键 3 启动。
5. 单击按键 3 或第二次超时则返回主菜单。

幅值选项：3.0、5.0、7.5、10.0 cm。  
相位选项：0°、30°、60°、90°、120°、150°、180°。

## 5. 任务状态

| 任务 | 功能 | 当前实现 |
|---|---|---|
| Task1 | 2 s 内瞄准靶心 | K230 误差 + PID |
| Task2 | 绘制两周期 y=sin(x) | PID 对心后开环绘制，22 cm、峰峰值 15 cm |
| Task3 | 4 s 内瞄准靶心 | K230 误差 + PID |
| Task4 | 绘制 y=A sin(x+P) | A/P 可调，PID 对心，LCD 同步显示 |
| Task5 | 矩形轮廓和尺寸 | 依赖 K230 轮廓点；0.1 cm/像素为临时值 |
| Task6 | 激光写数字 0～9 | 双轴 VACTUAL 开环七段式；默认数字 6 |
| Task7 | 复绘手绘线条 | 依赖 K230 线条点，20 s 超时 |

## 6. Task6 参数

参数位于 Core/Inc/task.h：

    TASK6_DEFAULT_DIGIT       默认数字，当前为 6
    TASK6_X_OFFSET_CM         数字中心右移量，当前 6.5 cm
    TASK6_HALF_WIDTH_CM       半宽，当前 3.0 cm
    TASK6_HALF_HEIGHT_CM      半高，当前 5.0 cm
    TASK6_MOVE_SPEED_CM_S     关激光移笔速度
    TASK6_DRAW_SPEED_CM_S     开激光绘制速度
    TASK6_YAW_SIGN            水平方向符号
    TASK6_PITCH_SIGN          垂直方向符号

运行前也可直接设置 task6_digit 为 0～9。若左右反向，修改 TASK6_YAW_SIGN；若上下反向，修改 TASK6_PITCH_SIGN。

Task6 假定进入任务时激光位于靶心。每笔按“关激光移动到起点 → 开激光匀速画线 → 关激光抬笔”执行。

## 7. K230 协议

STM32 向 K230：

    #<task_id>\n

K230 向 STM32：

    $<type>,<p1>,<p2>,<p3>,<p4>\n

| type | 用途 | 参数 |
|---|---|---|
| A | Task1～4 瞄准 | p1=err_x，p2=err_y |
| C | Task5 矩形轮廓 | p1=x，p2=y，p3=status |
| D | Task6 数字识别 | p1=digit；当前 Task6 暂不读取 |
| L | Task7 线条点 | p1=x，p2=y，p3=status |
| R | K230 就绪 | 可无参数 |

## 8. 编译

要求 CMake、Ninja 和 GNU Arm Embedded Toolchain 已加入 PATH。

    cd D:\xiangmu\miaozhun
    cmake --preset Debug
    cmake --build --preset Debug

若旧缓存指向其他目录，可使用：

    cmake -S . -B build/CodexDebug -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
    cmake --build build/CodexDebug

当前验证固件位于 build/CodexDebug/yuntai.elf、yuntai.hex、yuntai.bin。

## 9. 实机调试顺序

1. 断开激光或使用低功率指示光，检查电源和公共地。
2. 检查 LCD 方向、文字和四个按键。
3. 分别低速测试 Yaw/Pitch，使机构远离限位。
4. 检查 TMC2209 UART、微步和使能电平。
5. 标定 K230 误差正方向和 PID 符号。
6. 先测试 Task1/Task3，再测试 Task2/Task4。
7. 将激光置于靶心后测试 Task6，先画 1，再画 8。
8. 调整 Task6 的方向、偏移、宽高和速度。
9. 最后联调 Task5/Task7。

## 10. 已知限制

- 尚无自动回零、限位开关或绝对编码器，开环任务可能因丢步产生偏移。
- Task5 的像素到厘米比例尚未正式标定。
- Task6 当前为七段式字体；以后可替换为多段折线字体。
- K230 协议缺少校验、超时和丢帧恢复。
- task.c 中仍有少量未使用变量/常量编译警告。

## 11. 最近验证

- GNU Arm Embedded GCC 13.3.1，Debug 完整编译和链接通过。
- Flash 约 39.6 KB / 512 KB。
- RAM 约 3.3 KB / 128 KB；CCMRAM 2 KB / 64 KB。
- 固件输出：build/CodexDebug/yuntai.elf、yuntai.hex、yuntai.bin。

