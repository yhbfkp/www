#ifndef __CAR_CONFIG_H
#define __CAR_CONFIG_H

/* ==========================================================================
 * 循迹小车 - 参数总表  (2026-09-27 定版: 环形赛道 A->B / A->A)
 *
 * 算法来源: car-main v1.3 (2024电赛H题完赛代码) —— 加权差比和 + 转向PD + 增量式速度PID
 *           + 角度环pid_angle + HARD_BRAKE 急停
 * 数值全部照搬 v1.3 原值, 只做两处硬件适配:
 *   1) duty 刻度: v1.3 MAX_DUTY=50000 -> 本工程 TIM1(Period=7199) CCR, 在 motorA_duty() 内换算
 *   2) 编码器: v1.3 用 EXTI 单相下降沿(1计数/脉冲), F103 TIM 编码器模式 TI12 是4倍频
 *      -> 读数 /4 后再喂 PID, 保证速度量级与原版一致 (见 ENC_DIV)
 *
 * 想调参直接改这里; 标 [可调] 的也是 volatile 全局变量, Keil debug 里能直接改。
 * ========================================================================== */

/* ======================= 模块开关 ======================= */
#define CAR_USE_MOTOR       1   /* 0=完全不输出PWM(调试安全) */
#define CAR_USE_ENCODER     1
#define CAR_USE_GRAY        1   /* 芯流 T8 八路灰度 (硬件I2C2 PB10/PB11) */
#define CAR_USE_MPU         1   /* MPU6050 (软I2C PA6/PA7) */
#define CAR_USE_ULTRA       0   /* 超声波: 板上座子没走线, 必须飞线 PB0/PB1 才能开 */
#define CAR_USE_OLED        1
#define CAR_USE_KEY         1
#define CAR_USE_PRINT       1   /* 串口打印 (printf -> USART1 PA9/PA10 115200) */
#define CAR_USE_PLOT        0   /* 串口CSV波形输出(给 SerialPlot 看速度环) */

/* ======================= PWM / 控制周期 ======================= */
#define PWM_MAX             7199    /* TIM1 Period=7199, CCR 满值 */
#define MAX_DUTY            50000   /* 归一化 duty 满量程 (car-main 原值, PID输出上限) */

#define CTRL_TICK_MS        10      /* TIM3 心跳 */
#define CTRL_PID_DIV        10      /* 每 10 次心跳跑一次速度环 = 100ms (car-main 原周期) */

/* ======================= 电机 ======================= */
/* 网表: A路 PWMA=PA11(TIM1_CH4) AIN1=PB13 AIN2=PB12
         B路 PWMB=PA8 (TIM1_CH1) BIN1=PB14 BIN2=PB15 */
#define MOTOR_L_IS_A        1       /* 1=左轮接A路; 左右反了改 0 */
#define MOTOR_L_DIR         1       /* 转反了改 -1 */
#define MOTOR_R_DIR         1

/* ======================= 速度 (car-main 单位: 每100ms的等效单相编码器计数) ====== */
#define SPEED_FAST          45      /* [可调] 直道/基础速度 */
#define SPEED_SLOW          25      /* 丢线搜索速度 */
#define SPEED_MIN           2       /* 内轮最低速(调小=差速大) */

/* ======================= 编码器 ======================= */
#define ENC_DIV             4       /* TIM 编码器4倍频 -> 等效原版单相计数, 必须除 4 */
#define ENC_L_DIR           1       /* 读出来是负数就改 -1 (在 car_encoder.c) */
#define ENC_R_DIR           1
#define ENC_LINES           13      /* 编码器线数   —— 按你的电机改 */
#define ENC_REDUCTION       20.0f   /* 减速比       —— 按你的电机改 */
#define WHEEL_MM            65.0f   /* 轮径 mm      —— 按你的车轮改 */
/* 4倍频计数/转: 13*20*4 = 1040 */
#define ENC_CPR             ((float)ENC_LINES * ENC_REDUCTION * (float)ENC_DIV)
/* 每 mm 对应多少个原始计数 = 1040 / (PI*65) ≈ 5.09 */
#define ENC_CNT_PER_MM      (ENC_CPR / (3.14159265f * WHEEL_MM))

/* ======================= 循迹 (car-main gray_track) ======================= */
/* 加权差比和权重: 外层权重大, 偏得越多贡献越大 */
#define W1 6
#define W2 4.2
#define W3 1.8
#define W4 0
#define W5 0
#define W6 1.8
#define W7 4.2
#define W8 6

#define TRACK_KP            5.8f    /* [可调] 转向P (误差量级 0~12) */
#define TRACK_KD            3.0f    /* [可调] 转向D */
#define STEER_MAX           60      /* 转向输出上限(最大差速≈这个值×2) */

/* ======================= 速度环 PID (增量式, car-main 队友实测值) ========== */
#define SPD_KP              110.0f
#define SPD_KI              85.0f
#define SPD_KD              5.0f

/* ======================= 角度环 (MPU6050 yaw, 一般不开) ================== */
#define ANGLE_KP            0.8f
#define ANGLE_KI            0.05f
#define ANGLE_KD            3.0f
#define ANGLE_OUT_MAX       35.0f
#define ANGLE_ENABLE        0       /* 1=在循迹转向基础上叠加航向修正(需先验通6050) */

/* ======================= 赛道 / 任务 (二面题: 胶囊形环线) ================= */
#define TRK_STRAIGHT_MM     1500.0f /* 直道 1.5m */
#define TRK_R_MM            500.0f  /* 半圆 r=0.5m */
#define LAP_MM              (2.0f * TRK_STRAIGHT_MM + 2.0f * 3.14159265f * TRK_R_MM) /* ≈6142mm */

#define TASK_AB_MM          1500.0f /* A->B 沿上直道 (按实测改, 也可运行时改 g_ab_mm)
                                       天猛星实测 g_enc_ab=3800(双边沿)=7600(4倍频), 7600/5.09≈1493mm -> 取 1500 */
#define TASK_AB_SPEED       SPEED_FAST

/* 停止线判据 (照搬天猛星实测值: A点标记只覆盖8路里的4路左右, 不是横穿全宽)
   主判据: >=STOP_HITS_MIN 路同时压黑, 持续 STOP_HOLD_MS
   次级  : >=STOP_HITS_CAND 路, 连续 STOP_HITS_CAND_FRAMES 帧也判 (天猛星: c>=3 连2帧) */
#define STOP_HITS_MIN       4
#define STOP_HOLD_MS        100
#define STOP_HITS_CAND      3
#define STOP_HITS_CAND_FRAMES 2
/* 一圈最短里程: 跑够整圈的这个比例才允许判停止线 (防止起步就压在A线上误停) */
#define LAP_MIN_RATIO       0.75f
/* 里程兜底: 超过整圈这个比例还没看到线也强制停 (丢线保护) */
#define LAP_MAX_RATIO       1.08f

/* 到位动作 */
#define STOP_BEEP_MS        1000    /* 蜂鸣器鸣响 1s */
#define STOP_LED_MS         1000    /* LED 同步闪烁 1s */
#define LED_BLINK_MS        200     /* 闪烁半周期 */

/* 超声波附加项(需飞线) */
#define OBS_NEAR_MM         35      /* 距离 <= 这个值 判定有障碍 (2~3cm前停) */
#define OBS_FAR_MM          100     /* 距离 > 这个值 判定障碍已移开 (滞回) */
#define OBSTACLE_ENABLE     1

/* ======================= 灰度 芯流 T8 (IIC) ======================= */
#define GRAY_ADDR           0x40
#define GRAY_ACTIVE_LOW     1       /* 1=位取反(bit=1表示压线) */
#define GRAY_REVERSE        0       /* 装反了改 1 */
#define GRAY_TIMEOUT_MS     20

/* ======================= MPU6050 ======================= */
#define MPU_ADDR            0x68    /* i2cRead/i2cWrite 内部会 <<1 */
#define MPU_CAL_ENABLE      1       /* 上电静止2秒采陀螺零偏 (车必须静止!) */

/* ======================= OLED / 按键 ======================= */
#define OLED_ADDR           0x3C
/* 刷屏周期。一次全屏要软I2C推 1024 字节 ≈37ms, 期间主循环被卡住,
   所以别刷太勤。天猛星实测是 500ms 刷一次, 照搬。 */
#define OLED_REFRESH_MS     500
#define KEY_DEBOUNCE        3

/* --- SSD1306 时钟/电源配置 (值取自天猛星, 已实测点亮) ---
   0xD5: 显示时钟分频比 + 振荡器频率。0x80 = 分频1 + Fosc=8 (最高, 默认)
         若屏闪/雪花可试 0x90(降一档)
   0xD9: 预充电周期。0xF1 = phase1=1 DCLK, phase2=15 DCLK (默认)
   0x8D: 电荷泵。0x14=使能内部DC-DC升压(4脚I2C模块用这个)
         若你的模块是"外部VCC直供"型(7脚/带VCCIN), 改成 0x10 关电荷泵 */
#define OLED_CLK_DIV        0x80
#define OLED_PRECHARGE      0xF1
#define OLED_CHARGE_PUMP    0x14

#endif /* __CAR_CONFIG_H */
