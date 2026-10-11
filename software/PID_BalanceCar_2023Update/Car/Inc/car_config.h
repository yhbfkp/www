/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       car_config.h
  * @brief      循迹小车 - 参数总表 (2026-09-27 定版: 环形赛道 A->B / A->A)
  * @note       算法来源: car-main v1.3 (2024电赛H题完赛代码) —— 加权差比和 + 转向PD + 增量式速度PID
  *                       + 角度环pid_angle + HARD_BRAKE 急停
  *             数值全部照搬 v1.3 原值, 只做两处硬件适配:
  *               1) duty 刻度: v1.3 MAX_DUTY=50000 -> 本工程 TIM1(Period=7199) CCR, 在 motorA_duty() 内换算
  *               2) 编码器: v1.3 用 EXTI 单相下降沿(1计数/脉冲), F103 TIM 编码器模式 TI12 是4倍频
  *                  -> 读数 /4 后再喂 PID, 保证速度量级与原版一致 (见 ENC_DIV)
  *             想调参直接改这里; 标 [可调] 的也是 volatile 全局变量, Keil debug 里能直接改。
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-06-2026     BalanceCar      1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  */

#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/* ======================= 模块开关 ======================= */
/* 2026-09-29 综合验证: 打开 电机/编码器/灰度; OLED 与超声波暂不开 */
#define CAR_USE_MOTOR       1       //0=完全不输出PWM(调试安全)
#define CAR_USE_ENCODER     1
#define CAR_USE_GRAY        1       //芯流 T8 八路灰度 (硬件I2C2 PB10/PB11)
#define CAR_USE_MPU         1      //MPU6050 (软I2C PA6/PA7) —— 纯轮询 burst 读, 不依赖 INT 脚, 不用插 INT
#define CAR_USE_ULTRA       1       //超声波: 板上座子没走线, 必须飞线 PB0/PB1 才能开
#define CAR_USE_OLED        0    //屏点亮后改回1; 屏不亮时刷屏只会拖停灰度采样(循迹甩尾)       //打开: 跑 OLED 硬件自检(白闪/扫描/引脚自检); 屏幕问题处理完可再关
#define CAR_USE_KEY         1
#define CAR_USE_PRINT       1       //串口打印 (printf -> USART1 PA9/PA10 115200)
#define CAR_USE_PLOT        0       //串口CSV波形输出(给 SerialPlot 看速度环)

/* ======================= PWM / 控制周期 ======================= */
#define PWM_MAX             7199    //TIM1 Period=7199, CCR 满值
#define MAX_DUTY            50000   //归一化 duty 满量程 (car-main 原值, PID输出上限)

#define CTRL_TICK_MS        10      //TIM3 心跳定时器，10ms中断一次
#define CTRL_PID_DIV        10      //每 10 次中断跑一次速度环pid = 100ms (car-main 原周期)

/* ======================= 电机 ======================= */
/* 网表: A路 PWMA=PA11(TIM1_CH4) AIN1=PB13 AIN2=PB12
         B路 PWMB=PA8 (TIM1_CH1) BIN1=PB14 BIN2=PB15 */
#define MOTOR_L_IS_A        1       //1=左轮接A路; 左右反了改 0
#define MOTOR_L_DIR         1       //转反了改 -1
#define MOTOR_R_DIR         1

/* ======================= 速度 (car-main 单位: 每100ms的等效单相编码器计数) ====== */
#define SPEED_FAST          40      //[可调] 直道/基础速度
#define SPEED_SLOW          25      //丢线搜索速度
#define SPEED_MIN           8       //内轮最低速(调小=差速大)

/* ======================= 编码器 ======================= */
#define ENC_DIV             4       //TIM 编码器4倍频 -> 等效原版单相计数, 必须除 4
/* 编码器极性 —— 左右轮是镜像安装的, 两侧不一定相同, 必须分别定:
     左轮: 车前进时读出来是负的 -> -1
     右轮: 与左轮相反           -> +1
   判定方法: g_pid_tune=1 给目标后看 g_spd_now_*, 哪边是负的就翻哪边的 g_enc_dir_*。
   这两个只是初值, 运行时可在 Watch 里直接改 g_enc_dir_l / g_enc_dir_r 翻转, 不用重新编译。 */
#define ENC_L_DIR           -1
#define ENC_R_DIR           1
#define ENC_LINES           13      //编码器线数   —— 按你的电机改
#define ENC_REDUCTION       20.0f   //减速比       —— 按你的电机改
#define WHEEL_MM            65.0f   //轮径 mm      —— 按你的车轮改
/* 4倍频计数/转: 13*20*4 = 1040 (fp32 由 Car/Inc/struct_typedef.h 提供, 展开处都已包含它) */
#define ENC_CPR             ((fp32)ENC_LINES * ENC_REDUCTION * (fp32)ENC_DIV)
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

#define TRACK_KP            8.0f    //[可调] 转向P (误差量级 0~12)
#define TRACK_ERR_SLEW      1.0f    //[防跳变] error 每帧(10ms)最大变化量, 抑制灰度量化跳变
#define TRACK_OUT_SLEW      5
#define TRACK_START_HOLD   80      //[可调] 起步直行帧数(10ms/帧), 80=800ms, 跨过启停线混乱期       //[防跳变] L/R 目标轮速每帧(10ms)最大变化量, 阶跃变斜坡
#define TRACK_KD            7.0f    //[可调] 转向D
#define STEER_MAX           60      //转向输出上限(最大差速≈这个值×2)

/* ======================= 速度环 PID (增量式, car-main 队友实测值) ========== */
#define SPD_KP              110.0f
#define SPD_KI              130.0f
#define SPD_KD              5.0f

/* ======================= 角度环 (MPU6050 yaw, 一般不开) ================== */
#define ANGLE_KP            0.8f
#define ANGLE_KI            0.05f
#define ANGLE_KD            3.0f
#define ANGLE_OUT_MAX       35.0f
#define ANGLE_ENABLE        0       //1=在循迹转向基础上叠加航向修正(需先验通6050)

/* ======================= 赛道 / 任务 (二面题: 胶囊形环线) ================= */
#define TRK_STRAIGHT_MM     2800.0f //直道 1.5m
#define TRK_R_MM            500.0f  //半圆 r=0.5m
#define LAP_MM              (2.0f * TRK_STRAIGHT_MM + 2.0f * 3.14159265f * TRK_R_MM) //≈6142mm

#define TASK_AB_MM          2000.0f //A->B 沿上直道 (按实测改, 也可运行时改 g_ab_mm)
                                    //天猛星实测 g_enc_ab=3800(双边沿)=7600(4倍频), 7600/5.09≈1493mm -> 取 1500
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
#define STOP_BEEP_MS        1000    //蜂鸣器鸣响 1s
#define STOP_LED_MS         1000    //LED 同步闪烁 1s
#define LED_BLINK_MS        200     //闪烁半周期

/* 超声波附加项(需飞线) */
#define OBS_NEAR_MM         35      //距离 <= 这个值 判定有障碍 (2~3cm前停)
#define OBS_FAR_MM          100     //距离 > 这个值 判定障碍已移开 (滞回)
#define OBSTACLE_ENABLE     1
/* 连续这么多次没收到回波才认定"没测到"。单次丢包不能当无障碍(车前的障碍会因此
   被误判成已移开), 所以先沿用上一帧距离; 100ms 一次 -> 5 次 = 500ms。 */
#define ULTRA_MISS_MAX      5

/* ======================= 灰度 芯流 T8 (IIC) ======================= */
#define GRAY_ADDR           0x40
#define GRAY_ACTIVE_LOW     1       //1=位取反(bit=1表示压线)
#define GRAY_REVERSE        0       //装反了改 1
#define GRAY_TIMEOUT_MS     5       //读灰度现在在 TIM3 中断里, 超时不能长(否则拖垮心跳)。
                                    //正常一次传输 @100kHz <1ms, 5ms 足够

/* ======================= MPU6050 ======================= */
#define MPU_ADDR            0x68    //i2cRead/i2cWrite 内部会 <<1
#define MPU_CAL_ENABLE      1       //上电静止2秒采陀螺零偏 (车必须静止!)
#define MPU_INIT_RETRY      5       //初始化重试次数。每轮: 总线恢复 + 软复位 + 200ms + 读ID

/* ======================= OLED / 按键 ======================= */
#define OLED_ADDR           0x3C
/* --- OLED 面板类型 + 传输层 ---
   一个宏搞定: 换屏只动 OLED_TYPE, 驱动逻辑不用改。引脚在 car_oled.c 顶部(带接线表)。
   [!] 这块板子上能用 SPI 的干净脚只有三个, 别乱挑:
       PB8/PB9 = 原来跑软 I2C 的那两个(换 SPI 后正好空出来)
       PA5     = 网表里只引到 H8.11, 固件里配成上拉输入但从来没读过
       ⚠️ 不能用的: PB5 是蜂鸣器(基极会把 0.7V 以上的电平钳掉, 还会一直叫);
          PB3/PB4/PA12/PA15 各挂了 10µF 去抖电容(板载按键 SW2~SW5),
          SPI 时钟/数据上去直接被电容吃掉, 连 DC/RES 翻转都要约 1.5ms。 */
#define OLED_TYPE           2       /* 0=SSD1306 单色 软I2C | 1=SSD1306 单色 软SPI | 2=ST7789 IPS 彩屏 软SPI */
/* 彩屏面板参数(竖屏). 花屏/整体偏移就是这三个数不对, 改这里不用动驱动:
     1.14寸 135x240 偏移(40,52)    ← 当前默认
     1.3寸  240x240 偏移(0,0)      1.47寸 172x320 偏移(34,0)   2.0寸 240x320 偏移(0,0)
   另外: 画面上下/左右反了 -> 改 car_oled.c 里 ST7789 的 0x36 参数(0xA0 与 0x00 互换);
         红蓝反了 -> 那个参数再加 0x08。 */
#define LCD_PANEL_W         240
#define LCD_PANEL_H         240
#define LCD_PANEL_XOFF      0
#define LCD_PANEL_YOFF      80
#define OLED_RES_ENABLE     1       /* 1=用 GPIO 拉 RES 复位脉冲; 0=RES 直接接 3.3V(彩屏建议留 1) */
#define OLED_SPI_DLY_US     0       /* SPI 每个半周期延时(us): 0=最快(约1.5MHz); 屏不稳改 1 */
/* 刷屏周期。软 I2C 时一次全屏要推 1024 字节 ≈37ms, 期间主循环被卡住, 所以别刷太勤;
   换成软件 SPI 后一屏只要约 5ms(快 7 倍), 想更跟手可以降到 100~200。 */
#define OLED_REFRESH_MS     500
#define KEY_DEBOUNCE        3

/* [DIAG] 上电"全屏点亮"自检: 初始化完把整屏点白 1.5 秒再恢复正常。
   这是判断"屏硬件活没活"最直接的一招:
     看到白闪 -> 屏、内部升压(电荷泵)、I2C 全好; 之后没字才是软件的事;
     看不到白闪 且 oled_ack=1 -> 命令已经发到屏了屏却没反应 -> 屏硬件坏。
   调试完改 0 关掉。 */
#define OLED_ALLON_TEST_ON_BOOT 0

/* [DIAG] 彩屏上电四色自检(只对 OLED_TYPE=2 有效): 初始化完先把 红/绿/蓝/白 依次铺满
   "控制器的最大范围"(0~239, 故意不加面板偏移), 一刀切开两种故障:
     能看到颜色(哪怕只占一角) -> 通信是通的, 剩下只是 LCD_PANEL_* 尺寸/偏移没配对;
     四种颜色都看不到(只有背光) -> 通信/初始化根本没生效:
        先查 CS 有没有接地(悬空 = 接口永远不选中, 症状正好是"只有背光"), 再查 RES(平时应为3.3V)、
        SCK/MOSI/DC 的接线, 以及 VCC 是不是 3.3V。
   调完改 0。 */
#define LCD_BRINGUP_TEST    0

/* [DIAG] 屏没应答(oled_ack=0)时: 固件停在"引脚自检" —— PB8/PB9 输出 1Hz
   反相方波, 用万用表在屏的 SCL/SDA 脚量:
     0V<->3.3V 在跳 = MCU->屏 这段(引脚/走线/焊点/杜邦线)是通的, 嫌疑在屏本身;
     一直是死值     = 这段线/焊点/引脚有问题, 嫌疑在板上。
   屏修好后改回 0, 否则拔掉屏跑车也会卡在这里。 */
#define OLED_PIN_TEST_ON_FAIL   1

/* [DIAG] 上电无条件跑引脚自检方波(不管屏有没有应答)。
   SPI 模式没有 ACK 可判, "屏没应答才自检"那条路永远不会触发, 所以换新模块/查接线时
   把这个改成 1: 固件会停在"OLED 时钟/数据脚输出 1Hz 方波"上(死循环), 用万用表量屏的
   SCL/SDA 脚就能判断 MCU->屏 这段通不通。调完必须改回 0, 否则车永远停在自检里。 */
#define OLED_FORCE_PIN_TEST     0

/* --- SSD1306 时钟/电源配置 ---
   取值全部来自用户提供的官方规格书 QG-2864KLBEG01 Rev C
   (01经典款0.96寸OLED30pin规格书 第24/29/30页 "4.4.1 VCC Supplied Externally")
      - 修订史明确要求: 0x81 对比度 = 0xCF, 0xD9 预充电 = 0xF1, RIREF = 910K
   0xD5: 显示时钟分频比 + 振荡器频率。0x80 = 分频1 + Fosc=8 (最高, 默认)
         若屏闪/雪花可试 0x90(降一档)
   0xD9: 预充电周期。0xF1 = phase1=1 DCLK, phase2=15 DCLK
   0x8D: 电荷泵。0x14=使能内部DC-DC升压(4脚模块用这个, 官方流程同)
   0xDB: VCOMH 电平。0x30 = 0.83*Vcc, 0x40 = 1.0*Vcc
         官方流程用的是 0x30; 之前写死 0x40(来源不明) 是重点嫌疑之一 */
#define OLED_CLK_DIV        0x80
#define OLED_PRECHARGE      0xF1
#define OLED_CHARGE_PUMP    0x14
#define OLED_VCOMH          0x30

/* 软 I2C 半周期延时(us): SCL 周期约 2 倍于此, 2->约250kHz, 5->约100kHz。
   面包板长杜邦线 + 模块自带4.7k上拉时上升沿很慢, 速率一高从机就采样错,
   命令全被丢弃 -> ACK 能读到但屏死活不亮。宁慢勿快, 默认降速。 */
#define OLED_I2C_DLY_US     5

#endif
