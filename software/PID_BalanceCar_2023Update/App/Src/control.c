#include "control.h"
/**************************************************************************
Function: Read battery voltage
Input   : none
Output  : voltage * 100 (1200 = 12.00V)
函数功能：读取电池电压
说明    ：原工程用 ADC1(PA6) 采样电池分压。现在 PA6 已让给 MPU6050 的软I2C SCL，
          ADC1 被删除，adc.c 也不再参与编译，所以这里先用常量占位（认为电池正常）。
          需要真实低压保护时：把分压接到空闲 ADC 脚（如 PA5/PA4 若未用），
          在此函数里改成 HAL_ADC 采样即可，Turn_Off() 的阈值 1110(11.10V) 不用改。
**************************************************************************/
int Get_battery_volt(void)
{
    return 1200; // 12.00V，常量占位
}
/**************************************************************************
Function: Control function
Input   : none
Output  : none
函数功能：所有的控制代码都在这里面
         原工程由 MPU6050 的 INT 引脚 5ms 触发一次外部中断；
         现在 PA12(MPU6050_EXTI) 已改作 KEY2 按键，中断源没有了，
         改由 TIM3 每 10ms 溢出调用一次（见本文件末尾 TIM3_10ms_Task）。
入口参数：无
返回  值：无
**************************************************************************/
void Control_Task(void)
{
    static int Voltage_Temp, Voltage_Count, Voltage_All; // 电压测量相关变量
    int Encoder_Left, Encoder_Right;                     // 左右编码器的脉冲计数
    int Balance_Pwm, Velocity_Pwm, Turn_Pwm;             // 平衡环PWM变量，速度环PWM变量，转向环PWM变

    Get_Angle();             // 更新姿态，10ms一次
    Encoder_Left = -Read_Encoder(2);  // 读取左轮编码器的值，前进为正，后退为负
    Encoder_Right = -Read_Encoder(4); // 读取右轮编码器的值，前进为正，后退为负
                                      // 左轮A相接TIM2_CH1,右轮A相接TIM4_CH2,故这里两个编码器的极性相同
    Get_Velocity_Form_Encoder(Encoder_Left, Encoder_Right); // 编码器读数转速度（mm/s）

    Voltage_Temp = Get_battery_volt(); // 读取电池电压
    Voltage_Count++;                   // 平均值计数器
    Voltage_All += Voltage_Temp;       // 多次采样累积
    if (Voltage_Count == 100){         // 100次×10ms = 1s 求一次平均
        BalanceCar.Voltage = Voltage_All / 100;
        Voltage_All = 0;
        Voltage_Count = 0; // 求平均值
    }

    Read_Distane(); // 获取超声波测量距离值
    if (BalanceCar.Flag_follow == 0 && BalanceCar.Flag_avoid == 0)
        Led_Flash(100); // LED闪烁;常规模式 1s改变一次指示灯的状态
    if (BalanceCar.Flag_follow == 1 || BalanceCar.Flag_avoid == 1)
        Led_Flash(0);                                     // LED常亮;超声波跟随/避障模式
    Key();                                                // 扫描按键状态 单击双击可以改变小车运行状态
    Balance_Pwm = Balance(BalanceCar.Angle_Balance, BalanceCar.Gyro_Balance);   // 平衡PID控制 BalanceCar.Gyro_Balance平衡角速度极性：前倾为正，后倾为负
    Velocity_Pwm = Velocity(Encoder_Left, Encoder_Right); // 速度环PID控制	记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
    Turn_Pwm = Turn(BalanceCar.Gyro_Turn);                           // 转向环PID控制

    BalanceCar.Motor_Left = Balance_Pwm + Velocity_Pwm + Turn_Pwm;  // 计算左轮电机最终PWM
    BalanceCar.Motor_Right = Balance_Pwm + Velocity_Pwm - Turn_Pwm; // 计算右轮电机最终PWM
                                                         // PWM值正数使小车前进，负数使小车后退
    BalanceCar.Motor_Left = PWM_Limit(BalanceCar.Motor_Left, 6900, -6900);
    BalanceCar.Motor_Right = PWM_Limit(BalanceCar.Motor_Right, 6900, -6900);                       // PWM限幅
    if (Pick_Up(BalanceCar.Acceleration_Z, BalanceCar.Angle_Balance, Encoder_Left, Encoder_Right)) // 检查是否小车被拿起
        BalanceCar.Flag_Stop = 1;                                                       // 如果被拿起就关闭电机
    if (Put_Down(BalanceCar.Angle_Balance, Encoder_Left, Encoder_Right))                // 检查是否小车被放下
        BalanceCar.Flag_Stop = 0;                                                       // 如果被放下就启动电机
    Choose(Encoder_Left, Encoder_Right);                                     // 转动右轮选择小车模式
    if (Turn_Off(BalanceCar.Angle_Balance, BalanceCar.Voltage) == 0)                               // 如果不存在异常
        Set_Pwm(BalanceCar.Motor_Left, BalanceCar.Motor_Right);                                    // 赋值给PWM寄存器
}
/**************************************************************************
Function: TIM3 10ms 定时回调（覆盖 tim.c 中的 __weak 定义）
Input   : none
Output  : none
函数功能：TIM3 配置为 1MHz 计数、Period=9999，即每 10ms 溢出一次并调用本函数。
注意    ：IOI2C 目前还绑在 PB8/PB9（板上 OLED 座），MPU6050 在 PA6/PA7，
         在把 IOI2C 复制一份改到 PA6/PA7 之前，Get_Angle() 读回来的会是无效值。
         只想先验证下载跑起来的话，把下面的 Control_Task() 临时注释掉即可。
**************************************************************************/
void TIM3_10ms_Task(void)
{
    Control_Task();
}
/**************************************************************************
Function: Assign to PWM register
Input   : BalanceCar.Motor_Left：Left wheel PWM；BalanceCar.Motor_Right：Right wheel PWM
Output  : none
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int Motor_Left, int Motor_Right)
{
    if (Motor_Left > 0)
        BIN1 = 1, BIN2 = 0; // 前进
    else
        BIN1 = 0, BIN2 = 1; // 后退
    PWMB = myabs(Motor_Left);
    if (Motor_Right > 0)
        AIN2 = 1, AIN1 = 0; // 前进
    else
        AIN2 = 0, AIN1 = 1; // 后退
    PWMA = myabs(Motor_Right);
}
/**************************************************************************
Function: PWM limiting range
Input   : IN：Input  max：Maximum value  min：Minimum value
Output  : Output
函数功能：限制PWM赋值
入口参数：IN：输入参数  max：限幅最大值  min：限幅最小值
返回  值：限幅后的值
**************************************************************************/
int PWM_Limit(int IN, int max, int min)
{
    int OUT = IN;
    if (OUT > max)
        OUT = max;
    if (OUT < min)
        OUT = min;
    return OUT;
}
/**************************************************************************
Function: Press the key to modify the car running state
Input   : none
Output  : none
函数功能：按键修改小车运行状态
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{
    u8 tmp, tmp2;
    tmp = click_N_Double(50);
    if (tmp == 1)
    {
        BalanceCar.Flag_Stop = !BalanceCar.Flag_Stop;
    } // 单击控制小车的启停
    tmp2 = Long_Press();
    if (tmp2 == 1)
        BalanceCar.Flag_Show = !BalanceCar.Flag_Show; // 长按控制进入上位机模式，小车的显示停止
}
/**************************************************************************
Function: If abnormal, turn off the motor
Input   : angle：Car inclination；voltage：Voltage
Output  : 1：abnormal；0：normal
函数功能：异常关闭电机
入口参数：angle：小车倾角；voltage：电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off(float angle, int voltage)
{
    u8 temp;
    if (angle < -40 || angle > 40 || 1 == BalanceCar.Flag_Stop || voltage < 1110) // 电池电压低于11.1V关闭电机
    {                                                                             // 倾角大于40度关闭电机
        temp = 1;                                                                 // BalanceCar.Flag_Stop置1，即单击控制关闭电机
        AIN1 = 0;
        AIN2 = 0;
        BIN1 = 0;
        BIN2 = 0;
    }
    else
        temp = 0;
    return temp;
}

/**************************************************************************
Function: Get angle
Input   : way：The algorithm of getting angle 1：DMP  2：kalman  3：Complementary filtering
Output  : none
函数功能：获取角度
返回  值：无
**************************************************************************/
void Get_Angle(void)
{
    float Accel_Y, Accel_Z, Accel_X, Accel_Angle_x, Accel_Angle_y, Gyro_X, Gyro_Z, Gyro_Y;

    Gyro_X = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_XOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_XOUT_L);    // 读取X轴陀螺仪
    Gyro_Y = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_L);    // 读取Y轴陀螺仪
    Gyro_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_L);    // 读取Z轴陀螺仪
    Accel_X = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_L); // 读取X轴加速度计
    Accel_Y = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_YOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_YOUT_L); // 读取X轴加速度计
    Accel_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_L); // 读取Z轴加速度计
    if (Gyro_X > 32768)
        Gyro_X -= 65536; // 数据类型转换  也可通过short强制类型转换
    if (Gyro_Y > 32768)
        Gyro_Y -= 65536; // 数据类型转换  也可通过short强制类型转换
    if (Gyro_Z > 32768)
        Gyro_Z -= 65536; // 数据类型转换
    if (Accel_X > 32768)
        Accel_X -= 65536; // 数据类型转换
    if (Accel_Y > 32768)
        Accel_Y -= 65536; // 数据类型转换
    if (Accel_Z > 32768)
        Accel_Z -= 65536;                               // 数据类型转换
    BalanceCar.Gyro_Balance = -Gyro_X;                             // 更新平衡角速度
    Accel_Angle_x = atan2(Accel_Y, Accel_Z) * 180 / PI; // 计算倾角，转换单位为度
    Accel_Angle_y = atan2(Accel_X, Accel_Z) * 180 / PI; // 计算倾角，转换单位为度
    Gyro_X = Gyro_X / 16.4;                             // 陀螺仪量程转换，量程±2000°/s对应灵敏度16.4，可查手册
    Gyro_Y = Gyro_Y / 16.4;                             // 陀螺仪量程转换
    Pitch = -Kalman_Filter_x(Accel_Angle_x, Gyro_X); // 卡尔曼滤波
    Roll = -Kalman_Filter_y(Accel_Angle_y, Gyro_Y);
    BalanceCar.Angle_Balance = Pitch;    // 更新平衡倾角
    BalanceCar.Gyro_Turn = Gyro_Z;       // 更新转向角速度
    BalanceCar.Acceleration_Z = Accel_Z; // 更新Z轴加速度计
}
/**************************************************************************
Function: Absolute value function
Input   : a：Number to be converted
Output  : unsigned int
函数功能：绝对值函数
入口参数：a：需要计算绝对值的数
返回  值：无符号整型
**************************************************************************/
int myabs(int a)
{
    int temp;
    if (a < 0)
        temp = -a;
    else
        temp = a;
    return temp;
}
/**************************************************************************
Function: Check whether the car is picked up
Input   : Acceleration：Z-axis acceleration；Angle：The angle of balance；encoder_left：Left encoder count；encoder_right：Right encoder count
Output  : 1：picked up  0：No action
函数功能：检测小车是否被拿起
入口参数：Acceleration：z轴加速度；Angle：平衡的角度；encoder_left：左编码器计数；encoder_right：右编码器计数
返回  值：1:小车被拿起  0：小车未被拿起
**************************************************************************/
int Pick_Up(float Acceleration, float Angle, int encoder_left, int encoder_right)
{
    static u16 flag, count0, count1, count2;
    if (flag == 0) // 第一步
    {
        if (myabs(encoder_left) + myabs(encoder_right) < 30) // 条件1，小车接近静止
            count0++;
        else
            count0 = 0;
        if (count0 > 10)
            flag = 1, count0 = 0;
    }
    if (flag == 1) // 进入第二步
    {
        if (++count1 > 200)
            count1 = 0, flag = 0;                                                                    // 超时不再等待2000ms，返回第一步
        if (Acceleration > 26000 && (Angle > (-20 + Middle_angle)) && (Angle < (20 + Middle_angle))) // 条件2，小车是在0度附近被拿起
            flag = 2;
    }
    if (flag == 2) // 第三步
    {
        if (++count2 > 100)
            count2 = 0, flag = 0;                     // 超时不再等待1000ms
        if (myabs(encoder_left + encoder_right) > 70) // 条件3，小车的轮胎因为正反馈达到最大的转速
        {
            flag = 0;
            return 1; // 检测到小车被拿起
        }
    }
    return 0;
}
/**************************************************************************
Function: Check whether the car is lowered
Input   : The angle of balance；Left encoder count；Right encoder count
Output  : 1：put down  0：No action
函数功能：检测小车是否被放下
入口参数：平衡角度；左编码器读数；右编码器读数
返回  值：1：小车放下   0：小车未放下
**************************************************************************/
int Put_Down(float Angle, int encoder_left, int encoder_right)
{
    static u16 flag, count;
    if (BalanceCar.Flag_Stop == 0) // 防止误检
        return 0;
    if (flag == 0)
    {
        if (Angle > (-10 + Middle_angle) && Angle < (10 + Middle_angle) && encoder_left == 0 && encoder_right == 0) // 条件1，小车是在0度附近的
            flag = 1;
    }
    if (flag == 1)
    {
        if (++count > 50) // 超时不再等待 500ms
        {
            count = 0;
            flag = 0;
        }
        if (encoder_left > 3 && encoder_right > 3 && encoder_left < 40 && encoder_right < 40) // 条件2，小车的轮胎在未上电的时候被人为转动
        {
            flag = 0;
            flag = 0;
            return 1; // 检测到小车被放下
        }
    }
    return 0;
}
/**************************************************************************
Function: Encoder reading is converted to speed (mm/s)
Input   : none
Output  : none
函数功能：编码器读数转换为速度（mm/s）
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(int encoder_left, int encoder_right)
{
    float Rotation_Speed_L, Rotation_Speed_R; // 电机转速  转速=编码器读数（5ms每次）*读取频率/倍频数/减速比/编码器精度
    Rotation_Speed_L = encoder_left * Control_Frequency / EncoderMultiples / Reduction_Ratio / Encoder_precision;
    Velocity_Left = Rotation_Speed_L * PI * Diameter_67; // 求出编码器速度=转速*周长
    Rotation_Speed_R = encoder_right * Control_Frequency / EncoderMultiples / Reduction_Ratio / Encoder_precision;
    Velocity_Right = Rotation_Speed_R * PI * Diameter_67; // 求出编码器速度=转速*周长
}
/**************************************************************************
Function: Select car running mode
Input   : encoder_left：Left wheel encoder reading；encoder_right：Right wheel encoder reading
Output  : none
函数功能：选择小车运行模式
入口参数：encoder_left：左编码器读数  encoder_right：右编码器读数
返回  值：无
**************************************************************************/
void Choose(int encoder_left, int encoder_right)
{
    static int count;
    if (BalanceCar.Flag_Stop == 0)
        count = 0;
    if ((BalanceCar.Flag_Stop == 1) && (encoder_left < 10)) // 此时停止且左轮不动
    {
        count += myabs(encoder_right);
        if (count > 6 && count < 180) // 普通模式
        {
            BalanceCar.Flag_follow = 0;
            BalanceCar.Flag_avoid = 0;
        }
        if (count > 180 && count < 360) // 避障模式
        {
            BalanceCar.Flag_avoid = 1;
            BalanceCar.Flag_follow = 0;
        }
        if (count > 360 && count < 540) // 跟随模式
        {
            BalanceCar.Flag_avoid = 0;
            BalanceCar.Flag_follow = 1;
        }
        if (count > 540)
            count = 0;
    }
}
