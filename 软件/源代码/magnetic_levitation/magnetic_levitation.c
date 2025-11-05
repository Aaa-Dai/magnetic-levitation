
#include "magnetic_levitation.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "sys_gpio.h"
#include "sys_dev.h"


static double IIR_err = 0.8, IIR_derr = 0.89; //位置误差低通滤波系数、位置微分误差低通滤波系数
static double Kp = 0.006, Ki = 0, Kd = 0.2; //PID控制比例系数、积分系数、微分系数
static double Ex_adjust = 0, Ey_adjust = 0, K_adjust = 0.01; //X轴位置期望值调节、Y轴位置期望值调节、调节值积分系数
static double E_x = EX_DEFAULT, E_y = EY_DEFAULT, E_z = EZ_DEFAULT; //X轴位置期望值、Y轴位置期望值、Z轴位置期望值

static uint32_t value;
static double adc_x = 0, adc_y = 0, adc_z = 0; //X轴ADC值、Y轴ADC值、Z轴ADC值
static double err_x = 0, err_y = 0, err_z = 0; //X轴位置误差值、Y轴位置误差值、Z轴位置误差值
static double p_err_x = 0, p_err_y = 0, p_err_z = 0; //X轴上一次位置误差值、Y轴上一次位置误差值、Z轴上一次位置误差值
static double derr_x = 0, derr_y = 0, derr_z = 0; //X轴位置误差增量值、Y轴位置误差增量值、Z轴位置误差增量值
static double out_A = 0, out_B = 0, out_C = 0, out_D = 0; //线圈A输出PWM值、线圈B输出PWM值、线圈C输出PWM值、线圈D输出PWM值

static rt_adc_device_t adc1;
static struct rt_device_pwm *pwm3;
static rt_device_t timer4;
static bool magnetic_levitation_thread_should_exit = false;


static void gpio_init(void)
{
    gpio_pin_mode(STBY1, PIN_MODE_OUTPUT);
    gpio_pin_mode(STBY2, PIN_MODE_OUTPUT);
    gpio_pin_mode(AIN1, PIN_MODE_OUTPUT);
    gpio_pin_mode(AIN2, PIN_MODE_OUTPUT);
    gpio_pin_mode(BIN1, PIN_MODE_OUTPUT);
    gpio_pin_mode(BIN2, PIN_MODE_OUTPUT);
    gpio_pin_mode(CIN1, PIN_MODE_OUTPUT);
    gpio_pin_mode(CIN2, PIN_MODE_OUTPUT);
    gpio_pin_mode(DIN1, PIN_MODE_OUTPUT);
    gpio_pin_mode(DIN2, PIN_MODE_OUTPUT);

	gpio_pin_write(STBY1, PIN_LOW);
    gpio_pin_write(STBY2, PIN_LOW);
    gpio_pin_write(AIN1, PIN_LOW);
    gpio_pin_write(AIN2, PIN_LOW);
    gpio_pin_write(BIN1, PIN_LOW);
    gpio_pin_write(BIN2, PIN_LOW);
    gpio_pin_write(CIN1, PIN_LOW);
    gpio_pin_write(CIN2, PIN_LOW);
    gpio_pin_write(DIN1, PIN_LOW);
    gpio_pin_write(DIN2, PIN_LOW);
}


static rt_adc_device_t adc_init(const char *dev)
{
    rt_adc_device_t adc;
    adc = (rt_adc_device_t)sys_dev_find(dev);
	rt_adc_enable(adc, HALL_X);
    rt_adc_enable(adc, HALL_Y);
    rt_adc_enable(adc, HALL_Z);

    return adc;
}


static struct rt_device_pwm *pwm_init(const char *dev, rt_uint32_t period)
{
    struct rt_device_pwm *pwm_dev;
    pwm_dev = (struct rt_device_pwm *)rt_device_find(dev);
    if(pwm_dev == RT_NULL)
    {
        rt_kprintf("pwm sample run failed! can't find %s device!\n", dev);
        return RT_NULL;
    }
    rt_pwm_set(pwm_dev, PWM_A, period, 0);
    rt_pwm_set(pwm_dev, PWM_B, period, 0);
    rt_pwm_set(pwm_dev, PWM_C, period, 0);
    rt_pwm_set(pwm_dev, PWM_D, period, 0);
    rt_pwm_enable(pwm_dev, PWM_A);
    rt_pwm_enable(pwm_dev, PWM_B);
    rt_pwm_enable(pwm_dev, PWM_C);
    rt_pwm_enable(pwm_dev, PWM_D);

    return pwm_dev;
}


static rt_device_t timer_init(const char *dev, rt_uint16_t oflag)
{
    rt_err_t ret = RT_EOK;
    rt_device_t timer = RT_NULL;

    timer = rt_device_find(TIMER_DEV);
    if(timer == RT_NULL)
    {
        rt_kprintf("hwtimer sample run failed! can't find %s device!\n", dev);
    }

    ret = rt_device_open(timer, oflag);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", dev);
    }

    return timer;
}


static void solenoid_control(int channel, double duty_cycle)
{
    uint32_t pulse;

    /* 输出限幅 */
    if(duty_cycle > DUTY_CYCLE_LIMIT)
    {
        duty_cycle = DUTY_CYCLE_LIMIT;
    }
    else if(duty_cycle < -DUTY_CYCLE_LIMIT)
    {
        duty_cycle = -DUTY_CYCLE_LIMIT;
    }

    switch(channel)
    {
        case PWM_A:
            if(duty_cycle > 0)
            {
                pulse = duty_cycle*PWM_PERIOD;
                if((gpio_pin_read(AIN1) == PIN_HIGH) && (gpio_pin_read(AIN2) == PIN_LOW))
                {
                    rt_pwm_set_pulse(pwm3, PWM_A, pulse);
                }
                else
                {
                    gpio_pin_write(AIN2, PIN_LOW);
                    rt_pwm_set_pulse(pwm3, PWM_A, pulse);
                    gpio_pin_write(AIN1, PIN_HIGH);
                }
            }
            else if(duty_cycle < 0)
            {
                pulse = -duty_cycle*PWM_PERIOD;
                if((gpio_pin_read(AIN1) == PIN_LOW) && (gpio_pin_read(AIN2) == PIN_HIGH))
                {
                    rt_pwm_set_pulse(pwm3, PWM_A, pulse);
                }
                else
                {
                    gpio_pin_write(AIN1, PIN_LOW);
                    rt_pwm_set_pulse(pwm3, PWM_A, pulse);
                    gpio_pin_write(AIN2, PIN_HIGH);
                }
            }
            else
            {
                gpio_pin_write(AIN1, PIN_LOW);
                gpio_pin_write(AIN2, PIN_LOW);
            }
            break;

        case PWM_B:
            if(duty_cycle > 0)
            {
                pulse = duty_cycle*PWM_PERIOD;
                if((gpio_pin_read(BIN1) == PIN_HIGH) && (gpio_pin_read(BIN2) == PIN_LOW))
                {
                    rt_pwm_set_pulse(pwm3, PWM_B, pulse);
                }
                else
                {
                    gpio_pin_write(BIN2, PIN_LOW);
                    rt_pwm_set_pulse(pwm3, PWM_B, pulse);
                    gpio_pin_write(BIN1, PIN_HIGH);
                }
            }
            else if(duty_cycle < 0)
            {
                pulse = -duty_cycle*PWM_PERIOD;
                if((gpio_pin_read(BIN1) == PIN_LOW) && (gpio_pin_read(BIN2) == PIN_HIGH))
                {
                    rt_pwm_set_pulse(pwm3, PWM_B, pulse);
                }
                else
                {
                    gpio_pin_write(BIN1, PIN_LOW);
                    rt_pwm_set_pulse(pwm3, PWM_B, pulse);
                    gpio_pin_write(BIN2, PIN_HIGH);
                }
            }
            else
            {
                gpio_pin_write(BIN1, PIN_LOW);
                gpio_pin_write(BIN2, PIN_LOW);
            }
            break;

        case PWM_C:
            if(duty_cycle > 0)
            {
                pulse = duty_cycle*PWM_PERIOD;
                if((gpio_pin_read(CIN1) == PIN_LOW) && (gpio_pin_read(CIN2) == PIN_HIGH))
                {
                    rt_pwm_set_pulse(pwm3, PWM_C, pulse);
                }
                else
                {
                    gpio_pin_write(CIN1, PIN_LOW);
                    rt_pwm_set_pulse(pwm3, PWM_C, pulse);
                    gpio_pin_write(CIN2, PIN_HIGH);
                }
            }
            else if(duty_cycle < 0)
            {
                pulse = -duty_cycle*PWM_PERIOD;
                if((gpio_pin_read(CIN1) == PIN_HIGH) && (gpio_pin_read(CIN2) == PIN_LOW))
                {
                    rt_pwm_set_pulse(pwm3, PWM_C, pulse);
                }
                else
                {
                    gpio_pin_write(CIN2, PIN_LOW);
                    rt_pwm_set_pulse(pwm3, PWM_C, pulse);
                    gpio_pin_write(CIN1, PIN_HIGH);
                }
            }
            else
            {
                gpio_pin_write(CIN1, PIN_LOW);
                gpio_pin_write(CIN2, PIN_LOW);
            }
            break;

        case PWM_D:
            if(duty_cycle > 0)
            {
                pulse = duty_cycle*PWM_PERIOD;
                if((gpio_pin_read(DIN1) == PIN_LOW) && (gpio_pin_read(DIN2) == PIN_HIGH))
                {
                    rt_pwm_set_pulse(pwm3, PWM_D, pulse);
                }
                else
                {
                    gpio_pin_write(DIN1, PIN_LOW);
                    rt_pwm_set_pulse(pwm3, PWM_D, pulse);
                    gpio_pin_write(DIN2, PIN_HIGH);
                }
            }
            else if(duty_cycle < 0)
            {
                pulse = -duty_cycle*PWM_PERIOD;
                if((gpio_pin_read(DIN1) == PIN_HIGH) && (gpio_pin_read(DIN2) == PIN_LOW))
                {
                    rt_pwm_set_pulse(pwm3, PWM_D, pulse);
                }
                else
                {
                    gpio_pin_write(DIN2, PIN_LOW);
                    rt_pwm_set_pulse(pwm3, PWM_D, pulse);
                    gpio_pin_write(DIN1, PIN_HIGH);
                }
            }
            else
            {
                gpio_pin_write(DIN1, PIN_LOW);
                gpio_pin_write(DIN2, PIN_LOW);
            }
            break;
    }
}


static rt_err_t pid_ctrl(rt_device_t dev, rt_size_t size)
{
    /* 获取XYZ的ADC值,并进行一阶低通滤波 */
    value = rt_adc_read(adc1, HALL_X);
    adc_x = IIR_err*adc_x + (1.0 - IIR_err)*value;
    value = rt_adc_read(adc1, HALL_Y);
    adc_y = IIR_err*adc_y + (1.0 - IIR_err)*value;
    value = rt_adc_read(adc1, HALL_Z);
    adc_z = IIR_err*adc_z + (1.0 - IIR_err)*value;

    /* 保存上一次的位置误差值 */
    p_err_x = err_x;
    p_err_y = err_y;
    p_err_z = err_z;

    /* 计算当前位置误差值 */
    err_x = E_x-adc_x;
    err_y = E_y-adc_y;
    err_z = E_z-adc_z;

    /* 计算位置微分误差值,并进行一阶低通滤波 */
    derr_x = IIR_derr*derr_x + (1.0 - IIR_derr)*(err_x - p_err_x);
    derr_y = IIR_derr*derr_y + (1.0 - IIR_derr)*(err_y - p_err_y);
    derr_z = IIR_derr*derr_z + (1.0 - IIR_derr)*(err_z - p_err_z);

    /* 对位置环进行PD控制计算 */
    out_A = -Kp*err_x - Kd*derr_x;
    out_B = Kp*err_y + Kd*derr_y;
    out_C = -out_A;
    out_D = -out_B;

    /* 控制4个电磁线圈输出 */
    solenoid_control(PWM_A, out_A);
    solenoid_control(PWM_B, out_B);
    solenoid_control(PWM_C, out_C);
    solenoid_control(PWM_D, out_D);

    /* 在检测到浮子并且位置误差值小于一定范围后进行位置期望值调节(积分分离) */
    if(adc_z > ENABLE_ADC_VALUE)
    {
        if((err_x < ADJUST_ERR_LIMIT) && (err_x > -ADJUST_ERR_LIMIT))
        {
            Ex_adjust += K_adjust*(out_C - out_A);
            if(Ex_adjust > ADJUST_LIMIT)
            {
                Ex_adjust = ADJUST_LIMIT;
            }
            else if(Ex_adjust < -ADJUST_LIMIT)
            {
                Ex_adjust = -ADJUST_LIMIT;
            }

            E_x = EX_DEFAULT + Ex_adjust;
        }

        if((err_y < ADJUST_ERR_LIMIT) && (err_y > -ADJUST_ERR_LIMIT))
        {
            Ey_adjust += K_adjust*(out_B - out_D);
            if(Ey_adjust > ADJUST_LIMIT)
            {
                Ey_adjust = ADJUST_LIMIT;
            }
            else if(Ey_adjust < -ADJUST_LIMIT)
            {
                Ey_adjust = -ADJUST_LIMIT;
            }

            E_y = EY_DEFAULT + Ey_adjust;
        }
    }

    return RT_EOK;
}


static int magnetic_levitation_main(int argc, char *argv[])
{
    /* GPIO */
    gpio_init();

    /* ADC */
    adc1 = adc_init(ADC_DEV);

    /* PWM */
    pwm3 = pwm_init(PWM_DEV, PWM_PERIOD);

    /* TIMER */
    timer4 = timer_init(TIMER_DEV, RT_DEVICE_OFLAG_RDWR);

    /* 设置PID控制回调函数 */
    rt_device_set_rx_indicate(timer4, pid_ctrl);

    /* 设置定时器计数频率 */
    rt_uint32_t freq = 10000;
    rt_device_control(timer4, HWTIMER_CTRL_FREQ_SET, &freq);

    /* 设置定时器模式 */
    rt_hwtimer_mode_t mode = HWTIMER_MODE_PERIOD;
    rt_device_control(timer4, HWTIMER_CTRL_MODE_SET, &mode);

    /* 设置定时器超时时间 */
    rt_hwtimerval_t timeout_s;
    timeout_s.sec = 0;      /* 秒 */
    timeout_s.usec = 500;     /* 微秒 */
    rt_device_write(timer4, 0, &timeout_s, sizeof(timeout_s));

    /* vofa */
    vofa_just_float_t vofa_data = {
        .data = {0},
        .tail = {0x00, 0x00, 0x80, 0x7F}
    };
    rt_device_t debug_uart = NULL;
    debug_uart = rt_device_find(DEBUG_UART);
    uint64_t print_timestamp = sys_absolute_time();

    while(!magnetic_levitation_thread_should_exit)
    {
        if(adc_z > ENABLE_ADC_VALUE)
        {
            gpio_pin_write(STBY1, PIN_HIGH);
            gpio_pin_write(STBY2, PIN_HIGH);
        }
        else
        {
            gpio_pin_write(STBY1, PIN_LOW);
            gpio_pin_write(STBY2, PIN_LOW);
        }

        if(sys_elapsed_time(print_timestamp) > 10000)
        {
            print_timestamp = sys_absolute_time();
            /* 数据进行vofa建图 */
            /*vofa_data.data[0] = adc_x;
            vofa_data.data[1] = adc_y;
            vofa_data.data[2] = adc_z;
            vofa_data.data[3] = E_x;
            vofa_data.data[4] = E_y;
            vofa_data.data[5] = E_z;
            vofa_data.data[6] = out_A;
            vofa_data.data[7] = out_B;
            vofa_data.data[8] = out_C;
            vofa_data.data[9] = out_D;
            sys_dev_write(debug_uart, &vofa_data, sizeof(vofa_data));*/

            /* 数据进行打印显示 */
            /*printf("outA: %.2lf\r\n", out_A);
            printf("outB: %.2lf\r\n", out_B);
            printf("outC: %.2lf\r\n", out_C);
            printf("outD: %.2lf\r\n", out_D);
            printf("Kp: %lf\r\n", Kp);
            printf("Kd: %lf\r\n", Kd);
            printf("fp: %lf\r\n", IIR_err);
            printf("fd: %lf\r\n", IIR_derr);
            printf("ex: %.2lf\r\n", E_x);
            printf("ey: %.2lf\r\n", E_y);
            printf("ez: %.2lf\r\n", E_z);
            printf("X: %.2lf\r\n", adc_x);
            printf("Y: %.2lf\r\n", adc_y);
            printf("Z: %.2lf\r\n", adc_z);
            printf("\r\n");*/
        }

        task_delay_ms(5);
    }

	sys_task_destroy(sys_task_self());
	magnetic_levitation_thread_should_exit = true;
}

static int magnetic_levitation_task_create(void)
{
    sys_err_t ret;
    ret = sys_task_create("magnetic_levitation", 18, 1024, magnetic_levitation_main, NULL);
    if(ret != SYS_EOK)
    {
        printf("[error] magnetic levitation task create\r\n");
    }
    else
    {
        printf("[ok] magnetic levitation task create\r\n");
    }
	
	return ret;
}
USR_APP_START_EXPORT(magnetic_levitation_task_create);



#ifdef SYS_USING_CLI
#include <cli_common.h>

static int pid_cli_main(int argc, char *argv[])
{
    cli_print("\r\n\n");
	if(argc > 5)
    {
		cli_print("[param] too many arguments\r\n");
		return 0;
	}

    else if(argc == 3)
    {
        if(strncmp(argv[1], "-g", 2) == 0)
        {
            if(strncmp(argv[2], "kp", 2) == 0)
            {
                printf("Kp: %lf\r\n", Kp);
            }
            else if(strncmp(argv[2], "kd", 2) == 0)
            {
                printf("Kd: %lf\r\n", Kd);
            }
            else if(strncmp(argv[2], "ex", 2) == 0)
            {
                printf("ex: %.2lf\r\n", E_x);
            }
            else if(strncmp(argv[2], "ey", 2) == 0)
            {
                printf("ey: %.2lf\r\n", E_y);
            }
            else if(strncmp(argv[2], "ez", 2) == 0)
            {
                printf("ez: %.2lf\r\n", E_z);
            }
            else if(strncmp(argv[2], "all", 3) == 0)
            {
                printf("Kp: %lf\r\n", Kp);
                printf("Kd: %lf\r\n", Kd);
                printf("ex: %.2lf\r\n", E_x);
                printf("ey: %.2lf\r\n", E_y);
                printf("ez: %.2lf\r\n", E_z);
            }
            else
            {
                cli_print("parameter2 is incorrect\r\n");
                return 0;
            }
        }
        else
        {
            cli_print("parameter1 is incorrect\r\n");
            return 0;
        }
    }
    else if(argc == 4)
    {
        if(strncmp(argv[1], "-s", 2) == 0)
        {
            if(strncmp(argv[2], "kp", 2) == 0)
            {
                Kp = atof(argv[3]);
            }
            else if(strncmp(argv[2], "kd", 2) == 0)
            {
                Kd = atof(argv[3]);
            }
            else if(strncmp(argv[2], "ex", 2) == 0)
            {
                E_x = atof(argv[3]);
            }
            else if(strncmp(argv[2], "ey", 2) == 0)
            {
                E_y = atof(argv[3]);
            }
            else if(strncmp(argv[2], "ez", 2) == 0)
            {
                E_z = atof(argv[3]);
            }
            else if(strncmp(argv[2], "fp", 2) == 0)
            {
                IIR_err = atof(argv[3]);
            }
            else if(strncmp(argv[2], "fd", 2) == 0)
            {
                IIR_derr = atof(argv[3]);
            }
            else
            {
                cli_print("parameter2 is incorrect\r\n");
                return 0;
            }
        }
        else
        {
            cli_print("parameter1 is incorrect\r\n");
            return 0;
        }
    }

    return SYS_EOK;
}
CLI_FUNCTION_EXPORT(pid_cli_main, pid, pid command interface);
#endif
