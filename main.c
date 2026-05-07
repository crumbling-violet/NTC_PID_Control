/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float target_temp;  // 目标温度
    float integral;     // 积分累计值
    float prev_error;   // 上一次的误差 (用于算微分)
    float out_max;      // 输出上限 (PWM最大占空比)
    float out_min;      // 输出下限 (PWM最小占空比)
} PID_Controller;

// 呼吸状态枚举
typedef enum {
    STATE_PAUSE = 0,  // 停顿/平稳
    STATE_INHALE,     // 吸气中 (冷空气进入)
    STATE_EXHALE      // 呼气中 (热气呼出)
} BreathState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define ADC_BUF_LEN 64             // DMA 缓冲区长度（过采样次数）

uint16_t adc_buffer[ADC_BUF_LEN];  // 存放 DMA 自动搬运的原始 ADC 数据
float smoothed_adc = 0.0f;         // 滤波后的平滑 ADC 值
volatile float current_temp = 30.0f;         // 最终算出来的真实温度
uint8_t adc_ready = 0;             // 标志位：告诉主程序数据准备好了

// 呼吸检测相关变量
volatile BreathState current_state = STATE_PAUSE; // 当前呼吸状态
float last_temp = 0.0f;                  // 上一次的温度 (用于算变化率)
uint8_t breath_check_timer = 0;          // 呼吸检测计时器

// 真实 NTC 温度越高，阻值越小，ADC 读数通常越小
// 【终极真实表格】10K B3950 NTC + 6.8K 分压电阻 (30℃ ~ 40℃，步进 1℃)
// 数组长度为 11 个点
const uint16_t real_lut_adc[11] = {
    2218, // 30℃
    2174, // 31℃
    2130, // 32℃
    2085, // 33℃
    2041, // 34℃
    1998, // 35℃
    1954, // 36℃
    1911, // 37℃
    1869, // 38℃
    1827, // 39℃
    1785  // 40℃
}; 
const float base_temp = 30.0f; // 表格起始温度
PID_Controller heater_pid; // 实例化一个 PID 控制器
volatile float pwm_output = 0.0f;   // 用于观察算出来的 PWM 占空比
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 查表与线性插值算法 (适配 11 个点的真实表格)
float Calculate_Temp(float adc_val) {
    // 1. 极端情况保护（超出表格范围）
    if(adc_val >= real_lut_adc[0]) return base_temp;            // 低于30度
    if(adc_val <= real_lut_adc[10]) return base_temp + 10.0f;   // 高于40度

    // 2. 遍历表格 (注意这里改成了 i < 10)
    for(int i = 0; i < 10; i++) {
        if(adc_val <= real_lut_adc[i] && adc_val > real_lut_adc[i+1]) {
            
            // 3. 线性插值计算
            float diff_adc = real_lut_adc[i] - real_lut_adc[i+1]; 
            float diff_val = real_lut_adc[i] - adc_val;           
            float ratio = diff_val / diff_adc;                      
            
            return base_temp + (float)i + ratio;
        }
    }
    return 0.0f; 
}

// 1. PID 初始化函数
void PID_Init(void) {
    heater_pid.Kp = 400.0f;       // 比例稍微调小一点，因为我们有前馈了
    heater_pid.Ki = 5.0f;         
    heater_pid.Kd = 800.0f;         
    heater_pid.target_temp = 33.0f; // 【修改】目标温度改为人体舒适的 33度
    heater_pid.integral = 0.0f;
    heater_pid.prev_error = 0.0f;
    heater_pid.out_max = 1000.0f; 
    heater_pid.out_min = 0.0f;    
}

// 【新增】积分清零函数 (呼气时调用)
void PID_Reset(void) {
    heater_pid.integral = 0.0f;
    heater_pid.prev_error = 0.0f;
}

// 2. PID 计算核心函数 (带抗积分饱和)
// 2. 升级版 PID 计算核心函数 (带积分分离与单向刹车)
// 2. 升级版 PID 计算核心函数 (带积分分离、单向刹车与【降落伞逻辑】)
float PID_Compute(float current_val) {
    // a. 计算偏差 (目标 - 当前)
    float error = heater_pid.target_temp - current_val;
    
    // b. 比例项 (P)
    float p_out = heater_pid.Kp * error;
    
    // ===================================================
    // 【新增核心：降落伞逻辑 (Asymmetric P-term)】
    // ===================================================
    // 当温度高于目标（error < 0）时，P_out 是一个巨大的负数，会掩盖掉 D 项的动作。
    // 我们把超温时的负数 P 项削弱（比如只保留 10% 的压制力）。
    // 这样只要温度出现下降趋势（D 项为正），就能立刻冲破 P 的压制，提前输出功率“托底”！
    if (error < 0.0f) {
        p_out = p_out * 0.1f; 
    }
    
    // c. 积分项 (I) 
    if (error <= 0.0f) {
        heater_pid.integral = 0.0f; // 超温依然清空积分
    } else if (error < 1.5f) {
        heater_pid.integral += error;
    } else {
        heater_pid.integral = 0.0f;
    }
    
    float i_out = heater_pid.Ki * heater_pid.integral;
    
    if(i_out > heater_pid.out_max * 0.5f) { 
        i_out = heater_pid.out_max * 0.5f;
        heater_pid.integral = i_out / heater_pid.Ki; 
    }
    
    // d. 微分项 (D) - 预测趋势
    float d_out = heater_pid.Kd * (error - heater_pid.prev_error);
    heater_pid.prev_error = error;
    
    // e. 计算总输出
    float total_out = p_out + i_out + d_out;
    
    // f. 总输出限幅
    if(total_out > heater_pid.out_max) total_out = heater_pid.out_max;
    if(total_out < heater_pid.out_min) total_out = heater_pid.out_min;
    
    return total_out;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc1);//初始化
// 启动 ADC 的 DMA 传输，把数据源源不断地搬运到 adc_buffer 数组里
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUF_LEN);
  PID_Init(); // 初始化 PID 参数
  
  // 启动 Timer 2 的 PWM 输出 (通道1，对应 PA5 引脚)
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  
  // 启动 Timer 3 的定时中断 (作为 PID 的心跳，每 10ms 触发一次)
  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {  
      // 如果 DMA 已经帮我们把 64 个数据采集满了
      if(adc_ready == 1) {
          adc_ready = 0; // 清除标志位，等待下一波数据
          
          // 计算当前温度
          current_temp = Calculate_Temp(smoothed_adc);
          
          // 此时 current_temp 就是算出来的温度了！
          // (以后我们会在这里把 current_temp 丢给 PID 算法)
      }
			// ====================================================
      // 【新增代码】专治 ADC 假死：每次循环强制停止并重新启动 ADC
      // ====================================================
      //HAL_ADC_Stop_DMA(&hadc1);
      //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUF_LEN);
      
      // 延时一下，防止 CPU 跑得太累 (实际项目中尽量不用 HAL_Delay)
      HAL_Delay(10);
 
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// 这是 HAL 库内置的弱函数，我们在这里重写它。DMA 传输完成一半和全部完成都会调用。
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if(hadc->Instance == ADC1) {
        uint32_t sum = 0;
        
        // 1. 把 64 个原始数据加起来
        for(int i = 0; i < ADC_BUF_LEN; i++) {
            sum += adc_buffer[i];
        }
        
        // 2. 求平均值，得到一个比单次读取更稳定的 12位 ADC 值
        float current_avg = (float)sum / ADC_BUF_LEN;
        
        // 3. EMA 指数移动平均滤波 (Marlin 固件的精髓)
        if(smoothed_adc == 0.0f) {
            smoothed_adc = current_avg; // 第一次运行，给个初始值
        } else {
            // 新值占 1/8 权重，老值占 7/8 权重，极其平滑！
            smoothed_adc = (smoothed_adc * 7.0f + current_avg) / 8.0f;
        }
        
        // 4. 告诉主循环，数据处理好了
        adc_ready = 1;
    }
}
// 定时器中断回调函数 (TIM3 每 10ms 触发一次)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIM3) {
        
        // ==========================================
        // 1. 硬件级保命逻辑 (最高优先级)
        // ==========================================
        // a. 传感器脱落/短路保护 (ADC值太极端说明线断了)
        if(smoothed_adc < 100 || smoothed_adc > 4050) {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0); // 强制关断加热
            return; // 直接退出，不执行后续代码
        }
        
        // b. 软件超温保护 (绝对不能烫伤气道)
        if(current_temp > 40.0f) {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0); 
            return; 
        }

        // ==========================================
        // 2. 呼吸状态检测 (每 100ms 检测一次温度变化率)
        // ==========================================
        breath_check_timer++;
        if(breath_check_timer >= 10) { // 10ms * 10 = 100ms
            breath_check_timer = 0;
            
            float temp_change = current_temp - last_temp; // 计算 100ms 内的温度变化
            last_temp = current_temp; // 更新历史值
            
            // 判断逻辑：
            // 温度突然下降超过 0.1度 -> 吸气 (冷风吹过)
            // 温度突然上升超过 0.1度 -> 呼气 (体温热气吹过)
            if(temp_change < -0.1f) {
                current_state = STATE_INHALE;
            } 
            else if (temp_change > 0.1f) {
                current_state = STATE_EXHALE;
            } 
            else {
                // 温度没怎么变，说明气流停滞
                current_state = STATE_PAUSE;
            }
        }

        // ==========================================
        // 3. 状态机与前馈 PID 融合控制
        // ==========================================
        switch(current_state) {
            
            case STATE_INHALE: {// 【吸气中：需要大量热量】
                // 前馈功率：不等温度掉下来，直接先给 400 的基础功率 (占空比40%)
                float feedforward = 400.0f; 
                // PID 根据当前温度误差进行微调
                float pid_out = PID_Compute(current_temp); 
                pwm_output = feedforward + pid_out;
                break;
            } 
            case STATE_EXHALE: {// 【呼气中：人体自带热量，停止加热】
                pwm_output = 0.0f;
                PID_Reset(); // 极其重要：清空 PID 记忆，防止下次吸气时超调
                break;
            } 
            case STATE_PAUSE: {// 【停顿中：微弱保温】
                 pwm_output = PID_Compute(current_temp); 
                break;
            }
        }

        // ==========================================
        // 4. 输出限幅与硬件执行
        // ==========================================
        if(pwm_output > 1000.0f) pwm_output = 1000.0f;
        if(pwm_output < 0.0f) pwm_output = 0.0f;
        
        // 将最终功率输出给 MOS 管
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)pwm_output);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
