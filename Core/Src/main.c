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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PWM_MAX 1200            /* PWM上限；TIM3周期仍为3599。 */
#define RPM_MAX 120             /* 最大目标转速，单位RPM。 */
#define L_REV 1
#define R_REV 1
#define L_ENC_SIGN 1            /* 手转车轮向前，enc[0]应增加；否则改为-1。 */
#define R_ENC_SIGN -1           /* 右轮前进时计数取正。 */
#ifndef ENC_CPR
#define ENC_CPR 1468.2f         /* 每圈A/B四倍频计数，已与手转复测核对。 */
#endif
#define KP 10.0f                /* P由7略增至10，提高相同误差下的驱动力。 */
#define KD 0.02f

#if PWM_MAX < 1 || PWM_MAX > 3599 || RPM_MAX < 1 || RPM_MAX > 3276
#error "Invalid motor limits"
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static volatile int16_t left, right; /* 目标值单位0.1 RPM，不是PWM。 */
static uint8_t rx, frame[8], n;
static uint8_t tx[17] = {0xAA, 0x66}; /* 时间、左计数、右计数、和校验、帧尾。 */
static uint32_t sent;
static volatile uint32_t err;
static volatile int32_t enc[2];       /* 累计四倍频计数，供闭环和串口反馈使用。 */
static volatile float measured[2];    /* 实测RPM，可在调试器查看。 */
static volatile int16_t pwm[2];
static uint8_t state[2];
static float old_rpm[2];             /* 上一次实测转速，仅用于D项。 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static int16_t Limit(int16_t p);
static void Motor(GPIO_TypeDef *port, uint16_t a, uint16_t b, uint32_t ch, int16_t p);
static void Run(void);
static void Output(int16_t l, int16_t r);
static uint8_t Phase(uint8_t i);
static void Control(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* HEX：AA 55 左高 左低 右高 右低 0D 0A，115200/8N1。
 * 两个有符号16位大端数，单位0.1 RPM；例如300表示30.0 RPM。
 * 每20ms回传17字节：AA 66 时间ms(4) 左计数(4) 右计数(4) 和校验 0D 0A。
 * 三个32位数均为大端，计数有符号；校验为前14字节之和的低8位。
 */
static int16_t Limit(int16_t p)
{
  if (p > RPM_MAX * 10) return RPM_MAX * 10;
  if (p < -RPM_MAX * 10) return -RPM_MAX * 10;
  return p;
}

/* 按实测接线：左轮A/B为PB8/PB9，右轮为PB6/PB7。 */
static uint8_t Phase(uint8_t i)
{
  uint32_t bits = GPIOB->IDR;
  uint16_t a = i ? R_ENC_A_Pin : L_ENC_A_Pin;
  uint16_t b = i ? R_ENC_B_Pin : L_ENC_B_Pin;
  return ((bits & a) ? 2 : 0) | ((bits & b) ? 1 : 0);
}

static void Motor(GPIO_TypeDef *port, uint16_t a, uint16_t b, uint32_t ch, int16_t p)
{
  HAL_GPIO_WritePin(port, a, p > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(port, b, p < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim3, ch, p < 0 ? -p : p);
}

static void Output(int16_t l, int16_t r)
{
  Motor(RIGHT_IN1_GPIO_Port, RIGHT_IN1_Pin, RIGHT_IN2_Pin,
        TIM_CHANNEL_1, L_REV ? -l : l);
  Motor(LEFT_IN1_GPIO_Port, LEFT_IN1_Pin, LEFT_IN2_Pin,
        TIM_CHANNEL_2, R_REV ? -r : r);
}

/* TIM2固定100Hz调用。PD闭环留在下位机，上位机只给RPM目标。 */
static void Control(void)
{
  static int32_t previous[2];
  int32_t count[2];
  int16_t target[2];
  uint32_t key = __get_PRIMASK();
  __disable_irq();
  count[0] = enc[0]; count[1] = enc[1];
  target[0] = left; target[1] = right;
  __set_PRIMASK(key);

  for (uint8_t i = 0; i < 2; i++)
  {
    int32_t delta = (int32_t)((uint32_t)count[i] - (uint32_t)previous[i]);
    previous[i] = count[i];
    if (ENC_CPR <= 0.0f)          /* 未标定时可手转计数，电机保持关闭。 */
    {
      measured[i] = old_rpm[i] = 0;
      pwm[i] = 0;
      continue;
    }
    measured[i] = delta * 6000.0f / ENC_CPR;
    float change = (measured[i] - old_rpm[i]) * 100.0f;
    old_rpm[i] = measured[i];
    if (target[i] == 0)
    {
      pwm[i] = 0;
      continue;
    }
    float rpm = target[i] * 0.1f;
    /* D项对实测速度求导，避免目标变化时产生微分冲击；没有积分项。 */
    float output = KP * (rpm - measured[i]) - KD * change;
    if (output > PWM_MAX) output = PWM_MAX;
    if (output < -PWM_MAX) output = -PWM_MAX;
    pwm[i] = (int16_t)output;
  }
  Output(pwm[0], pwm[1]);
}

/* 将32位数按大端顺序写入反馈包。 */
static void Put(uint8_t *p, uint32_t value)
{
  for (uint8_t i = 0; i < 4; i++) p[i] = value >> (24 - i * 8);
}

/* 只负责反馈；电机输出统一由TIM2更新，避免两个地方交替写PWM。 */
static void Run(void)
{
  uint32_t key = __get_PRIMASK();
  __disable_irq();
  uint32_t now = HAL_GetTick();
  uint8_t send = (uint32_t)(now - sent) >= 20 && huart1.gState == HAL_UART_STATE_READY;
  int32_t count[2] = {enc[0], enc[1]}; /* 与时间在同一临界区快照。 */
  __set_PRIMASK(key);

  if (!send) return;
  Put(tx + 2, now);
  Put(tx + 6, (uint32_t)count[0]);
  Put(tx + 10, (uint32_t)count[1]);
  tx[14] = 0;
  for (uint8_t i = 0; i < 14; i++) tx[14] += tx[i];
  tx[15] = 0x0D; tx[16] = 0x0A;
  if (HAL_UART_Transmit_IT(&huart1, tx, sizeof(tx)) == HAL_OK) sent = now;
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

/* 两路PWM初值均为0，启动成功后再使能TB6612。 */
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
    Error_Handler();
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);

  __HAL_UART_CLEAR_OREFLAG(&huart1);
  HAL_UART_Receive_IT(&huart1, &rx, 1);
  state[0] = Phase(0); state[1] = Phase(1);
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) Error_Handler();
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (huart1.RxState == HAL_UART_STATE_READY)
      HAL_UART_Receive_IT(&huart1, &rx, 1);
    Run();
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
}

/* USER CODE BEGIN 4 */

/* 每收到一个字节就检查最近8字节，支持拆包、连续包和错位后恢复。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
  if (uart != &huart1 || uart->ErrorCode != HAL_UART_ERROR_NONE)
    return;

  for (uint8_t i = 0; i < 7; i++)
    frame[i] = frame[i + 1];
  frame[7] = rx;
  if (n < 8)
    n++;

  if (n == 8 && frame[0] == 0xAA && frame[1] == 0x55 &&
      frame[6] == 0x0D && frame[7] == 0x0A)
  {
    left = Limit((int16_t)((uint16_t)frame[2] << 8 | frame[3]));
    right = Limit((int16_t)((uint16_t)frame[4] << 8 | frame[5]));
    n = 0;                      /* 完整帧已消费，避免重复解析重叠数据。 */
  }
  HAL_UART_Receive_IT(uart, &rx, 1);
}

/* 串口错误后丢弃残包并恢复接收，保留正在进行的发送。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
  if (uart != &huart1)
    return;
  err++;
  n = 0;
  HAL_UART_AbortReceive(uart);
  __HAL_UART_CLEAR_OREFLAG(uart);
  HAL_UART_Receive_IT(uart, &rx, 1);
}
/* A/B四倍频状态表；非法的跨状态跳变不虚构计数。 */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
  static const int8_t step[16] = {0,1,-1,0, -1,0,0,1, 1,0,0,-1, 0,-1,1,0};
  uint8_t i;
  if (pin & (L_ENC_A_Pin | L_ENC_B_Pin)) i = 0;
  else if (pin & (R_ENC_A_Pin | R_ENC_B_Pin)) i = 1;
  else return;
  uint8_t next = Phase(i);
  int32_t delta = step[state[i] * 4 + next] * (i ? R_ENC_SIGN : L_ENC_SIGN);
  enc[i] = (int32_t)((uint32_t)enc[i] + delta);
  state[i] = next;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
  if (timer->Instance == TIM2) Control();
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
