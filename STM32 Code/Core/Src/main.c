/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "string.h"
#include "stm32f4xx_hal.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim2_ch1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Define PID Input
#define   kp 0.95f   // P Gain Input--good kp = 0.66f--New P = 0.60f
#define   ki 16.66f  // I Gain Input--good ki = 16.66f--New I = 10.725f
#define   kd 0.00f   // D Gain Input

// setPoint
float setpoint = 400.0f;

// Define PID limits
//maximum duty circle 690/2 = 345-Dead Time
//Haft Duty circle = 365.0f - (360ns) = 345
#define   PID_MIN_OUTPUT 0.0f
#define   PID_MAX_OUTPUT 330.0f //MAX 330 is good
// Define Kalman filter parameters
//for voltage feedback
#define   Q 0.01f // Process noise covariance
#define   R 0.5f  // Measurement noise covariance
//for current sensor
#define   Q1 0.01f // Process noise covariance
#define   R1 0.5f  // Measurement noise covariance
//for voltage sensor input
#define   Q2 0.01f // Process noise covariance
#define   R2 0.5f  // Measurement noise covariance

// Define anti-windup parameters
#define   INTEGRAL_MIN 0.0f
#define   INTEGRAL_MAX 800.0f

// Define Derivative clamp parameters
#define   tau 0.02

// ADC parameters
#define   ADC_MAX_VALUE 4095.0f // 12-bit ADC
#define   ADC_MIN_VALUE 0.0f

// Map function parameters
#define   INPUT_MIN 0.0f
#define   INPUT_MAX 5.0f  //500V = 5V

// PID Parameter
float error = 0.0f, last_error = 0.0f, proportional = 0.0f, integral = 0.0f,
		derivate = 0.0f;
float PID_out = 0.0f;
float out = 0.0f;
float PWM = 0.0f;
double input = 0.0f;
float last_feedback = 0.0f;
float UVout = 0.0f;
float Vin = 0.0f;
float Iout = 0.0f;
int UVref = 50.0f;
int Iref = 20.0f;   //Max 50A
int first_start_pwm = 40.0f; //first_start_pwm D12.5% = 83.25 ,D25% = 166.5, D45% = 333
float i = 0.0f;
int Iset = 30.0f;
float Pin = 0.0f;

char data[100];

#define   SAMPLE_TIME      10 	// 10 milliseconds
#define   SAMPLE_TIME1     10 	// 10 milliseconds
float dt = 0.0f; 		// Time interval between control loop iterations
float dt1 = 0.0f;
uint32_t last_time = 0.0f;
uint32_t current_time;
uint32_t now;
uint32_t last;

float previousMillis = 0.0f;
float interval = 100.0f;

/*-----------------------------map value------------------------------------*/
float map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
float map1(float x1, float in_min1, float in_max1, float out_min1, float out_max1) {
	return (x1 - in_min1) * (out_max1 - out_min1) / (in_max1 - in_min1) + out_min1;
}
float map2(float x2, float in_min2, float in_max2, float out_min2, float out_max2) {
	return (x2 - in_min2) * (out_max2 - out_min2) / (in_max2 - in_min2) + out_min2;
}
float map3(float x3, float in_min3, float in_max3, float out_min3, float out_max3) {
	return (x3 - in_min3) * (out_max3 - out_min3) / (in_max3 - in_min3) + out_min3;
}
/*--------------------------------------------------------------------------*/

/*------------------------------kalman_filter-------------------------------*/
float kalman_filter1(float measurement1, float dt1) {
	static float x_Est1 = 0.0f; // State estimate
	static float P1 = 1.0f;     // Estimate covariance
	static float H1 = 1.0f;  // Measurement matrix
	static float F1 = 1.0f;  // State transition matrix

	// Prediction
	float x_est_minus1 = F1 * x_Est1;
	float P_minus1 = P1 + Q1 * dt1;

	// Update
	float K1 = (P_minus1 * H1)/(H1 * P_minus1 + R1);
	x_Est1 = x_est_minus1 + K1 * (measurement1 - x_est_minus1);
	P1 = (1 - K1 * H1) * P_minus1;
	return x_Est1;
}

float kalman_filter2(float measurement2, float dt1) {
	static float x_Est2 = 0.0f; // State estimate
	static float P2 = 1.0f;     // Estimate covariance
	static float H2 = 1.0f;  // Measurement matrix
	static float F2 = 1.0f;  // State transition matrix

	// Prediction
	float x_est_minus2 = F2 * x_Est2;
	float P_minus2     = F2 * P2 + Q2 * dt1;

	// Update
	float K2 = (P_minus2 * H2)/(H2 * P_minus2 + R2);
	x_Est2 = x_est_minus2 + K2 * (measurement2 - H2 * x_est_minus2);
	P2 = (1 - K2 * H2) * P_minus2;
	return x_Est2;
}

// Function to update the Kalman filter
float Kalman_Update(float z, float delta) {
	// Kalman filter variables
	static float x = 0.0f;  // State estimate
	static float P = 1.0f;  // Estimate covariance
	static float H = 1.0f;  // Measurement matrix
	static float F = 1.0f;  // State transition matrix

	// Prediction step
	float x_pred = F * x;
	float P_pred = F * P + Q * delta;

	// Update step
	float K = P_pred * H / (H * P_pred + R);  // Kalman gain
	x = x_pred + K * (z - H * x_pred);
	P = (1 - K * H) * P_pred;
	return x;
}

/*------------------------PID COMPUTE---------------------------------------*/
void PID_comput(float setPoint, float feedback) {
	setPoint = map(setPoint, 0.0f, 500.0f, 0.0f, 330);
	feedback = map(feedback, 0.0f, 500.0f, 0.0f, 330);
	error = setPoint - feedback;

	proportional = kp * error; //Proportional--------------------------------------------------

	/* Integral term with ANTI WIND-UP */

	integral += ( ki * dt * error); // integral += ((ki * error) * dt);-----------
	if (integral > INTEGRAL_MAX) {
		integral = INTEGRAL_MAX / ki; //integral = INTEGRAL_MAX/ki;
	} else if (integral < INTEGRAL_MIN) {
		integral = INTEGRAL_MIN / ki; // integral = INTEGRAL_MIN/ki;
	}

	//-------------------------------------

	/* Derivative (band-limited differentiator)*/

	//derivate = kd * ((error - last_error)/dt);
	derivate = -(2.0f * kd * (feedback - last_feedback)/* Note: derivative on measurement, therefore minus sign in front of equation! */
	+ (2.0f * tau - dt) * derivate) / (2.0f * tau + dt);
	//-------------------------------------

	PID_out = proportional + integral + derivate; //PID OUTPUT

	/*Output Limit-------------------------*/
	if (PID_out > PID_MAX_OUTPUT) {
		integral -= (ki * error * dt); // integral =  integral + (ki * error);
		PID_out = PID_MAX_OUTPUT;
	} else if (PID_out < PID_MIN_OUTPUT) {
		//integral += (ki * error * dt);
		PID_out = PID_MIN_OUTPUT;
	}
	last_error = error;
	last_feedback = feedback;
	//------------------------------
	//return PID_out;
}
//------------------------UART Serial Print Data-----------------------------------------------------------------------------------------------------------------------------------------
void print_data(float IN1, float IN2, float IN3, float IN4, float IN5,
		float IN6, float IN7) {
	// Blink the status LED
	unsigned long currentMillis = HAL_GetTick();
	if (currentMillis - previousMillis >= interval) {
		previousMillis = currentMillis;
		sprintf(data,
				"I:%.2fA  Vf:%.2fV  PWM:%.2f  Pin:%.2fW  ut:%.2f  Targ:%.fV  Vin:%.2fV \r\n",
				IN1, IN2, IN3, IN4, IN5, IN6, IN7);
		HAL_UART_Transmit(&huart1, (uint8_t*) data, strlen(data),
				HAL_MAX_DELAY);
	}
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void ADC_Select_CH0(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

void ADC_Select_CH1(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

void ADC_Select_CH2(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

void ADC_Select_CH3(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

void ADC_Select_CH4(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);	//Pin PA8
	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);	//Pin PA5

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t adc_value[5];

	while (1) {

		ADC_Select_CH0();            //ADC1 CHANNEL0(PA0)
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		adc_value[0] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		ADC_Select_CH1();            //ADC1 CHANNEL1(PA1)
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		adc_value[1] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		ADC_Select_CH2();            //ADC1 CHANNEL0(PA2)
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		adc_value[2] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		ADC_Select_CH3();            //ADC1 CHANNEL0(PA3)
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		adc_value[3] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		ADC_Select_CH4();            //ADC1 CHANNEL0(PA4)
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		adc_value[4] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

//------------------------------------------------------------
		//voltage feedback map value of ADC1(PA0)
		input = map(adc_value[0], ADC_MIN_VALUE, ADC_MAX_VALUE, INPUT_MIN,
				INPUT_MAX);
		float Vfeedback = ((input - 0.29f) * 125.0f) / 1.079f;	      //117
		if (Vfeedback > 600.0f) {
			Vfeedback = 600.0;
		} else if (Vfeedback < 0.0f) {
			Vfeedback = 0.0f;
		}
//-----------------------------------------------------------------------------------
		//current feedback map value of ADC1(PA1)
		Iout = map1(adc_value[1], 0.0f, 4095.0f, 0.0f, 5.0f);
		Iout = (Iout - (2.427f)) / 0.040f;	 //convert voltage to current 40mV/A
		if (Iout >= 60.0f) {
			Iout = 60.0f;
		} else if (Iout < 0.0f) {
			Iout = 0.0f;
		}

//-----------------------------------------------------------------------------------
		//V_input feedback map value of ADC1(PA2)
		Vin = map2(adc_value[2], 0.0f, 4095.0f, 0.0f, 103.4f);

//-----------------------------------------------------------------------------------
		// Run filter
		float filtered1 = Kalman_Update(Vfeedback, dt1);
	    float filtered2 = kalman_filter1(Iout,dt1);
	    float filtered3 = kalman_filter2(Vin,dt1);

		now = HAL_GetTick();
		dt1 = (now - last) / 1000.0f; // Convert ms to seconds
		if (dt1 >= (SAMPLE_TIME1 / 1000.0f)) {

			last = now; // Update last
		}
		current_time = HAL_GetTick();
		dt = (current_time - last_time) / 1000.0f; // Convert ms to seconds
		if (dt >= (SAMPLE_TIME / 1000.0f)) {
			//-------------------------------------------------------------------------

			PID_comput(setpoint, filtered1); //PID_comput(setPoint, Input);
			//PI_comput(Iset, filtered2); //P_comput(setPoint, Input) For current
			last_time = current_time; // Update last_time
		}

		Pin = filtered2 * filtered3;//calculate output power

		PWM = PID_out; // Otherwise, use voltage control

//----------------Timer PWM Generator--------------------------
		TIM1->CCR1 = PWM;
		TIM2->CCR1 = PWM;
//-------------------------------------------------------------
		if (i < 1) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_Delay(300);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		}
//--------------------------PWM SHUT DOWN----------------------
		if ((Vin >= 60) || (Vin <= 40)) {//When batteries are Higher or Lower than reference, System will be shut down
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			while (1) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
				HAL_Delay(1000);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
				HAL_Delay(1000);

				if ((Vin < 60) || (Vin > 46)) {//When batteries are normal, System will be reset
					NVIC_SystemReset();//Automatic system reset
					break;
				}
			}
		}
//-------------------------------------------------------------

		if (filtered2 >= Iref) {// Overload Current
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			while (1) {

				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
				HAL_Delay(300);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
				HAL_Delay(300);

			}

		}
		//-----------------------------------------------------------
		print_data(filtered2, filtered1, PWM, Pin, PID_out, setpoint, filtered3);

//soft start-----------------------------------------------------------------------------------------------------------
		while (i <= first_start_pwm) {
			i += 0.0002f;

			filtered1 = Kalman_Update(Vfeedback, dt1);
			filtered2 = kalman_filter1(Iout, dt1);
			filtered3 = kalman_filter2(Vin, dt1);
			PID_comput(setpoint, filtered1); //PID_comput(setPoint, Input);
//					 PI_comput(Iset, filtered2); //P_comput(setPoint, Input) For current
			TIM1->CCR1 = i;
			TIM2->CCR1 = i;
			print_data(filtered2, filtered1, i, Pin, PID_out, setpoint, filtered3);
			if (i == first_start_pwm) {
				break;
			}
		}
//--------------------------------------------------------------------------------------------------------------------

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 690;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 345;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 690;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
