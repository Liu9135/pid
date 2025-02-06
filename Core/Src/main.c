/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_dwt.h"
#include "motor_simulation.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float disturbance=0;
float a;
float Velocity;
float Angle;
float Current;
float V;
float A1;
float A2;
float t  = 0;
float mode=3;
float type =3;
typedef struct PID
{   
	  float i;
	  float d; 
    float Kp;
    float Ki;
    float Kd;
    float setpoint;
    float err;
    float last_err;
    float last_last_err;
	  float output;
} pid_t;
pid_t pidv;
pid_t pida;
void pidv_Init(pid_t *pid){
	pid -> i=0;
	pid -> d=0;
	if(mode==1&&type==1){
	pid -> Kp=3.00;
	pid -> Ki=2.00;
	pid -> Kd=0.00;
	pid -> setpoint=10.00;
	}
	if(mode==1&&type==3){
	pid -> Kp=0.65;
	pid -> Ki=0.00;
	pid -> Kd=0.00;
	}
	if(mode==2&&type==1){
	pid -> Kp=8.00;
	pid -> Ki=5.00;
	pid -> Kd=0.00;
	}
	if(mode==2&&type==3){
	pid -> Kp=8.00;
	pid -> Ki=5.00;
	pid -> Kd=0.00;
	}
	if(mode==3){
	pid -> Kp=3.00;
	pid -> Ki=0.00;
	pid -> Kd=0.00;
	}
	pid -> err=0;
	pid -> last_err=pid -> setpoint;	
	pid -> last_last_err=0.00;
	pid -> output =0;
}
void pida_Init(pid_t *pid){
	pid -> i=0;
	pid -> d=0;
	if(mode==1&&type==2){
	pid -> Kp=3.00;
	pid -> Ki=0.20;
	pid -> Kd=2.00;
	pid -> setpoint=2*3.1415926;
	}
	if(mode==1&&type==3){
	pid -> Kp=5.00;
	pid -> Ki=0.30;
	pid -> Kd=1.95;
	pid -> setpoint=2*3.1415926;
	}
	if(mode==2&&type==2){
	pid -> Kp=3.00;
	pid -> Ki=0.00;
	pid -> Kd=0.00;
	}
	if(mode==2&&type==3){
	pid -> Kp=5.15;
	pid -> Ki=0.00;
	pid -> Kd=0.00;
	}
	if(mode==3){
	pid -> Kp=3.00;
	pid -> Ki=0.20;
	pid -> Kd=2.00;
	}
	pid -> err=0;
	pid -> last_err=pid -> setpoint;	
	pid -> last_last_err=0.00;
	pid -> output =0;
}
motorObject_t Motor;
float PID_VCalc(float *Measure,float dt){
	  pidv.err=pidv.setpoint-*Measure;
		pidv.i+=pidv.err*dt;
		pidv.d=(pidv.err-pidv.last_err)/dt;
		pidv.last_last_err=pidv.last_err;
		pidv.last_err=pidv.err;
	  if(pidv.i>MAX_I){
		pidv.i=MAX_I;
		}
		if(pidv.i<-1*MAX_I){
		pidv.i=-1*MAX_I;
		}
		pidv.output=pidv.Kp*pidv.err+pidv.Ki*pidv.i+pidv.Kd*pidv.d;
	return pidv.output;
}
float PID_ACalc(float *Measure,float dt){
	  pida.err=pida.setpoint-*Measure;
		pida.i+=pida.err*dt;
		pida.d=(pida.err-pida.last_err)/dt;
		pida.last_last_err=pida.last_err;
		pida.last_err=pida.err;
	if(pida.i>MAX_I){
		pida.i=MAX_I;
		}
		if(pida.i<-1*MAX_I){
		pida.i=-1*MAX_I;
		}
		pida.output=pida.Kp*pida.err+pida.Ki*pida.i+pida.Kd*pida.d;
		if(type==2){
	return pida.output;
		}
		if(type==3){
			pidv.setpoint=pida.output;
		pida.output=PID_VCalc(&Motor.MeasureVelocity,dt);
			return pida.output;
		}
}
uint32_t DWT_CNT;
float dt =0 ;
float input=0;
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
  /* USER CODE BEGIN 2 */
DWT_Init(72);
Motor_Object_Init(&Motor);
pidv_Init(&pidv);
pida_Init(&pida);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { dt=DWT_GetDeltaT(&DWT_CNT);
		t+=dt;
		Current = Get_Motor_Current(&Motor);
    Velocity = Get_Motor_Velocity(&Motor);
    Angle = Get_Motor_Angle(&Motor);
		if(type==1){
     input = PID_VCalc(&Motor.MeasureVelocity,dt);
		}
		if(type==2||type==3){
		 input = PID_ACalc(&Motor.MeasureAngle,dt);
		}
		if(mode==2&&type!=3){
		pidv.setpoint=t;
		pida.setpoint=2*3.14159*sin(5*t);
		}
		if(mode==2&&type==3){
		pida.setpoint=2*3.14159*sin(5*t);
		}
		if(mode==3&&type!=3){
    pidv.setpoint=10*sin(15*t);
		pida.setpoint=0;
		}
		if(mode==3&&type==3){
		pida.setpoint=0;
		}
		 Motor_Simulation(&Motor,input+disturbance, dt);
		V=7.07*sin(15*t-2.58/6);
		A1=0.707*2*3.14159*sin(5*t-1.5);
		A2=0.707*2*3.14159*sin(5*t-0.83);
		HAL_Delay(1);
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
