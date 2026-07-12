#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "PWM.h"
#include "CAR.h"
#include "Serial.h"
#include "Servo.h"
#include "Ultrasound.h"
#include "Track.h"

uint16_t Data1;
uint8_t ObstacleMode = 0;   // 避障模式标志  0=关闭 1=开启
uint8_t TrackMode = 0;      // 循迹模式标志  0=关闭 1=开启

void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		Data1 = USART_ReceiveData(USART1);

		// ===== 手动控制指令（会退出自动模式）=====
		if(Data1 == 0x30) { Car_Stop();  ObstacleMode = 0; TrackMode = 0; }
		if(Data1 == 0x31) { Go_Ahead();  ObstacleMode = 0; TrackMode = 0; }
		if(Data1 == 0x32) { Go_Back();   ObstacleMode = 0; TrackMode = 0; }
		if(Data1 == 0x33) { Turn_Left(); ObstacleMode = 0; TrackMode = 0; }
		if(Data1 == 0x34) { Turn_Right();ObstacleMode = 0; TrackMode = 0; }
		if(Data1 == 0x35) { Self_Left(); ObstacleMode = 0; TrackMode = 0; }
		if(Data1 == 0x36) { Self_Right();ObstacleMode = 0; TrackMode = 0; }
		if(Data1 == 0x37) { Servo_SetAngle(0);   }
		if(Data1 == 0x38) { Servo_SetAngle(90);  }
		if(Data1 == 0x39) { Servo_SetAngle(180); }

		// ===== 避障模式切换 =====
		if(Data1 == 0x40)
		{
			ObstacleMode = !ObstacleMode;   // 切换避障模式
			TrackMode = 0;                  // 关闭循迹模式
			if(ObstacleMode)
			{
				Servo_SetAngle(90);         // 舵机回正
				Go_Back();                  // 开始前进（电机接线反了）
			}
			else
			{
				Car_Stop();                 // 关闭避障，停车
			}
		}

		// ===== 循迹模式切换 =====
		if(Data1 == 0x41)
		{
			TrackMode = !TrackMode;         // 切换循迹模式
			ObstacleMode = 0;               // 关闭避障模式
			if(TrackMode == 0)
			{
				Car_Stop();                 // 关闭循迹，停车
			}
		}

		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}



int main(void)
{
	Car_Init();
	Serial_Init();
	Servo_Init();
	Ultrasound_Init();
	Infrared_Init();

	Servo_SetAngle(90);                     // 舵机初始回正

	while (1)
	{
		// ========== 避障模式 ==========
		if(ObstacleMode == 1)
		{
			uint16_t dist = Test_Distance();

			if(dist > 0 && dist < 15)       // 前方有障碍物
			{
				Car_Stop();
				Delay_ms(100);

				// 扫描左侧
				Servo_SetAngle(0);
				Delay_ms(800);
				uint16_t dist_left = Test_Distance();

				// 扫描右侧
				Servo_SetAngle(180);
				Delay_ms(800);
				uint16_t dist_right = Test_Distance();

				// 舵机回正
				Servo_SetAngle(90);
				Delay_ms(500);

				// 判断哪边空间更大
				if(dist_left > dist_right && dist_left > 15)
				{
					// 左边空间大 → 原地左转
					Self_Left();
					Delay_ms(600);
				}
				else if(dist_right > 15)
				{
					// 右边空间大 → 原地右转
					Self_Right();
					Delay_ms(600);
				}
				else
				{
					// 左右都堵住了 → 原地掉头
					Self_Right();
					Delay_ms(1200);
				}

				// 后退一段距离
				Go_Ahead();
				Delay_ms(500);
				Car_Stop();
				Delay_ms(100);
				Go_Back();                  // 继续前进（电机接线反了）
			}
		}

		// ========== 循迹模式 ==========
		else if(TrackMode == 1)
		{
			uint8_t PB5 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5);
			uint8_t PB6 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6);
			uint8_t PB7 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);
			uint8_t PB8 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8);

			if(PB8 == 0 && PB6 == 0 && PB7 == 0 && PB5 == 0)
			{
				Go_Ahead();                 // 四路都在黑线上 → 直行
			}
			else if(PB8 == 1 && PB6 == 1 && PB7 == 1 && PB5 == 1)
			{
				Car_Stop();                 // 四路都离线 → 停车
			}
			else if(PB8 == 0 && PB6 == 0 && PB7 == 1 && PB5 == 1)
			{
				Self_Right();               // 车偏左 → 原地右转修正
			}
			else if(PB8 == 0 && PB6 == 0 && PB7 == 1 && PB5 == 0)
			{
				Turn_Right();               // 略偏左 → 右转
			}
			else if(PB8 == 0 && PB6 == 0 && PB7 == 0 && PB5 == 1)
			{
				Turn_Right();               // 略偏左 → 右转
			}
			else if(PB8 == 0 && PB6 == 1 && PB7 == 0 && PB5 == 0)
			{
				Turn_Left();                // 略偏右 → 左转
			}
			else if(PB8 == 1 && PB6 == 1 && PB7 == 0 && PB5 == 0)
			{
				Self_Left();                // 车偏右 → 原地左转修正
			}
			else if(PB8 == 1 && PB6 == 0 && PB7 == 0 && PB5 == 0)
			{
				Turn_Left();                // 略偏右 → 左转
			}
		}

		Delay_ms(50);
	}
}
