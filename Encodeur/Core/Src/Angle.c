/*
 * Angle.c
 *
 *  Created on: Mar 29, 2024
 *      Author: asus
 */
#include "Angle.h"
#include "main.h"
#include "gpio.h" // Include necessary header files for GPIO and timer configurations
#define ENCODER_A_PIN GPIO_PIN_2 // Replace with Tach B
#define ENCODER_B_PIN GPIO_PIN_3 // Replace with Tach A
#define SAMPLE_TIME_MS 10 // Sampling time
#define ENCODER_RESOLUTION 360 // Encoder resolution in PPR
/*
void MX_GPIO_Init(void) {


	HAL_Init();
	  // Configure encoder pins as input with rising edge interrupts

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = ENCODER_A_PIN | ENCODER_B_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // Rising edge interrupt
	GPIO_InitStruct.Pull = GPIO_NOPULL;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // Replace port based on pin assignment

	  // Enable and set priority for encoder interrupts (adjust priority as needed)
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0); // Assuming interrupts on EXTI1
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);


}

void MX_TIM2_Init(void) {


	TIM_HandleTypeDef htim2;

	  // Configure timer for desired sampling frequency based on SAMPLE_TIME_MS

	htim2.Instance = TIM2;
	htim2.Init.Period = (SystemCoreClock / 1000) * SAMPLE_TIME_MS - 1; // Adjust for desired frequency
	htim2.Init.Prescaler = 0;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim2);
	// Enable timer interrupt
	HAL_TIM_Base_Start_IT(&htim2);
}
*/
float calculate_angle(uint32_t encoder_count) {
    // Add angle calculation logic here
    float angle = ((float)encoder_count / ENCODER_RESOLUTION) * 360.0f;
    return(angle);
}


