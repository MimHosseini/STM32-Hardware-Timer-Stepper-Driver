/**
 * ******************************************************************************
 * @file    stepper.h
 * @brief   Stepper Motor Management Library Header strictly aligned with HAL callbacks
 * ******************************************************************************
 */

#ifndef STEPPER_H_
#define STEPPER_H_

/* Engineering Reason: Update this include directive depending on your specific STM32 family */
#include "stm32f0xx_hal.h" 

/**
 * @brief Comprehensive structure for stepper motor hardware binding and state management
 */
typedef struct {
    TIM_HandleTypeDef* htim;             /* Pointer to the HAL Timer Handle structure */
    uint32_t           Channel;          /* Timer Channel (e.g., TIM_CHANNEL_1) */
    GPIO_TypeDef*      DirPort;          /* GPIO Port for Direction pin */
    uint16_t           DirPin;           /* GPIO Pin number for Direction */
    GPIO_TypeDef*      EnPort;           /* GPIO Port for Driver Enable pin */
    uint16_t           EnPin;            /* GPIO Pin number for Driver Enable */
    
    uint32_t           SystemClockFreq;  /* Timer input clock frequency (e.g., 48000000 Hz) */
    uint8_t            IsComplementary;  /* 1 for complementary channels (e.g., CH1N), 0 for standard channels */
    uint8_t            InvertEnableLogic;/* 1 for Active-Low drivers, 0 for Active-High drivers */
    
    volatile uint32_t  StepsLeft;        /* Remaining pulses (Countdown handled inside the ISR) */
    volatile uint8_t   IsMoving;         /* Motor motion state flag (1: Moving, 0: Stopped) */
} Stepper_HandleTypeDef;

/* --- Core Application Programming Interface (API) --- */
void Stepper_Init(Stepper_HandleTypeDef* hstep);
void Stepper_SetSpeed(Stepper_HandleTypeDef* hstep, uint32_t freq_hz);
void Stepper_Move(Stepper_HandleTypeDef* hstep, uint32_t steps, uint8_t direction);
void Stepper_Stop(Stepper_HandleTypeDef* hstep);

/* --- Standard HAL Callback Integration (Hardware-layer abstraction preserved) --- */
void Stepper_PeriodElapsedCallback(Stepper_HandleTypeDef* hstep, TIM_HandleTypeDef* htim);

#endif /* STEPPER_H_ */