/**
 * ******************************************************************************
 * @file    stepper.c
 * @brief   Stepper motor control functions implementation fully compliant with HAL architecture
 * ******************************************************************************
 */

#include "stepper.h"
#include <stddef.h>

/**
 * @brief Software initialization and fail-safe hardware pin state enforcement at startup
 */
void Stepper_Init(Stepper_HandleTypeDef* hstep)
{
    if (hstep == NULL) return;

    hstep->StepsLeft = 0;
    hstep->IsMoving = 0;

    /* Engineering Reason: Disable the driver initially to prevent motor lock-up and high current draw during boot */
    if (hstep->InvertEnableLogic) {
        HAL_GPIO_WritePin(hstep->EnPort, hstep->EnPin, GPIO_PIN_SET);   /* Active-Low Logic: 1 = Disabled */
    } else {
        HAL_GPIO_WritePin(hstep->EnPort, hstep->EnPin, GPIO_PIN_RESET); /* Active-High Logic: 0 = Disabled */
    }

    /* Engineering Reason: Force the compare value to zero to ensure no accidental pulse generation occurs */
    __HAL_TIM_SET_COMPARE(hstep->htim, hstep->Channel, 0);

    /* Engineering Reason: Enable channel output connection in hardware registers without starting the main counter */
    if (hstep->IsComplementary) {
        hstep->htim->Instance->CCER |= TIM_CCER_CC1NE;
    } else {
        hstep->htim->Instance->CCER |= TIM_CCER_CC1E;
    }

    /* Engineering Reason: Advanced timers (e.g., TIM1, TIM16) require the Main Output Enable (MOE) bit to be set */
    if (hstep->htim->Instance == TIM1 || hstep->htim->Instance == TIM16) {
        hstep->htim->Instance->BDTR |= TIM_BDTR_MOE;
    }
}

/**
 * @brief Dynamic calculation and adjustment of Prescaler and ARR for various speed profiles
 * @note This algorithm ensures highly accurate pulse generation even at very low frequencies (down to 1Hz) on 16-bit timers.
 */
void Stepper_SetSpeed(Stepper_HandleTypeDef* hstep, uint32_t freq_hz)
{
    if (hstep == NULL || freq_hz == 0) return;

    /* Calculate the absolute clock division required */
    uint32_t total_divider = hstep->SystemClockFreq / freq_hz;
    uint32_t psc = 0;
    uint32_t arr = 0;

    /* Engineering Reason: If the clock division fits within a standard 16-bit register */
    if (total_divider <= 65536) {
        psc = 0;
        arr = total_divider - 1;
    } else {
        /* Engineering Reason: For ultra-low frequencies, shift a portion of the division burden to the Prescaler */
        psc = (total_divider / 60000); 
        arr = (hstep->SystemClockFreq / ((psc + 1) * freq_hz)) - 1;
    }

    /* Apply the calculated values to the Shadow Registers */
    __HAL_TIM_SET_PRESCALER(hstep->htim, psc);
    __HAL_TIM_SET_AUTORELOAD(hstep->htim, arr);
    
    /* Engineering Reason: Fix the duty cycle precisely at 50% to generate a perfectly symmetrical square wave */
    __HAL_TIM_SET_COMPARE(hstep->htim, hstep->Channel, arr / 2);

    /* Engineering Reason: If the timer is halted, manually trigger an update event to immediately load shadow registers */
    if (!(hstep->htim->Instance->CR1 & TIM_CR1_CEN)) {
        hstep->htim->Instance->EGR = TIM_EGR_UG;
        __HAL_TIM_CLEAR_FLAG(hstep->htim, TIM_FLAG_UPDATE); /* Clear the flag triggered by the manual software update */
    }
}

/**
 * @brief Loads step count and initiates hardware-driven pulse generation
 */
void Stepper_Move(Stepper_HandleTypeDef* hstep, uint32_t steps, uint8_t direction)
{
    if (hstep == NULL || steps == 0 || hstep->IsMoving) return;

    hstep->StepsLeft = steps;
    hstep->IsMoving = 1;

    /* Establish rotation direction pin state */
    HAL_GPIO_WritePin(hstep->DirPort, hstep->DirPin, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* Engineering Reason: Energize motor coils (Activate driver) safely before issuing pulses */
    if (hstep->InvertEnableLogic) {
        HAL_GPIO_WritePin(hstep->EnPort, hstep->EnPin, GPIO_PIN_RESET); /* Active-Low: 0 = Enabled */
    } else {
        HAL_GPIO_WritePin(hstep->EnPort, hstep->EnPin, GPIO_PIN_SET);   /* Active-High: 1 = Enabled */
    }

    /* Engineering Reason: Reset the internal timer counter to guarantee a synchronized and aligned start */
    __HAL_TIM_SET_COUNTER(hstep->htim, 0);
    __HAL_TIM_CLEAR_FLAG(hstep->htim, TIM_FLAG_UPDATE);
    
    /* Enable the Timer Update Interrupt via standard HAL macro */
    __HAL_TIM_ENABLE_IT(hstep->htim, TIM_IT_UPDATE);

    /* Engineering Reason: Assure the output compare pin is physically connected to the timer channel */
    if (hstep->IsComplementary) {
        hstep->htim->Instance->CCER |= TIM_CCER_CC1NE;
    } else {
        hstep->htim->Instance->CCER |= TIM_CCER_CC1E;
    }

    /* Start the main timer counter (Hardware pulse generation begins here) */
    __HAL_TIM_ENABLE(hstep->htim);
}

/**
 * @brief Emergency and instantaneous halt mechanism at any movement phase
 */
void Stepper_Stop(Stepper_HandleTypeDef* hstep)
{
    if (hstep == NULL) return;

    /* Halt the timer and disable the overflow interrupt */
    __HAL_TIM_DISABLE(hstep->htim);
    __HAL_TIM_DISABLE_IT(hstep->htim, TIM_IT_UPDATE);
    
    /* Engineering Reason: Hardware disconnect of the pulse pin to guarantee absolute zero state at the physical level */
    if (hstep->IsComplementary) {
        hstep->htim->Instance->CCER &= ~TIM_CCER_CC1NE;
    } else {
        hstep->htim->Instance->CCER &= ~TIM_CCER_CC1E;
    }

    /* Engineering Reason: De-energize motor coils to strictly prevent stationary overheating */
    if (hstep->InvertEnableLogic) {
        HAL_GPIO_WritePin(hstep->EnPort, hstep->EnPin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(hstep->EnPort, hstep->EnPin, GPIO_PIN_RESET);
    }

    hstep->StepsLeft = 0;
    hstep->IsMoving = 0;
}

/**
 * @brief Hardware pulse management handler bound to standard HAL CallBack architecture
 * @note This function avoids manual interrupt flag clearing, as the HAL layer already processes it beforehand.
 */
void Stepper_PeriodElapsedCallback(Stepper_HandleTypeDef* hstep, TIM_HandleTypeDef* htim)
{
    /* Verify the overflow event strictly belongs to this specific motor's dedicated timer */
    if (htim->Instance == hstep->htim->Instance)
    {
        if (hstep->StepsLeft > 0) {
            hstep->StepsLeft--;
        }

        /* Evaluate user-requested step completion */
        if (hstep->StepsLeft == 0)
        {
            /* Engineering Reason: Disable timer strictly at hardware level to eliminate potential Ghost Pulses */
            __HAL_TIM_DISABLE(hstep->htim);
            __HAL_TIM_DISABLE_IT(hstep->htim, TIM_IT_UPDATE);

            /* Isolate the output pin from the timer peripheral */
            if (hstep->IsComplementary) {
                hstep->htim->Instance->CCER &= ~TIM_CCER_CC1NE;
            } else {
                hstep->htim->Instance->CCER &= ~TIM_CCER_CC1E;
            }

            hstep->IsMoving = 0;
        }
    }
}