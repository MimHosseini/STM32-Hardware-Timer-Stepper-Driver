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

    /* Engineering Reason: Explicitly resetting state variables to prevent undefined behavior during cold boot */
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

    /* Engineering Reason: Dynamic bit-calculation to enable the correct output channel safely without hardcoding CC1E/CC1NE. 
       In STM32, TIM_CHANNEL_X maps to 0x00, 0x04, 0x08, 0x0C. Shifting 1U by this exact channel value perfectly aligns with the hardware CCxE bits. */
    uint32_t channel_enable_bit = (hstep->IsComplementary) ? (1U << (hstep->Channel + 2U)) : (1U << hstep->Channel);
    hstep->htim->Instance->CCER |= channel_enable_bit;

    /* Engineering Reason: Advanced timers (e.g., TIM1, TIM8, TIM15, TIM16, TIM17) require the Main Output Enable (MOE) bit to route the signal to physical pins */
    if (hstep->htim->Instance == TIM1 || hstep->htim->Instance == TIM16 || hstep->htim->Instance == TIM17) {
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

    /* Engineering Reason: Calculate the absolute clock division required based on the configured bus frequency */
    uint32_t total_divider = hstep->SystemClockFreq / freq_hz;
    uint32_t psc = 0;
    uint32_t arr = 0;

    /* Engineering Reason: If the clock division fits within a standard 16-bit Auto-Reload Register */
    if (total_divider <= 65536) {
        psc = 0;
        arr = total_divider - 1;
    } else {
        /* Engineering Reason: For ultra-low frequencies, distribute a portion of the division burden to the Prescaler to avoid ARR overflow */
        psc = (total_divider / 60000); 
        arr = (hstep->SystemClockFreq / ((psc + 1) * freq_hz)) - 1;
    }

    /* Engineering Reason: Apply the calculated values directly to the macro-driven Shadow Registers */
    __HAL_TIM_SET_PRESCALER(hstep->htim, psc);
    __HAL_TIM_SET_AUTORELOAD(hstep->htim, arr);
    
    /* Engineering Reason: Fix the duty cycle precisely at 50% to generate a perfectly symmetrical square wave for optimal driver triggering */
    __HAL_TIM_SET_COMPARE(hstep->htim, hstep->Channel, arr / 2);

    /* Engineering Reason: If the timer is halted, manually trigger an update event to immediately load shadow registers into active registers */
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

    /* Engineering Reason: Load the payload safely before engaging hardware to prevent race conditions in RTOS contexts */
    hstep->StepsLeft = steps;

    /* Engineering Reason: Establish rotation direction pin state */
    HAL_GPIO_WritePin(hstep->DirPort, hstep->DirPin, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* Engineering Reason: Energize motor coils (Activate driver) safely before issuing pulses */
    if (hstep->InvertEnableLogic) {
        HAL_GPIO_WritePin(hstep->EnPort, hstep->EnPin, GPIO_PIN_RESET); /* Active-Low: 0 = Enabled */
    } else {
        HAL_GPIO_WritePin(hstep->EnPort, hstep->EnPin, GPIO_PIN_SET);   /* Active-High: 1 = Enabled */
    }

    /* Engineering Reason: Reset the internal timer counter to guarantee a synchronized and aligned pulse start */
    __HAL_TIM_SET_COUNTER(hstep->htim, 0);
    __HAL_TIM_CLEAR_FLAG(hstep->htim, TIM_FLAG_UPDATE);
    
    /* Engineering Reason: Enable the Timer Update Interrupt via standard HAL macro */
    __HAL_TIM_ENABLE_IT(hstep->htim, TIM_IT_UPDATE);

    /* Engineering Reason: Dynamically assure the requested output compare pin is physically connected to the timer channel */
    uint32_t channel_enable_bit = (hstep->IsComplementary) ? (1U << (hstep->Channel + 2U)) : (1U << hstep->Channel);
    hstep->htim->Instance->CCER |= channel_enable_bit;

    /* Engineering Reason: Securely flag the state machine as active just before clock engagement */
    hstep->IsMoving = 1;

    /* Engineering Reason: Start the main timer counter (Hardware pulse generation begins instantly here) */
    __HAL_TIM_ENABLE(hstep->htim);
}

/**
 * @brief Emergency and instantaneous halt mechanism at any movement phase
 */
void Stepper_Stop(Stepper_HandleTypeDef* hstep)
{
    if (hstep == NULL) return;

    /* Engineering Reason: Halt the timer and disable the overflow interrupt simultaneously to cease processing */
    __HAL_TIM_DISABLE(hstep->htim);
    __HAL_TIM_DISABLE_IT(hstep->htim, TIM_IT_UPDATE);
    
    /* Engineering Reason: Hardware disconnect of the dynamic pulse pin to guarantee absolute zero voltage state at the physical level */
    uint32_t channel_enable_bit = (hstep->IsComplementary) ? (1U << (hstep->Channel + 2U)) : (1U << hstep->Channel);
    hstep->htim->Instance->CCER &= ~channel_enable_bit;

    /* Engineering Reason: De-energize motor coils to strictly prevent stationary overheating and mechanical resonance */
    if (hstep->InvertEnableLogic) {
        HAL_GPIO_WritePin(hstep->EnPort, hstep->EnPin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(hstep->EnPort, hstep->EnPin, GPIO_PIN_RESET);
    }

    /* Engineering Reason: Purge remaining queue buffers and release the state lock */
    hstep->StepsLeft = 0;
    hstep->IsMoving = 0;
}

/**
 * @brief Hardware pulse management handler bound to standard HAL CallBack architecture
 * @note This function avoids manual interrupt flag clearing, as the HAL layer already processes it beforehand.
 */
void Stepper_PeriodElapsedCallback(Stepper_HandleTypeDef* hstep, TIM_HandleTypeDef* htim)
{
    /* Engineering Reason: Verify the overflow event strictly belongs to this specific motor's dedicated hardware timer */
    if (htim->Instance == hstep->htim->Instance)
    {
        /* Engineering Reason: Decrement pulse count asynchronously on every hardware tick */
        if (hstep->StepsLeft > 0) {
            hstep->StepsLeft--;
        }

        /* Engineering Reason: Evaluate user-requested step completion accurately */
        if (hstep->StepsLeft == 0)
        {
            /* Engineering Reason: Disable timer strictly at hardware level to eliminate potential software-induced Ghost Pulses */
            __HAL_TIM_DISABLE(hstep->htim);
            __HAL_TIM_DISABLE_IT(hstep->htim, TIM_IT_UPDATE);

            /* Engineering Reason: Isolate the precise output pin dynamically from the timer peripheral to ensure signal termination */
            uint32_t channel_enable_bit = (hstep->IsComplementary) ? (1U << (hstep->Channel + 2U)) : (1U << hstep->Channel);
            hstep->htim->Instance->CCER &= ~channel_enable_bit;

            /* Engineering Reason: Release the mutex state to allow subsequent motion commands */
            hstep->IsMoving = 0;
        }
    }
}