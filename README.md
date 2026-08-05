# Advanced Hardware-Timer Stepper Motor Driver for STM32

![License](https://img.shields.io/badge/License-MIT-blue.svg)
![Architecture](https://img.shields.io/badge/Architecture-Hardware%20Timer%20%7C%20Non--Blocking-success)
![Hardware](https://img.shields.io/badge/Hardware-STM32%20%7C%20ARM%20Cortex--M-orange)

## 📌 Overview
An industrial-grade, fully asynchronous, and zero-latency stepper motor driver for STM32 microcontrollers. Designed specifically for time-critical embedded systems, this library completely offloads pulse generation to the STM32 hardware timers, ensuring the main CPU loop is never blocked. 

Engineered by **Mohamad Hosseini** at **Esfahan Drive**, this driver focuses on robust performance for precision motion control, intelligent algorithms, and power electronics applications where CPU stalling is unacceptable.

## ✨ Key Features
* **Zero CPU Blocking:** Hardware timers handle the high-frequency pulse generation automatically, leaving the CPU 100% free for main application logic or AI tasks.
* **Dynamic Speed Calculation:** The algorithm dynamically calculates optimal Prescaler (PSC) and Auto-Reload Register (ARR) values to generate highly accurate frequencies, even down to 1Hz on 16-bit timers.
* **Ghost Pulse Prevention:** Synchronous timer shutdown inside the ISR perfectly isolates the Output Compare pin the moment the exact step count is reached.
* **Multi-Axis Object-Oriented Design:** Encapsulates states, timer handles, and pins within a `Stepper_HandleTypeDef` structure, allowing multiple motor instances simultaneously.
* **Advanced Hardware Support:** Natively supports both standard timer channels and complementary channels (e.g., `CH1N`), alongside configurable Active-Low/Active-High enable logic.

## 🛠️ Hardware Requirements & CubeMX Configuration
To use this library, configure your STM32 via STM32CubeMX:

1. **Timer Configuration:**
   * Enable a Hardware Timer (e.g., `TIM16`) and configure a channel as **PWM Generation** or **Output Compare**.
   * Enable the **Timer Update Interrupt** in the NVIC settings.
2. **GPIO Configuration:** 
   * Configure two standard Output GPIOs for the **DIR** (Direction) and **EN** (Enable) pins.

## 🚀 How to Use (Step-by-Step)

### 1. Include and Instantiate
Include the header file and create a stepper handle object:
```c
#include "stepper.h"

// Instantiate a multi-axis object
Stepper_HandleTypeDef AxisX_Stepper; 
```

### 2. Initialization
Bind your configured hardware pins and timer parameters to the handle in your `main()` function:
```c
// Bind physical pins and timer to the handle
AxisX_Stepper.htim              = &htim16;          // HAL Timer Handle
AxisX_Stepper.Channel           = TIM_CHANNEL_1;    // Timer Channel
AxisX_Stepper.SystemClockFreq   = 48000000;         // Timer input clock frequency (e.g., 48MHz)
AxisX_Stepper.IsComplementary   = 1;                // 1 for complementary (CHxN), 0 for standard (CHx)
AxisX_Stepper.InvertEnableLogic = 1;                // 1 for Active-Low, 0 for Active-High
AxisX_Stepper.DirPort           = GPIOA;
AxisX_Stepper.DirPin            = GPIO_PIN_4;
AxisX_Stepper.EnPort            = GPIOA;
AxisX_Stepper.EnPin             = GPIO_PIN_5;

// Initialize the stepper state machine securely
Stepper_Init(&AxisX_Stepper);
```

### 3. Service the ISR
Route the standard HAL Timer Period Elapsed Callback to the library's ISR handler to manage step counting:
```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Pass the interrupt event to the library without blocking
    Stepper_PeriodElapsedCallback(&AxisX_Stepper, htim);
}
```

### 4. Move the Motor Asynchronously
Set the target frequency (speed) and command the motor to move. The function returns instantly:
```c
// Set target speed to 1000 Hz (1000 steps per second)
Stepper_SetSpeed(&AxisX_Stepper, 1000);

// Move 3200 steps in the forward direction (1)
// Executes instantly, zero CPU blocking!
Stepper_Move(&AxisX_Stepper, 3200, 1);

while (1)
{
    // The CPU is completely free here. 
    // You can check the motion status seamlessly:
    if (AxisX_Stepper.IsMoving == 0)
    {
        // Motion is complete, you can trigger the next action
        HAL_Delay(2000); 
        Stepper_SetSpeed(&AxisX_Stepper, 500);
        Stepper_Move(&AxisX_Stepper, 1600, 0); // Move reverse
    }
}
```

## ⚙️ Advanced API
* **`Stepper_Stop(&AxisX_Stepper);`**: Immediately disables the hardware timer and cuts off the motor current safely, preventing stationary over-heating.
* **`Stepper_SetSpeed(&AxisX_Stepper, freq_hz);`**: Can be called dynamically to change the motor's operating frequency.

## 📄 License
This project is open-source and available under the MIT License. Feel free to fork, modify, and use it in both personal and commercial projects.
