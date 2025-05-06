#include <stdint.h>
#include <math.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"

#define PI 3.14159265358979323846
#define STEPS_PER_45_DEGREES 256

// VL53L1X Sensor
uint16_t dev = 0x29;
int status = 0;
uint8_t dataReady = 0;
uint16_t Distance;
volatile unsigned long motor_status = 0;
volatile unsigned long data_acquisition = 0;  // Flag for data acquisition status

// Stepper Motor Pins (PH0 - PH3)
#define MOTOR_STEP_0 0b0001  
#define MOTOR_STEP_1 0b0010  
#define MOTOR_STEP_2 0b0100  
#define MOTOR_STEP_3 0b1000  

const uint8_t clockwise_stepper[4] = {
    0b0011,  // Step 1: PH0 + PH1 
    0b0110,  // Step 2: PH1 + PH2
    0b1100,  // Step 3: PH2 + PH3
    0b1001   // Step 4: PH3 + PH0
};

const uint8_t counterclockwise_stepper[4] = {
    0b1001,  // Step 1: PH0 + PH1 
    0b1100,  // Step 2: PH1 + PH2
    0b0110,  // Step 3: PH2 + PH3
    0b0011   // Step 4: PH3 + PH0
};
int stepIndex = 0;

void PortH_INIT(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0);
    GPIO_PORTH_DIR_R |= (MOTOR_STEP_0 | MOTOR_STEP_1 | MOTOR_STEP_2 | MOTOR_STEP_3);
    GPIO_PORTH_DEN_R |= (MOTOR_STEP_0 | MOTOR_STEP_1 | MOTOR_STEP_2 | MOTOR_STEP_3);
}

void PortJ_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;          // Activate clock for Port J
  while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};      // Allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x03;                      // Make PJ0 and PJ1 input 
  GPIO_PORTJ_DEN_R |= 0x03;                       // Enable digital I/O on PJ0 and PJ1
  GPIO_PORTJ_PCTL_R &= ~0x000000FF;               // Configure PJ0 and PJ1 as GPIO 
  GPIO_PORTJ_AMSEL_R &= ~0x03;                // Disable analog functionality on PJ0 and PJ1    
  GPIO_PORTJ_PUR_R |= 0x03;                  // Enable weak pull-up resistor on PJ0 and PJ1
}

void PortJ_Interrupt_Init(void){
    GPIO_PORTJ_IS_R = 0;                       // PJ0 and PJ1 are edge-sensitive 
    GPIO_PORTJ_IBE_R = 0;                    // PJ0 and PJ1 are not triggered by both edges 
    GPIO_PORTJ_IEV_R = 0;                    // PJ0 and PJ1 are falling edge events 
    GPIO_PORTJ_ICR_R = 0x03;                   // Clear interrupt flag by setting proper bit in ICR register
    GPIO_PORTJ_IM_R = 0x03;                    // Arm interrupt on PJ0 and PJ1 by setting proper bits in IM register
    NVIC_EN1_R = 0x00080000;                   // Enable interrupt 51 in NVIC (which is in Register EN1)
    NVIC_PRI12_R = 0xA0000000;                // Set interrupt priority to 5
}

void GPIOJ_IRQHandler(void){
    if ((GPIO_PORTJ_RIS_R & 0x01) == 0x01) {   // PJ0 (Data Acquisition Button)
        data_acquisition = !data_acquisition;  // Toggle data acquisition
        if (data_acquisition) {
           UART_printf("Data acquisition started.\r\n");
        } else {
            UART_printf("Data acquisition stopped.\r\n");
        }
    }
    if ((GPIO_PORTJ_RIS_R & 0x02) == 0x02) {   // PJ1 (Motor + Data Acquisition Button)
        if (data_acquisition == 0) {
            data_acquisition = 1;  // Start data acquisition if it's not already started
        }
        motor_status = !motor_status;  // Toggle motor and data acquisition state
        if (motor_status) {
            UART_printf("Motor and data acquisition started.\r\n");
        } else {
           UART_printf("Motor and data acquisition stopped.\r\n");
        }
    }
    GPIO_PORTJ_ICR_R = 0x03;  // Clear interrupt flag by setting proper bit in ICR register
}

void I2C_Init(void) {
    SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
    while ((SYSCTL_PRGPIO_R & 0x02) == 0) {}
    GPIO_PORTB_AFSEL_R |= 0x0C;
    GPIO_PORTB_ODR_R |= 0x08;
    GPIO_PORTB_DEN_R |= 0x0C;
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFF00FF) + 0x00002200;
    I2C0_MCR_R = 0x10;
    I2C0_MTPR_R = 0x3B;
}

void stepper_step(void) {
    GPIO_PORTH_DATA_R = clockwise_stepper[stepIndex];
    stepIndex = (stepIndex + 1) % 4;
}

void counterstepper(void) {
    GPIO_PORTH_DATA_R = counterclockwise_stepper[stepIndex];
    stepIndex = (stepIndex + 1) % 4;
}

void scan360_and_send_uart(void) {
    UART_printf("Starting 360\xB0 Scan:\r\n");
    UART_printf("Index,Angle(Deg),X(mm),Y(mm),Z(mm)\r\n");

    int x_offset = 0;
    while (motor_status) {  // Run continuously until PJ1 is pressed to stop the motor
        for (int i = 0; i < 8; i++) {
					FlashLED2(1); //Blink for bus speed  (12MHz least sig digit = 0)
            if (data_acquisition) {  // Only acquire data if enabled
                while (dataReady == 0) {
                    VL53L1X_CheckForDataReady(dev, &dataReady);
                    FlashLED3(1);  // Blink LED to indicate measurement is in progress
                    VL53L1_WaitMs(dev, 1);
                }
                dataReady = 0;
                VL53L1X_GetDistance(dev, &Distance);
                FlashLED4(1);  // Blink LED when measurement is sent
                VL53L1X_ClearInterrupt(dev);

                double angle_deg = i * 45;
                double angle_rad = angle_deg * (PI / 180.0);
                double x = x_offset;
                double y = Distance * cos(angle_rad);
                double z = Distance * sin(angle_rad);

                sprintf(printf_buffer, "%d,%.2f,%.2f,%.2f,%.2f\r\n", i, angle_deg, x, y, z);
                UART_printf(printf_buffer);
            }

            // Rotate stepper motor CW
            for (int j = 0; j < STEPS_PER_45_DEGREES; j++) {
                if (motor_status) {
                    stepper_step();  // Rotate motor
                    SysTick_Wait10ms(1);
                }
            }
        }

        //UART_printf("One full rotation complete, untangling wires...\r\n");

        // Move the stepper motor BACKWARD (CCW) after one full CW rotation (360�)
        for (int k = 0; k < 8 * STEPS_PER_45_DEGREES; k++) {
            if (motor_status) {
                counterstepper();  // Rotate motor backward (CCW)
                SysTick_Wait10ms(1);
            }
        }

        x_offset += 100;  // Increment offset for the next scan
    }

    UART_printf("Motor stopped. Scan complete.\r\n");
}

int main(void) {
    PLL_Init();
    SysTick_Init();
    UART_Init();
    onboardLEDs_Init();
    I2C_Init();
    PortH_INIT();
    PortJ_Init();
    PortJ_Interrupt_Init();

    UART_printf("VL53L1X Stepper Scan Program Begins\r\n");

    uint8_t sensorState = 0;
    while (sensorState == 0) {
        status = VL53L1X_BootState(dev, &sensorState);
        SysTick_Wait10ms(10);
    }
    VL53L1X_SensorInit(dev);
    VL53L1X_StartRanging(dev);

    while (1) {
        if (data_acquisition && motor_status) {
            scan360_and_send_uart();
        }
    }

    VL53L1X_StopRanging(dev);
}
