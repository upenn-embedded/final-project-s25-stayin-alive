
#include <avr/io.h>
#include "uart.h"

#define F_CPU 16000000UL  // Assume a 16 MHz clock frequency.

#define MOTOR_PIN PB1

// Initialize PWM on Timer1 (OC1A, i.e., PB1)
void pwm_init(void) {
    // Set PB1 (OC1A) as output.
    DDRB |= (1 << PB1);

    /* Configure Timer/Counter1 for Fast PWM mode 14:
     *
     *  - WGM13:0 = 14  ==> WGM13=1, WGM12=1, WGM11=1, WGM10=0. This mode uses ICR1 as TOP.
     *  - Non-inverting mode on OC1A (COM1A1 = 1, COM1A0 = 0).
     *  - Prescaler: 8 (CS11 = 1).
     *
     * The PWM frequency is given by:
     *    f_PWM = F_CPU / (Prescaler * (1 + TOP))
     *
     * For a desired f_PWM of ~20 kHz:
     *    TOP = (F_CPU / (Prescaler * f_PWM)) - 1
     *    TOP = (16,000,000 / (8 * 20,000)) - 1 = 99.
     */
    
    TCCR1A = (1 << COM1A1) | (1 << WGM11);           // Non-inverting output & part of WGM setting.
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Fast PWM mode with ICR1 as TOP, prescaler=8.
    ICR1 = 99;  // Set TOP value for ~20 kHz PWM frequency.
}

// Set PWM duty cycle (0 to 100 percent)
void set_pwm_duty_cycle(uint8_t duty_percent) {
    if (duty_percent > 100)
        duty_percent = 100;

    // Scale duty_percent (0-100) to match the PWM TOP (ICR1).
    // For example, a 50% duty cycle when TOP is 99 gives OCR1A = 50.
    OCR1A = ((uint16_t)duty_percent * ICR1) / 100;
}

int main(void) {
    // Initialize PWM hardware.
    pwm_init();

    // Set an initial duty cycle. Adjust this to change the average voltage applied
    // to the motor. In practice, you might adjust this dynamically (e.g., based on a sensor).
    set_pwm_duty_cycle(50);  // 50% duty cycle as a starting point.

    // Main loop ? your application can adjust the PWM duty cycle based on additional logic,
    // such as sensor feedback or user input to drive the motor at around 100?120 RPM.
    while (1) {
        // For demonstration, the duty cycle is kept fixed.
        // Add your control code here if you need to adjust the speed.
    }
    
    // Although 'return 0;' is not really reached in embedded systems, it is included for compliance.
    return 0;
}
