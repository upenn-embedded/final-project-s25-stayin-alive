#include <avr/io.h>
#include <util/delay.h>
#include "uart.h"

#define F_CPU 16000000UL  // 16 MHz clock frequency.
#define MOTOR_PIN PB1     // OC1A output pin; used to switch the motor's ground side.


#define MAX_DUTY 50
#define MIN_DUTY 40

#define FIRST_INSTR PB0

// Initialize PWM on Timer1 (mode 14: Fast PWM with ICR1 as TOP)
// This configuration provides a PWM frequency of approximately 20 kHz.
void pwm_init(void) {
    // Configure MOTOR_PIN (PB1) as an output.
    DDRB |= (1 << MOTOR_PIN);

    // Set up Timer/Counter1:
    // - Fast PWM mode 14: WGM13=1, WGM12=1, WGM11=1, WGM10=0.
    // - Non-inverting mode on OC1A: COM1A1=1, COM1A0=0.
    // - Prescaler set to 8 (CS11=1) so that:
    //    PWM frequency f_PWM = F_CPU / (8 * (1 + TOP)).
    // With TOP = 99 we get ~20 kHz.
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    ICR1 = 99;  // Set TOP value for ~20 kHz PWM frequency.
}

// Set the PWM duty cycle (0 to 100%).
void set_pwm_duty_cycle(uint8_t duty_percent) {
    if (duty_percent > 100)
        duty_percent = 100;
    
    // Scale the duty cycle percentage to the PWM range.
    OCR1A = ((uint16_t)duty_percent * ICR1) / 100;
}

//// Enable PWM output on OC1A, which in turn switches the motor's ground line.
//// This function is used to "turn on" the motor.
static inline void pwm_enable(void) {
    // Re-enable non-inverting output on OC1A.
    TCCR1A |= (1 << COM1A1);
}

// Disable PWM output on OC1A. The motor's ground line is disconnected,
// effectively turning the motor "off".
static inline void pwm_disable(void) {
    // Clear the PWM output compare settings to turn off OC1A.
    TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0));
}

void init_adc() {
     
     // ADC STUFF (thumb stick)
     ADMUX |= (1 << REFS0);
     ADMUX &= ~(1 << REFS1);
     
     // Set ADC click div by 128 (16M/128 = 125kHz)
     ADCSRA |= (1 << ADPS0);
     ADCSRA |= (1 << ADPS1);
     ADCSRA |= (1 << ADPS2);
     
     // Select Channel ADC0 (pin C0)
     ADMUX &= ~(1 << MUX0);
     ADMUX &= ~(1 << MUX1);
     ADMUX &= ~(1 << MUX2);
     ADMUX &= ~(1 << MUX3);
     
     // Trigger ADC
     ADCSRA |= (1 << ADATE);
     
     // free funning mode (ADTS[2:0] = 000
     ADCSRB &= ~(1 << ADTS0);
     ADCSRB &= ~(1 << ADTS1);
     ADCSRB &= ~(1 << ADTS2);
 
     // Disable digital input buffer on ADC pin
     DIDR0 |= (1 << ADC0D);
     
     // Enable ADC
     ADCSRA |= (1 << ADEN);
     
     // Start conversion
     ADCSRA |= (1 << ADSC);
     
 }




int main(void) {
    // Initialize the PWM hardware.
    pwm_init();
    init_adc();
    uart_init();
   
    

    // Pre-set the PWM duty cycle to 50% for the "on" period.
    set_pwm_duty_cycle(50);
    pwm_enable();
    while (1) {
        
        printf("ADC value: %d\n", ADC);
        
        set_pwm_duty_cycle(ADC / 20);
       
        if (ADC > 526) { // paddle up
            set_pwm_duty_cycle(MAX_DUTY);
        } else if (ADC < 200) {
            set_pwm_duty_cycle(10);
        } else {
            set_pwm_duty_cycle(0);
        }
    }

    pwm_disable();

    
    return 0;
}
