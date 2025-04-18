#include <avr/io.h>
#include <util/delay.h>
#include "uart.h"
#include <avr/interrupt.h>

#define F_CPU 16000000UL  // 16 MHz clock frequency.
#define MOTOR_PIN PB1     // OC1A output pin; used to switch the motor's ground side.


#define MAX_DUTY 50
#define MIN_DUTY 40

#define ENCODER PB0

volatile int tickCount = 0;

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

void set_pwm_duty_cycle(uint8_t duty_percent) { // default sets duty to 0, will have motor action.
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

void encoder_init() {
    // input capture setup for encoder
    
    // External interrupt 0 (pin d2) fire on rising edge
    DDRD  &= ~(1 << DDD2);    // PD2 as input
    PORTD |=  (1 << PORTD2);  // enable pull?up (if you don?t have an external pull?down)

    EICRA |= ((1 << ISC00) | (1 << ISC01)); // rising edge
    EIMSK |= (1 << INT0); // enable int0
    
    //scale timer 1 down to 15.625kHz
    TCCR3B |= ((1 << CS30) | (1 << CS32));
    TCCR3B &= ~(1 << CS31);
    
//     set timer 1 to normal (just count up, not pwm or CTC)
    TCCR3A &= ~((1 << WGM30) | (1 << WGM31));
    TCCR3B &= ~((1 << WGM32) | (1 << WGM33));
    
    // set timer to 0
    TCNT3 = 0;
    
    sei();
}

ISR(INT0_vect) {
    tickCount++; // more encoder tick counts
}

volatile int rpmNew = 0;
volatile int rpmOld = 0;

ISR(TIMER3_OVF_vect) {
    rpmNew == 0; // no rpm (or below 15)
}

void stop() {

//    if (tickCount > 560) {
//        set_pwm_duty_cycle(0);
//        while(tickCount < 1170 && (ADC < 900 && ADC > 100)) {
//        }
//        while (tickCount < 2205 && (ADC < 900 && ADC > 100)) {
//        }
        pwm_disable();
//        _delay_ms(1000);
//    }
}

int main(void) {
    // Initialize the PWM hardware.
    
    pwm_init(); // turns on PWM w TOP = 0 but still signal.
    pwm_disable();
    init_adc();
    uart_init();
    encoder_init();

    // Pre-set the PWM duty cycle to 50% for the "on" period.
//    set_pwm_duty_cycle(50, T > 4 sec);
    
    while (1) {
        
        if (tickCount >= 1120) {
            tickCount = 0 + tickCount - 1120;
            rpmNew = 15625.0 * 60.0 / TCNT3;
            TCNT3 = 0;
        }
        
        if (rpmNew != rpmOld) {
            printf("Compression RPM: %d\n", rpmNew);
            printf("ADC value: %d\n", ADC);
            rpmOld = rpmNew;
        }
        
        

//        set_pwm_duty_cycle(ADC / 20);

//        MODE: actual // howard eats ass 
        
        
        // every day 
        
        if (ADC > 1016) { // paddle up
            pwm_enable();
            set_pwm_duty_cycle(55);
        } else if (ADC < 20) {
            pwm_enable();
            set_pwm_duty_cycle(41); 
        } else {
            stop();
        }
        
        //MODE: manual tuning
//        if (ADC > 1016) { // paddle up
//            pwm_enable();
//            set_pwm_duty_cycle(0);
//        } else if (ADC < 20) {
//            pwm_enable();
//            set_pwm_duty_cycle(0); // motor spins, but! Controllable!
//        } else {
//            pwm_disable();
//        }
        
        
     }

    // pwm_disable();

    
    // return 0;
}
