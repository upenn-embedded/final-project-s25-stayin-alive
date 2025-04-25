#include <avr/io.h>
#include <util/delay.h>
#include "uart.h"
#include <avr/interrupt.h>

#define F_CPU 16000000UL  // 16 MHz clock frequency.
#define MOTOR_PIN PB1     // OC1A output pin; used to switch the motor's ground side.
#define COMPRESSION_PIN PB0 

#define INTRO_AV PB2
#define STAYIN_ALIVE_AV PB3
#define TIKTOK_AV PB4

#define THRESHOLD_LOW 33
#define THRESHOLD_HIGH 70

volatile int tickCount = 0;
volatile int pwmEnabled = 0;

// Initialize PWM on Timer1 (mode 14: Fast PWM with ICR1 as TOP)
// This configuration provides a PWM frequency of approximately 20 kHz.
void pwm_init(void) {
    // Configure MOTOR_PIN (PB1) as an output.
    DDRB |= (1 << MOTOR_PIN);

    // Set up Timer/Counter1 for Fast PWM mode 14 (ICR1 = TOP):
    //  - WGM13=1, WGM12=1, WGM11=1, WGM10=0
    //  - Non-inverting on OC1A: COM1A1=1, COM1A0=0
    //  - Prescaler = 1 (CS10=1)
    //
    // With F_CPU = 16 MHz and TOP = 999:
    //   f_PWM = 16 MHz / (1 * (1 + 999)) ? 16 kHz
    //    ? 1 000 discrete duty steps
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13)  | (1 << WGM12)
           | (1 << CS10);    // no prescaling

    ICR1 = 1000;  // TOP = 1000 ? 1001?step resolution
}


void init_compressions_button() {

    // Set PB1 as input with internal pull-up (button pin)
    DDRB &= ~(1 << COMPRESSION_PIN);
    PORTB |= (1 << COMPRESSION_PIN);

    // Enable pin change interrupt for PORTB (PCIE0)
    PCICR |= (1 << PCIE0);
    
    // Enable pin change interrupt on PB0 (PCINT0)
    PCMSK0 |= (1 << COMPRESSION_PIN);
}

// Set the PWM duty cycle (0 to 100%). input is 0 to 100.

void set_pwm_duty_cycle(float d) { // default sets duty to 0, will have motor action.
    if (d < 0)   d = 0;
    if (d > 100) d = 100;
    
    // Scale the duty cycle percentage to the PWM compare match range.
    OCR1A = (uint16_t)(d * 10);
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
    TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0)); // why COM1A0 too?
}

ISR(PCINT0_vect) {
    static uint8_t lastButtonState = 1;

    // Read current state of PB1
    uint8_t currentState = (PINB & (1 << COMPRESSION_PIN)) >> COMPRESSION_PIN;

    // Detect falling edge (button press)
    if (lastButtonState == 1 && currentState == 0) {
        if (pwmEnabled == 1) {
            pwm_disable();
            pwmEnabled = 0;
        } else {
            pwm_enable();
            pwmEnabled = 1;
        }
    }

    lastButtonState = currentState;
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

     // Enable ADC interrupt (for relaying audio!)
     ADCSRA |= (1 << ADIE);
     
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
    
    //scale timer 3 down to 15.625kHz
    TCCR3B |= ((1 << CS30) | (1 << CS32));
    TCCR3B &= ~(1 << CS31);
    
//     set timer 3 to normal (just count up, not pwm or CTC)
    TCCR3A &= ~((1 << WGM30) | (1 << WGM31));
    TCCR3B &= ~((1 << WGM32) | (1 << WGM33));
    
    // enable timer overflow interrupt
    EIFR  |= (1 << TOV3);     // clear any pending flags
    TIMSK3 |= (1 << TOIE3); // actually enable

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
    rpmNew = 0; // no rpm (or below 15)
}

float duty = 55;
float Kp = 0.1;
int error = 0;
int bpm;

void feedback(int beats) {
    printf("rpmold%d\n", rpmOld);

    error = beats - rpmOld;
    duty += error * Kp; // Kp = 0.1
    if (duty < 33) {
        duty = 33;
    }
    if (duty > 70) {
        duty = 70;
    }
    set_pwm_duty_cycle(duty);
}


void init_av_pins() {
    
    DDRB |= (1 << INTRO_AV);
    PORTB &= ~(1 << INTRO_AV);

    DDRB |= (1 << STAYIN_ALIVE_AV);
    PORTB &= ~(1 << STAYIN_ALIVE_AV);
    
    DDRB |= (1 << TIKTOK_AV);
    PORTB &= ~(1 << TIKTOK_AV);
}


ISR(ADC_vect) {
    uint16_t value = ADC;

    if (value < THRESHOLD_LOW) {
        PORTB |= (1 << STAYIN_ALIVE_AV);  // Set PB3 HIGH
    } else {
        PORTB &= ~(1 << STAYIN_ALIVE_AV); // Set PB3 LOW
    }
    
    // if (value > THRESHOLD_HIGH) {
    //     PORTB |= (1 << TIKTOK_AV);  // Set PB4 HIGH
    // } else {
    //     PORTB &= ~(1 << TIKTOK_AV); // Set PB4 LOW
    // }
    
}




int main(void) {
    // Initialize the PWM hardware.
    
    pwm_init(); // turns on PWM w TOP = 0 but still signal.
    pwm_enable();
    init_adc();
    uart_init();
    encoder_init();

    init_compressions_button();
    init_av_pins();
    
    // set_pwm_duty_cycle(duty);
    
    while (1) {
        
        // target bpm
        bpm = 100 + ADC / 51;
//        printf("ADC%d\n", ADC);
//        printf("bpm%d\n", bpm);
//
//        printf("tickCount is %d\n", tickCount);
        if (tickCount >= 1120 && pwmEnabled == 1) {
//            printf("tick%d\n", tickCount);

            SREG &= ~(1 << 7); // disable global interrupts
            
            uint16_t t = TCNT3; // grab timer, prevent interrupt during calc before reset
            TCNT3 = 0;
            SREG |= (1 << 7); // enable interrupts
            
            rpmNew = 15625.0 * 60.0 / t; // calculate rpm using 15625 hz timer
//            printf("bpm%d\n", bpm);
            feedback(bpm);
            tickCount = 0; // reset tickCount

        }
        
        if (rpmNew != rpmOld) {
            printf("Compression RPM: %d\n", rpmNew);
            printf("ADC value: %d\n", ADC);
            rpmOld = rpmNew;
        }
        
        
     }

    pwm_disable();

    
    return 0;
}