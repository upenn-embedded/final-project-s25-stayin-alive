#include <avr/io.h>
#include <util/delay.h>
#include "uart.h"
#include <avr/interrupt.h>
 #include "ST7735.h"
 #include "LCD_GFX.h"
#include "ASCII_LUT.h"
#include <stdio.h>
#include <string.h>


#define F_CPU 16000000UL  // 16 MHz clock frequency.
#define MOTOR_PIN PD1     // OC1A output pin; used to switch the motor's ground side.
#define COMPRESSION_PIN PC1 // pin change interrupt
#define ENCODER_PIN PD2

#define STAYIN_ALIVE_AV PC2
#define TIKTOK_AV PC3

#define THRESHOLD_LOW 33
#define THRESHOLD_HIGH 70

volatile int tickCount = 0;
volatile int pwmEnabled = 0;
volatile int prev_adc_value = 0;


void pwm_init(void) {
    // Configure MOTOR_PIN (PB1) as an output.
    DDRD |= (1 << MOTOR_PIN);

    // Set up Timer/Counter1 for Fast PWM mode 14 (ICR1 = TOP):
    //  - WGM13=1, WGM12=1, WGM11=1, WGM10=0
    //  - Non-inverting on OC1A: COM1A1=1, COM1A0=0
    //  - Prescaler = 1 (CS10=1)
    //
    // With F_CPU = 16 MHz and TOP = 999:
    //   f_PWM = 16 MHz / (1 * (1 + 999)) ? 16 kHz
    //    ? 1 000 discrete duty steps
    TCCR4A = (1 << COM4A1) | (1 << WGM41);
    TCCR4B = (1 << WGM43)  | (1 << WGM42)
           | (1 << CS40);    // no prescaling

    ICR4 = 1000;  // TOP = 1000 ? 1001?step resolution
}

// Initialize PWM on Timer1 (mode 14: Fast PWM with ICR1 as TOP)
// This configuration provides a PWM frequency of approximately 20 kHz.
//void pwm_init(void) {
//    // Configure MOTOR_PIN (PD1) as an output.
//    DDRD |= (1 << MOTOR_PIN);
//
//    // Set up Timer/Counter2 for Fast PWM mode (Mode 3: Fast PWM with OCR2A as TOP):
//    //  - WGM21=1, WGM20=1 (Fast PWM mode)
//    //  - Non-inverting on OC2A: COM2A1=1, COM2A0=0
//    //  - Prescaler = 1 (CS20=1)
//    // 
//    // With F_CPU = 16 MHz and TOP = 255:
//    //   f_PWM = 16 MHz / (1 * (1 + 255)) ? 62.5 kHz (we will adjust the prescaler to get around 20 kHz)
//    
//    // Set Timer 2 in Fast PWM mode, non-inverting
////    TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); // Fast PWM, Clearing OC2B on Compare Match, set OC2B at bottom
////    TCCR2B = (1 << WGM22) | (1 << CS22) | (1 << CS21) | (1 << CS20); // Prescale 8
////    TCCR2A &= ~((1 << COM2A1) | (1 << COM2A0)); // Ensure OC2A (PB3) is not used
//    
//    // prescale by 64
//    TCCR0B |= (1 << CS01) | (1 << CS00);
//    
//    // timer0, fast PWM mode (Mode 3)
//    TCCR0A |= (1 << WGM00) | (1 << WGM01);
//    TCCR0B &= ~(1 << WGM02);
//    
//    // 
//    TCCR0A |= (1 << COM0A1) | (1 << COM0B1);
//    TCCR0A &= ~((1 << COM0A0) | (1 << COM0B0));
//        
//    OCR0A = 200;
//    OCR0B = 200 * 1/4;
//    
////    TCCR0B |= (1 << CS00);
////    
////    TCCR0A |= (1 << WGM00) | (1 << WGM01);
////    TCCR0B |= (1 << WGM01);
////            
////    OCR0A = 39;
////    OCR0B = OCR0A * 1/4;
////    
////    TCCR0A |= (1 << COM0B1);
//     
////     sei();
//    
//    
//    
////    // Set Fast PWM Mode (WGM21 and WGM20)
////    TCCR2A = (1 << COM2A1) | (1 << WGM21);  // Non-inverting PWM on OC2B (PD3)
////    TCCR2B = (1 << WGM13) | (1 << WGM22) | (1 << CS20); // Fast PWM, prescaler = 1 (no scaling)
////
////    // Set the TOP value (OCR2A) for 20 kHz PWM frequency
////    OCR2A = 255;  // With 16 MHz clock and prescaler 1, 79 gives ~20 kHz (16 MHz / (1 * (79 + 1)) = ~20 kHz)
////    
////    // Set duty cycle to 0% initially
////    OCR2B = 10; // This sets the initial duty cycle (compare match value for the output)
//}


void init_compressions_button() {

    DDRC &= ~(1 << COMPRESSION_PIN);  // Set PC1 as input
    PORTC |= (1 << COMPRESSION_PIN);  // Enable pull-up resistor
    
    // Enable pin change interrupt for PORTC (PCIE1)
    PCICR |= (1 << PCIE1);  // Enable pin change interrupt for PC1
    
    // Enable pin change interrupt on PC1 (PCINT9)
    PCMSK1 |= (1 << COMPRESSION_PIN);  // Enable interrupt for PC1
}

// Set the PWM duty cycle (0 to 100%). input is 0 to 100.

//// Set the PWM duty cycle (0 to 100%). Input is 0 to 100.
//void set_pwm_duty_cycle(float d) {
//    if (d < 0)   d = 0;
//    if (d > 100) d = 100;
//    
//    // Scale the duty cycle percentage to the PWM compare match range (OCR2A)
//    OCR2B = (uint8_t)(d * (OCR2A + 1) / 100);  // Map the duty cycle to the OCR2B range
//}
//
//// Enable PWM output on OC2A (PD1), which in turn switches the motor's ground line.
//static inline void pwm_enable(void) {
//    // Enable non-inverting output on OC2A (PD1).
//    TCCR2A |= (1 << COM2A1);  // Set COM2A1 to enable non-inverting PWM on OC2A
//    TCCR2A &= ~(1 << COM2A0); // Clear COM2A0 to ensure non-inverting mode
//}
//
//// Disable PWM output on OC2A (PD1). The motor's ground line is disconnected,
//// effectively turning the motor "off".
//static inline void pwm_disable(void) {
//    // Clear the PWM output compare settings to turn off OC2A (PD1).
//    TCCR2A &= ~((1 << COM2A1) | (1 << COM2A0)); // Clear both COM2A1 and COM2A0
//}

void set_pwm_duty_cycle(float d) { // default sets duty to 0, will have motor action.
    if (d < 0)   d = 0;
    if (d > 100) d = 100;
    
    // Scale the duty cycle percentage to the PWM compare match range.
    OCR4A = (uint16_t)(d * 10);
}

//// Enable PWM output on OC1A, which in turn switches the motor's ground line.
//// This function is used to "turn on" the motor.
static inline void pwm_enable(void) {
    // Re-enable non-inverting output on OC1A.
    TCCR4A |= (1 << COM4A1);
}

// Disable PWM output on OC1A. The motor's ground line is disconnected,
// effectively turning the motor "off".
static inline void pwm_disable(void) {
    // Clear the PWM output compare settings to turn off OC1A.
    TCCR4A &= ~((1 << COM4A1) | (1 << COM4A0)); // why COM1A0 too?
}


ISR(PCINT1_vect) {
    static uint8_t lastButtonState = 1;
    printf("button pressed\n");
    

    // Read current state of PB1
    uint8_t startState = (PINC & (1 << COMPRESSION_PIN)) >> COMPRESSION_PIN;
    _delay_ms(50);
    uint8_t currentState = (PINC & (1 << COMPRESSION_PIN)) >> COMPRESSION_PIN;
    if (startState != currentState) return;
        
    // Detect falling edge (button press)
    if (lastButtonState == 1 && currentState == 0) {
        if (pwmEnabled == 1) {
            pwm_disable();
            printf("turning PWM off\n");
            pwmEnabled = 0;
        } else {
            pwm_enable();
            printf("turning PWM on\n");
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
//     ADCSRA |= (1 << ADIE);
     
     // Start conversion
     ADCSRA |= (1 << ADSC);
     
 }

void encoder_init() {
    // input capture setup for encoder
    
    // External interrupt 0 (pin d2) fire on rising edge
    DDRD  &= ~(1 << ENCODER_PIN);    // PD2 as input
    PORTD |=  (1 << ENCODER_PIN);  // enable pull?up (if you don?t have an external pull?down)

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

float duty = 45;
float Kp = 0.3;
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
    
    DDRC |= (1 << STAYIN_ALIVE_AV);
    PORTC &= ~(1 << STAYIN_ALIVE_AV);
    
    DDRC |= (1 << TIKTOK_AV);
    PORTC &= ~(1 << TIKTOK_AV);
}


//ISR(ADC_vect) {
//    uint16_t value = ADC;
//
//    if (value < 10) {
//        PORTD |= (1 << STAYIN_ALIVE_AV);  // Set PB3 HIGH
//    } else {
//        PORTD &= ~(1 << STAYIN_ALIVE_AV); // Set PB3 LOW
//
//    }
//    
//     if (value > 1000) {
//         PORTC |= (1 << TIKTOK_AV);  // Set PB4 HIGH
//
//     } else {
//         PORTC &= ~(1 << TIKTOK_AV); // Set PB4 LOW
//     }
//    
//}

void int_to_char_array(int num, char* str) {
    // Use sprintf to convert the integer to a string (char array)
    sprintf(str, "%d", num);
}


int main(void) {
    // Initialize the PWM hardware.
//    PRR0 |= (1 << PRUSART0);
    
    lcd_init();
    
    char bpm_string[20];  // Make sure the char array is large enough to hold the string representation
    pwm_init(); // turns on PWM w TOP = 0 but still signal.
//    pwm_enable();
    init_adc();
//    uart_init();
    encoder_init();

    init_compressions_button();
    init_av_pins();
    
    set_pwm_duty_cycle(duty);
    pwm_disable();
//    pwm_enable();
    
    SREG &= ~(1 << 7); // disable global interrupts
    LCD_setScreen(BLACK);        
    SREG |= (1 << 7); // enable interrupts
    
    PORTC &= ~(1 << TIKTOK_AV); // Set PB4 LOW
    

   while (1) {

        // target bpm
       bpm = 100 + ADC / 51;
       
       
       if (tickCount >= 1120) {

           SREG &= ~(1 << 7); // disable global interrupts
           uint16_t t = TCNT3; // grab timer, prevent interrupt during calc before reset
           
           // printing here
           int_to_char_array(bpm, bpm_string);
           strcat(bpm_string, " CPM");
           LCD_drawString(34, 60, bpm_string, WHITE, 0);

           rpmNew = 15625.0 * 60.0 / t; // calculate rpm using 15625 hz timer
           feedback(bpm);
           
           uint16_t value = ADC;
           
            if (value < 10 && prev_adc_value > 10) {
                PORTC |= (1 << STAYIN_ALIVE_AV);  // Set HIGH

            } else if (value > 10) {
                PORTC &= ~(1 << STAYIN_ALIVE_AV); // Set LOW
            }
           
           
           if (value > 800 && prev_adc_value < 800) {
                PORTC |= (1 << TIKTOK_AV);  // Set HIGH

            } else if (value < 800) {
                PORTC &= ~(1 << TIKTOK_AV); // Set LOW
            }
           
           prev_adc_value = value;
           
           tickCount = 0; // reset tickCount
           TCNT3 = 0;
           SREG |= (1 << 7); // enable interrupts


       }
       
       if (rpmNew != rpmOld) {
           
            rpmOld = rpmNew;

       }
       
//       printf("ADC value: %d\n", ADC);
       
       // Convert the integer to a string
       

       
       for (int i = 0; i < 20; i++) {
           bpm_string[i] = '\0';  // Null-terminate each character
       }
       
   }

    pwm_disable();

    
    return 0;
}