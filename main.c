#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
 
#define BLINK_DELAY_MS 1000
 
#define DELAY 1

static const long scale = 205;

struct settings {
    int pressure;
    int hysteresis;
};

static struct settings set = { 200, 50 };

static long adc_sum = 0;
static long adc_count = 0;
ISR(ADC_vect, ISR_BLOCK)
{
    long adc = ADCL;
    adc += ADCH << 8;
    adc_sum += adc;
    ++adc_count;
}

static const char next[] = {
    /*        00  01  10  11 */
    /* 00 */   0, -1,  1,  0,
    /* 01 */   1,  0,  0, -1,
    /* 10 */  -1,  0,  0,  1,
    /* 11 */   0,  1, -1,  0,
};

static char prevState = 3;
static int rotary = 0;
static int rotaryLatch = 0;

ISR(PCINT1_vect, ISR_BLOCK)
{
    char state = (PINC >> 1) & 0x03;
    rotary += next[(prevState << 2) | state];

    if (state == 3) {
        rotaryLatch += rotary / 4;
        rotary = 0;
    }

    prevState = state;

    /* PORTB ^= _BV(PORTB5); */
}


static void _display_write(int is_data, char byte)
{
    char base = 0x00;
    if (is_data)
        base = 0x08;
    PORTD = base;
    _delay_us(DELAY);
    PORTD = base | (byte & 0xf0) | 0x04;
    _delay_us(DELAY);
    PORTD = base | (byte & 0xf0) | 0x00;
    _delay_us(DELAY);
    PORTD = base | ((byte & 0xf) << 4) | 0x04;
    _delay_us(DELAY);
    PORTD = base | ((byte & 0xf) << 4) | 0x00;
    _delay_us(DELAY);
}

#define display_write(is_data, byte, delay) \
    _display_write(is_data, byte);          \
    _delay_us(delay)

static void set_display(char *str1, char *str2)
{
    display_write(0, 0x80, 50);
    while (str1 && *str1) {
        display_write(1, *str1++, 50);
    }
    display_write(0, 0x80 + 40, 50);
    while (str2 && *str2) {
        display_write(1, *str2++, 50);
    }
}


int main (void)
{
    /* set pin 5 of PORTB for output*/
    DDRB |= _BV(DDB5);
    DDRB |= _BV(DDB0);
    DDRB |= _BV(DDB1);
 
    PORTB |= _BV(PORTB5);

    DDRD |= 0xfc;

    DDRC &= ~_BV(DDC1);
    DDRC &= ~_BV(DDC2);
    DDRC &= ~_BV(DDC3);
    PORTC |= _BV(PORTC3);

    PCICR |= _BV(PCIF1);
    PCMSK1 |= _BV(PCINT9);
    PCMSK1 |= _BV(PCINT10);
    PCMSK1 |= _BV(PCINT11);

    _delay_ms(30);

    PORTD = 0x24;
    _delay_us(DELAY);
    PORTD = 0x20;
    _delay_us(DELAY);
    display_write(0, 0x28, 500);

    display_write(0, 0x28, 500);

    display_write(0, 0x0c, 50);

    display_write(0, 0x48, 50);

    display_write(1, 0x04, 50);
    display_write(1, 0x04, 50);
    display_write(1, 0x1f, 50);
    display_write(1, 0x04, 50);
    display_write(1, 0x04, 50);
    display_write(1, 0x00, 50);
    display_write(1, 0x1f, 50);
    display_write(1, 0x00, 50);

    display_write(0, 0x01, 2000);

    /* for (i = 0; i < 40; ++i) { */
    /*     display_write(1, 0x30 + i, 50); */
    /*     _delay_us(50); */
    /* } */

               //  1234567812345678
    char buf1[] = "C:      S:200" "\1" "25";
    char buf2[] = "                ";

    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    ADMUX |= (1 << REFS0);
    ADCSRB = 0;
    ADCSRA |= (1 << ADATE);
    ADCSRA |= (1 << ADIE);
    ADCSRA |= (1 << ADEN);
    ADCSRA |= (1 << ADSC);

    const long scale = 205;

    int call = 0;

    while (1) {
        cli();
        int rot = rotaryLatch;
        rotaryLatch = 0;
        sei();
        set.pressure += rot * 5;
        if (set.pressure < 0)
            set.pressure = 0;
        if (set.pressure > 500)
            set.pressure = 500;
        
        long setP = set.pressure * scale;
        long hysteresis = set.hysteresis * scale;
        long hyst13 = hysteresis / 3;
        long lowMark = setP - hysteresis / 2;
        long highMark = setP + hysteresis / 2;

        char buf[8];
        cli();
        long adc = (adc_sum * 100) / adc_count;
        adc_sum = 0;
        adc_count = 0;
        sei();

        int newCall = call;
        if (call == 0) {
            if (adc < lowMark)
                newCall = 2;
        } else if (call == 2) {
            if (adc > highMark)
                newCall = 0;
            else if (adc > highMark - hyst13)
                newCall = 1;
        } else if (call == 1) {
            if (adc > highMark)
                newCall = 0;
            else if (adc < lowMark + hyst13)
                newCall = 2;
        }
        call = newCall;

        if (call == 0) {
            memcpy(buf2 + 7, "   Off   ", 9);
            PORTB &= ~0x03;
        } else if (call == 1) {
            memcpy(buf2 + 7, " Low Fire", 9);
            PORTB &= ~0x01;
            PORTB |= 0x02;
        } else if (call == 2) {
            memcpy(buf2 + 7, "High Fire", 9);
            PORTB |= 0x03;
        }

        ltoa(adc / scale, buf, 10);
        memcpy(buf1 + 2, "   ", 3);
        memcpy(buf1 + 2, buf, strlen(buf));
        ltoa(set.pressure, buf, 10);
        memcpy(buf1 + 10, "   ", 3);
        memcpy(buf1 + 10, buf, strlen(buf));
        /* ltoa(count , buf, 10); */
        /* memcpy(buf2, "        ", 8); */
        /* memcpy(buf2, buf, strlen(buf)); */
        /* buf2[0] = PINC & _BV(PINC3) ? '1' : '0'; */

        /* memcpy(buf2 + 2, "      ", 6); */
        /* memcpy(buf2 + 2, buf, strlen(buf)); */
        
        set_display(buf1, buf2);

        /* set pin 5 high to turn led on */
        //PORTB ^= _BV(PORTB5);
        _delay_ms(100);
        //ADCSRA |= (1 << ADSC);
    }
}
