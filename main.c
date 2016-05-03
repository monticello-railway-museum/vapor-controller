#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
 
#define BAUD 57600

#include <util/setbaud.h>

void uart_init(void) {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~(_BV(U2X0));
#endif

    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */
}

#include <stdio.h>

int uart_putchar(char c, FILE *stream) {
    if (c == '\n')
        uart_putchar('\r', stream);
    loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
    UDR0 = c;
    return 0;
}

int uart_getchar(void) {
    loop_until_bit_is_set(UCSR0A, RXC0); /* Wait until data exists. */
    return UDR0;
}

static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL,
                                         _FDEV_SETUP_WRITE);

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

#define Kp 0.01
#define Ki 0.002
#define Kd 0.1

#define LOW 0.40
#define HIGH 1.00

#define NUM_DIFFS 16

#define VALVE_SPEED 3.5

float
process(register float val, int reset) {
   static float buf[2];
   register float tmp, fir, iir;
   if (reset)
       memset(buf, 0, sizeof(buf));
   tmp= buf[0]; memmove(buf, buf+1, 1*sizeof(float));
   // use 0.06745527388907191 below for unity gain at 100% level
   iir= val * 0.0674552738890719;
   iir -= 0.4128015980961885*tmp; fir= tmp;
   iir -= -1.142980502539901*buf[0]; fir += buf[0]+buf[0];
   fir += iir;
   buf[1]= iir; val= fir;
   return val;
}

static float actValvePos;

static float integral;
static float lastAdj;
static float lastDiff;
static float diffHistory[NUM_DIFFS];
static float diffSum;
static uint8_t diffIndex;

float pid(float setPress, float press, float maxPress, int reset)
{
    float diff = setPress - press;
    if (reset) {
        integral = 0;
        memset(diffHistory, 0, sizeof(diffHistory));
        lastDiff = diff;
        diffSum = 0;
    }

    float p = diff * Kp;
    if (lastAdj > LOW - 0.1 && lastAdj < HIGH + 0.1)
        integral += diff * Ki;
    else
        integral *= 0.95;

    float diffDiff = diff - lastDiff;

    /* diffSum -= diffHistory[diffIndex % NUM_DIFFS]; */
    /* diffSum += diffHistory[diffIndex++ % NUM_DIFFS] = diffDiff; */
    float smoothDiff = process(diffDiff, reset);
    float d = smoothDiff * Kd;

    float adj = LOW + p + integral + d;

    lastAdj = adj;

    if (adj > HIGH)
        adj = HIGH;
    if (adj < LOW)
        adj = LOW;

    float ticksToMax = 999;
    if (smoothDiff < 0)
        ticksToMax = (maxPress - press) / -smoothDiff;
    if (ticksToMax > 999)
        ticksToMax = 999;
    float valveCloseTime = actValvePos * 10 * VALVE_SPEED;

    printf("pres: %+5d adj: %+5d p: %+5d int: %+5d d: %+5d tts: %+5d vct: %+5d\n",
           (int)(press * 100),
           (int)(adj * 100),
           (int)(p * 100),
           (int)(integral * 100),
           (int)(d * 100),
           (int)(ticksToMax + 0.5),
           (int)(valveCloseTime + 0.5));

    if (valveCloseTime > ticksToMax)
        adj = 0;

    lastDiff = diff;

    return adj;
}

int main (void)
{
    uart_init();
    stdout = &mystdout;
    printf("hi world!\n");

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
        if (adc_count < 100) {
            sei();
            goto wait;
        }
        long adc = (adc_sum * 100) / adc_count;
        adc_sum = 0;
        adc_count = 0;
        sei();

        int newCall = call;
        int resetPid = 0;
        if (call == 0) {
            if (adc < lowMark)
                newCall = 2;
            resetPid = 1;
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

        float adj = 0;
        char fbuf[10];
        if (call == 1 || call == 2) {
            adj = pid(set.pressure, (float)adc / scale, (float)highMark / scale, resetPid);
            snprintf(fbuf, sizeof(fbuf), "%+9d", (int)(adj * 100));
            memcpy(buf2 + 7, fbuf, 9);
        }

        if (adj > actValvePos) {
            actValvePos += 1 / VALVE_SPEED / 10;
            if (actValvePos > adj)
                actValvePos = adj;
        } else if (adj < actValvePos) {
            actValvePos -= 1 / VALVE_SPEED / 10;
            if (actValvePos < adj)
                actValvePos = adj;
        }

        snprintf(fbuf, sizeof(fbuf), "%+6d", (int)(actValvePos * 100));
        memcpy(buf2 + 0, fbuf, 6);

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

    wait:
        set_display(buf1, buf2);

        /* set pin 5 high to turn led on */
        //PORTB ^= _BV(PORTB5);
        _delay_ms(100);
        //ADCSRA |= (1 << ADSC);
    }
}
