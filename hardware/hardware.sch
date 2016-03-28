EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:arduinoprominidev-11113
EELAYER 25 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ATMEGA328-P U1
U 1 1 56F85BC8
P 2300 4950
F 0 "U1" H 1550 6200 50  0000 L BNN
F 1 "ATMEGA328-P" H 2700 3550 50  0000 L BNN
F 2 "DIL28" H 2300 4950 50  0000 C CIN
F 3 "" H 2300 4950 50  0000 C CNN
	1    2300 4950
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X04 JP5
U 1 1 56F8781B
P 9650 5150
F 0 "JP5" H 9650 5400 50  0000 C CNN
F 1 "Relay Connector" V 9750 5150 50  0000 C CNN
F 2 "" H 9650 5150 50  0000 C CNN
F 3 "" H 9650 5150 50  0000 C CNN
	1    9650 5150
	1    0    0    -1  
$EndComp
$Comp
L Crystal Y1
U 1 1 56F87BAF
P 2050 7000
F 0 "Y1" V 2200 7150 50  0000 C CNN
F 1 "16MHz" V 1950 7250 50  0000 C CNN
F 2 "" H 2050 7000 50  0000 C CNN
F 3 "" H 2050 7000 50  0000 C CNN
	1    2050 7000
	0    -1   -1   0   
$EndComp
Text Label 4150 4250 2    60   ~ 0
MISO
Text Label 4150 4150 2    60   ~ 0
MOSI
Text Label 1600 7300 0    60   ~ 0
XTAL1
Text Label 1600 6700 0    60   ~ 0
XTAL2
$Comp
L C C6
U 1 1 56F87EC5
P 2450 7300
F 0 "C6" V 2600 7250 50  0000 L CNN
F 1 "20pF" V 2300 7200 50  0000 L CNN
F 2 "" H 2488 7150 50  0000 C CNN
F 3 "" H 2450 7300 50  0000 C CNN
	1    2450 7300
	0    -1   -1   0   
$EndComp
$Comp
L C C5
U 1 1 56F87F0E
P 2450 6700
F 0 "C5" V 2600 6650 50  0000 L CNN
F 1 "20pF" V 2300 6600 50  0000 L CNN
F 2 "" H 2488 6550 50  0000 C CNN
F 3 "" H 2450 6700 50  0000 C CNN
	1    2450 6700
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR01
U 1 1 56F88212
P 3050 7150
F 0 "#PWR01" H 3050 6900 50  0001 C CNN
F 1 "GND" H 3050 7000 50  0000 C CNN
F 2 "" H 3050 7150 50  0000 C CNN
F 3 "" H 3050 7150 50  0000 C CNN
	1    3050 7150
	1    0    0    -1  
$EndComp
Text Label 4150 4450 2    60   ~ 0
XTAL1
Text Label 4150 4550 2    60   ~ 0
XTAL2
$Comp
L C C4
U 1 1 56F88844
P 1150 4700
F 0 "C4" H 1175 4800 50  0000 L CNN
F 1 "0.1uF" H 1175 4600 50  0000 L CNN
F 2 "" H 1188 4550 50  0000 C CNN
F 3 "" H 1150 4700 50  0000 C CNN
	1    1150 4700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 56F889D0
P 1150 4950
F 0 "#PWR02" H 1150 4700 50  0001 C CNN
F 1 "GND" H 1150 4800 50  0000 C CNN
F 2 "" H 1150 4950 50  0000 C CNN
F 3 "" H 1150 4950 50  0000 C CNN
	1    1150 4950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 56F88A97
P 1150 6250
F 0 "#PWR03" H 1150 6000 50  0001 C CNN
F 1 "GND" H 1150 6100 50  0000 C CNN
F 2 "" H 1150 6250 50  0000 C CNN
F 3 "" H 1150 6250 50  0000 C CNN
	1    1150 6250
	1    0    0    -1  
$EndComp
Text Label 4150 5650 2    60   ~ 0
LCD_E
Text Label 4150 5750 2    60   ~ 0
LCD_RS
Text Label 4150 6050 2    60   ~ 0
LCD_D6
Text Label 4150 6150 2    60   ~ 0
LCD_D7
Text Label 4150 5850 2    60   ~ 0
LCD_D4
Text Label 4150 5950 2    60   ~ 0
LCD_D5
Text Label 4150 4800 2    60   ~ 0
ROT_DT
Text Label 4150 4700 2    60   ~ 0
PRESS_5V
Text Label 4150 5000 2    60   ~ 0
ROT_SW
Text Label 4150 4900 2    60   ~ 0
ROT_CLK
Text Label 4150 5300 2    60   ~ 0
RST
Text Label 4150 4350 2    60   ~ 0
SCK
Text Label 4150 3950 2    60   ~ 0
BP_HIGH
Text Label 4150 3850 2    60   ~ 0
BP_LOW
$Comp
L CONN_02X03 JP7
U 1 1 56F897CA
P 5500 4800
F 0 "JP7" H 5500 5000 50  0000 C CNN
F 1 "ISP Connector" H 5500 4600 50  0000 C CNN
F 2 "" H 5500 3600 50  0000 C CNN
F 3 "" H 5500 3600 50  0000 C CNN
	1    5500 4800
	1    0    0    -1  
$EndComp
Text Label 4950 4700 0    60   ~ 0
MISO
Text Label 4950 4800 0    60   ~ 0
SCK
Text Label 4950 4900 0    60   ~ 0
RST
Text Label 6050 4800 2    60   ~ 0
MOSI
$Comp
L GND #PWR04
U 1 1 56F89872
P 5950 4900
F 0 "#PWR04" H 5950 4650 50  0001 C CNN
F 1 "GND" H 5950 4750 50  0000 C CNN
F 2 "" H 5950 4900 50  0000 C CNN
F 3 "" H 5950 4900 50  0000 C CNN
	1    5950 4900
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X08 JP4
U 1 1 56F89CC1
P 9650 4100
F 0 "JP4" H 9650 4550 50  0000 C CNN
F 1 "LCD Connector" V 9750 4100 50  0000 C CNN
F 2 "" H 9650 4100 50  0000 C CNN
F 3 "" H 9650 4100 50  0000 C CNN
	1    9650 4100
	1    0    0    -1  
$EndComp
Text Label 9100 3950 0    60   ~ 0
LCD_RS
Text Label 9100 4050 0    60   ~ 0
LCD_E
Text Label 9100 4150 0    60   ~ 0
LCD_D4
Text Label 9100 4250 0    60   ~ 0
LCD_D5
Text Label 9100 4350 0    60   ~ 0
LCD_D6
Text Label 9100 4450 0    60   ~ 0
LCD_D7
$Comp
L GND #PWR05
U 1 1 56F89D9A
P 8950 3850
F 0 "#PWR05" H 8950 3600 50  0001 C CNN
F 1 "GND" H 8950 3700 50  0000 C CNN
F 2 "" H 8950 3850 50  0000 C CNN
F 3 "" H 8950 3850 50  0000 C CNN
	1    8950 3850
	-1   0    0    -1  
$EndComp
Text Label 9000 5100 0    60   ~ 0
BP_LOW
Text Label 9000 5200 0    60   ~ 0
BP_HIGH
$Comp
L GND #PWR06
U 1 1 56F8A238
P 9150 5300
F 0 "#PWR06" H 9150 5050 50  0001 C CNN
F 1 "GND" H 9150 5150 50  0000 C CNN
F 2 "" H 9150 5300 50  0000 C CNN
F 3 "" H 9150 5300 50  0000 C CNN
	1    9150 5300
	-1   0    0    -1  
$EndComp
$Comp
L 7805 U3
U 1 1 56F8A2FA
P 1650 1400
F 0 "U3" H 1800 1204 50  0000 C CNN
F 1 "7805" H 1650 1600 50  0000 C CNN
F 2 "" H 1650 1400 50  0000 C CNN
F 3 "" H 1650 1400 50  0000 C CNN
	1    1650 1400
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR07
U 1 1 56F8A393
P 1100 1150
F 0 "#PWR07" H 1100 1000 50  0001 C CNN
F 1 "+12V" H 1100 1290 50  0000 C CNN
F 2 "" H 1100 1150 50  0000 C CNN
F 3 "" H 1100 1150 50  0000 C CNN
	1    1100 1150
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR08
U 1 1 56F8A3C5
P 2200 1150
F 0 "#PWR08" H 2200 1000 50  0001 C CNN
F 1 "+5V" H 2200 1290 50  0000 C CNN
F 2 "" H 2200 1150 50  0000 C CNN
F 3 "" H 2200 1150 50  0000 C CNN
	1    2200 1150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 56F8A438
P 1650 2050
F 0 "#PWR09" H 1650 1800 50  0001 C CNN
F 1 "GND" H 1650 1900 50  0000 C CNN
F 2 "" H 1650 2050 50  0000 C CNN
F 3 "" H 1650 2050 50  0000 C CNN
	1    1650 2050
	1    0    0    -1  
$EndComp
$Comp
L TL082 U2
U 1 1 56F8A64F
P 8650 1450
F 0 "U2" H 8600 1650 50  0000 L CNN
F 1 "TL082" H 8600 1200 50  0000 L CNN
F 2 "" H 8650 1450 50  0000 C CNN
F 3 "" H 8650 1450 50  0000 C CNN
	1    8650 1450
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR010
U 1 1 56F8A6F6
P 8550 1150
F 0 "#PWR010" H 8550 1000 50  0001 C CNN
F 1 "+12V" H 8550 1290 50  0000 C CNN
F 2 "" H 8550 1150 50  0000 C CNN
F 3 "" H 8550 1150 50  0000 C CNN
	1    8550 1150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR011
U 1 1 56F8A722
P 8550 1750
F 0 "#PWR011" H 8550 1500 50  0001 C CNN
F 1 "GND" H 8550 1600 50  0000 C CNN
F 2 "" H 8550 1750 50  0000 C CNN
F 3 "" H 8550 1750 50  0000 C CNN
	1    8550 1750
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 56F8A8D1
P 8750 2100
F 0 "R6" V 8830 2100 50  0000 C CNN
F 1 "1k" V 8750 2100 50  0000 C CNN
F 2 "" V 8680 2100 50  0000 C CNN
F 3 "" H 8750 2100 50  0000 C CNN
	1    8750 2100
	0    -1   -1   0   
$EndComp
$Comp
L R R5
U 1 1 56F8A960
P 8250 2350
F 0 "R5" V 8330 2350 50  0000 C CNN
F 1 "1k" V 8250 2350 50  0000 C CNN
F 2 "" V 8180 2350 50  0000 C CNN
F 3 "" H 8250 2350 50  0000 C CNN
	1    8250 2350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR012
U 1 1 56F8AEE1
P 8250 2550
F 0 "#PWR012" H 8250 2300 50  0001 C CNN
F 1 "GND" H 8250 2400 50  0000 C CNN
F 2 "" H 8250 2550 50  0000 C CNN
F 3 "" H 8250 2550 50  0000 C CNN
	1    8250 2550
	1    0    0    -1  
$EndComp
Text Label 10050 1450 2    60   ~ 0
VALVE_CTL_10V
$Comp
L R R1
U 1 1 56F8C15C
P 4250 1350
F 0 "R1" V 4330 1350 50  0000 C CNN
F 1 "4.7k" V 4250 1350 50  0000 C CNN
F 2 "" V 4180 1350 50  0000 C CNN
F 3 "" H 4250 1350 50  0000 C CNN
	1    4250 1350
	0    1    1    0   
$EndComp
$Comp
L C C7
U 1 1 56F8C1A7
P 4550 1650
F 0 "C7" H 4575 1750 50  0000 L CNN
F 1 "0.01uF" H 4575 1550 50  0000 L CNN
F 2 "" H 4588 1500 50  0000 C CNN
F 3 "" H 4550 1650 50  0000 C CNN
	1    4550 1650
	1    0    0    -1  
$EndComp
Text Label 3350 1350 0    60   ~ 0
VALVE_CTL_PWM
$Comp
L GND #PWR013
U 1 1 56F8C392
P 4550 1950
F 0 "#PWR013" H 4550 1700 50  0001 C CNN
F 1 "GND" H 4550 1800 50  0000 C CNN
F 2 "" H 4550 1950 50  0000 C CNN
F 3 "" H 4550 1950 50  0000 C CNN
	1    4550 1950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR014
U 1 1 56F8C729
P 5150 1950
F 0 "#PWR014" H 5150 1700 50  0001 C CNN
F 1 "GND" H 5150 1800 50  0000 C CNN
F 2 "" H 5150 1950 50  0000 C CNN
F 3 "" H 5150 1950 50  0000 C CNN
	1    5150 1950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR015
U 1 1 56F8CB43
P 5750 1950
F 0 "#PWR015" H 5750 1700 50  0001 C CNN
F 1 "GND" H 5750 1800 50  0000 C CNN
F 2 "" H 5750 1950 50  0000 C CNN
F 3 "" H 5750 1950 50  0000 C CNN
	1    5750 1950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR016
U 1 1 56F8CB59
P 6350 1950
F 0 "#PWR016" H 6350 1700 50  0001 C CNN
F 1 "GND" H 6350 1800 50  0000 C CNN
F 2 "" H 6350 1950 50  0000 C CNN
F 3 "" H 6350 1950 50  0000 C CNN
	1    6350 1950
	1    0    0    -1  
$EndComp
Text Label 7700 1350 2    60   ~ 0
VALVE_CTL_5V
Text Notes 4900 900  0    60   ~ 0
Valve Control Signal\nPWM Lowpass Filter
Text Notes 8000 850  0    60   ~ 0
Valve Control Signal Amplifier
Text Notes 7250 3200 0    60   ~ 0
Connectors
$Comp
L R R2
U 1 1 56F8FB25
P 4850 1350
F 0 "R2" V 4930 1350 50  0000 C CNN
F 1 "4.7k" V 4850 1350 50  0000 C CNN
F 2 "" V 4780 1350 50  0000 C CNN
F 3 "" H 4850 1350 50  0000 C CNN
	1    4850 1350
	0    1    1    0   
$EndComp
$Comp
L R R3
U 1 1 56F8FB7C
P 5450 1350
F 0 "R3" V 5530 1350 50  0000 C CNN
F 1 "4.7k" V 5450 1350 50  0000 C CNN
F 2 "" V 5380 1350 50  0000 C CNN
F 3 "" H 5450 1350 50  0000 C CNN
	1    5450 1350
	0    1    1    0   
$EndComp
$Comp
L R R4
U 1 1 56F8FBDE
P 6050 1350
F 0 "R4" V 6130 1350 50  0000 C CNN
F 1 "4.7k" V 6050 1350 50  0000 C CNN
F 2 "" V 5980 1350 50  0000 C CNN
F 3 "" H 6050 1350 50  0000 C CNN
	1    6050 1350
	0    1    1    0   
$EndComp
$Comp
L C C8
U 1 1 56F91B2F
P 5150 1650
F 0 "C8" H 5175 1750 50  0000 L CNN
F 1 "0.01uF" H 5175 1550 50  0000 L CNN
F 2 "" H 5188 1500 50  0000 C CNN
F 3 "" H 5150 1650 50  0000 C CNN
	1    5150 1650
	1    0    0    -1  
$EndComp
$Comp
L C C9
U 1 1 56F91BC7
P 5750 1650
F 0 "C9" H 5775 1750 50  0000 L CNN
F 1 "0.01uF" H 5775 1550 50  0000 L CNN
F 2 "" H 5788 1500 50  0000 C CNN
F 3 "" H 5750 1650 50  0000 C CNN
	1    5750 1650
	1    0    0    -1  
$EndComp
$Comp
L C C10
U 1 1 56F91C28
P 6350 1650
F 0 "C10" H 6375 1750 50  0000 L CNN
F 1 "0.01uF" H 6375 1550 50  0000 L CNN
F 2 "" H 6388 1500 50  0000 C CNN
F 3 "" H 6350 1650 50  0000 C CNN
	1    6350 1650
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 56F92384
P 2200 1650
F 0 "C2" H 2225 1750 50  0000 L CNN
F 1 "0.1uF" H 2225 1550 50  0000 L CNN
F 2 "" H 2238 1500 50  0000 C CNN
F 3 "" H 2200 1650 50  0000 C CNN
	1    2200 1650
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 56F92405
P 1100 1650
F 0 "C1" H 1125 1750 50  0000 L CNN
F 1 "0.3uF" H 1125 1550 50  0000 L CNN
F 2 "" H 1138 1500 50  0000 C CNN
F 3 "" H 1100 1650 50  0000 C CNN
	1    1100 1650
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 56F93BD7
P 850 4250
F 0 "C3" H 875 4350 50  0000 L CNN
F 1 "0.1uF" H 875 4150 50  0000 L CNN
F 2 "" H 888 4100 50  0000 C CNN
F 3 "" H 850 4250 50  0000 C CNN
	1    850  4250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR017
U 1 1 56F93D0B
P 850 4550
F 0 "#PWR017" H 850 4300 50  0001 C CNN
F 1 "GND" H 850 4400 50  0000 C CNN
F 2 "" H 850 4550 50  0000 C CNN
F 3 "" H 850 4550 50  0000 C CNN
	1    850  4550
	1    0    0    -1  
$EndComp
Text Label 4150 4050 2    60   ~ 0
VALVE_CTL_PWM
$Comp
L CONN_01X03 JP2
U 1 1 56F95FD5
P 8250 4800
F 0 "JP2" H 8250 5000 50  0000 C CNN
F 1 "Pressure Sensor" V 8350 4800 50  0000 C CNN
F 2 "" H 8250 4800 50  0000 C CNN
F 3 "" H 8250 4800 50  0000 C CNN
	1    8250 4800
	1    0    0    -1  
$EndComp
Text Label 7300 4900 0    60   ~ 0
PRESS_5V
$Comp
L GND #PWR018
U 1 1 56F96735
P 7100 4800
F 0 "#PWR018" H 7100 4550 50  0001 C CNN
F 1 "GND" H 7100 4650 50  0000 C CNN
F 2 "" H 7100 4800 50  0000 C CNN
F 3 "" H 7100 4800 50  0000 C CNN
	1    7100 4800
	-1   0    0    -1  
$EndComp
$Comp
L +12V #PWR019
U 1 1 56F96A86
P 7100 4700
F 0 "#PWR019" H 7100 4550 50  0001 C CNN
F 1 "+12V" H 7100 4840 50  0000 C CNN
F 2 "" H 7100 4700 50  0000 C CNN
F 3 "" H 7100 4700 50  0000 C CNN
	1    7100 4700
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X03 JP3
U 1 1 56F970ED
P 8250 5800
F 0 "JP3" H 8250 6000 50  0000 C CNN
F 1 "Valve Control" V 8350 5800 50  0000 C CNN
F 2 "" H 8250 5800 50  0000 C CNN
F 3 "" H 8250 5800 50  0000 C CNN
	1    8250 5800
	1    0    0    -1  
$EndComp
Text Label 7300 5900 0    60   ~ 0
VALVE_CTL_10V
$Comp
L GND #PWR020
U 1 1 56F970F7
P 7100 5800
F 0 "#PWR020" H 7100 5550 50  0001 C CNN
F 1 "GND" H 7100 5650 50  0000 C CNN
F 2 "" H 7100 5800 50  0000 C CNN
F 3 "" H 7100 5800 50  0000 C CNN
	1    7100 5800
	-1   0    0    -1  
$EndComp
$Comp
L +12V #PWR021
U 1 1 56F970FD
P 7100 5700
F 0 "#PWR021" H 7100 5550 50  0001 C CNN
F 1 "+12V" H 7100 5840 50  0000 C CNN
F 2 "" H 7100 5700 50  0000 C CNN
F 3 "" H 7100 5700 50  0000 C CNN
	1    7100 5700
	1    0    0    -1  
$EndComp
Text Notes 1300 800  0    60   ~ 0
Power Regulation
Text Notes 2000 3350 0    60   ~ 0
Microcontroller
$Comp
L CONN_01X02 JP1
U 1 1 56F99A77
P 8250 3900
F 0 "JP1" H 8250 4050 50  0000 C CNN
F 1 "Power" V 8350 3900 50  0000 C CNN
F 2 "" H 8250 3900 50  0000 C CNN
F 3 "" H 8250 3900 50  0000 C CNN
	1    8250 3900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR022
U 1 1 56F99B6E
P 7850 3950
F 0 "#PWR022" H 7850 3700 50  0001 C CNN
F 1 "GND" H 7850 3800 50  0000 C CNN
F 2 "" H 7850 3950 50  0000 C CNN
F 3 "" H 7850 3950 50  0000 C CNN
	1    7850 3950
	-1   0    0    -1  
$EndComp
$Comp
L +12V #PWR023
U 1 1 56F99B74
P 7850 3850
F 0 "#PWR023" H 7850 3700 50  0001 C CNN
F 1 "+12V" H 7850 3990 50  0000 C CNN
F 2 "" H 7850 3850 50  0000 C CNN
F 3 "" H 7850 3850 50  0000 C CNN
	1    7850 3850
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X05 JP6
U 1 1 56F9D9AE
P 9650 6100
F 0 "JP6" H 9650 6400 50  0000 C CNN
F 1 "Rotary Encoder Connector" V 9750 6100 50  0000 C CNN
F 2 "" H 9650 6100 50  0000 C CNN
F 3 "" H 9650 6100 50  0000 C CNN
	1    9650 6100
	1    0    0    -1  
$EndComp
Text Label 9000 6100 0    60   ~ 0
ROT_SW
Text Label 9000 6200 0    60   ~ 0
ROT_DT
Text Label 9000 6300 0    60   ~ 0
ROT_CLK
$Comp
L +5V #PWR024
U 1 1 56F9EDD6
P 1150 3750
F 0 "#PWR024" H 1150 3600 50  0001 C CNN
F 1 "+5V" H 1150 3890 50  0000 C CNN
F 2 "" H 1150 3750 50  0000 C CNN
F 3 "" H 1150 3750 50  0000 C CNN
	1    1150 3750
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR025
U 1 1 56F9F1D5
P 5950 4700
F 0 "#PWR025" H 5950 4550 50  0001 C CNN
F 1 "+5V" H 5950 4840 50  0000 C CNN
F 2 "" H 5950 4700 50  0000 C CNN
F 3 "" H 5950 4700 50  0000 C CNN
	1    5950 4700
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR026
U 1 1 56F9F686
P 8950 3750
F 0 "#PWR026" H 8950 3600 50  0001 C CNN
F 1 "+5V" H 8950 3890 50  0000 C CNN
F 2 "" H 8950 3750 50  0000 C CNN
F 3 "" H 8950 3750 50  0000 C CNN
	1    8950 3750
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR027
U 1 1 56F9F6E2
P 9150 5000
F 0 "#PWR027" H 9150 4850 50  0001 C CNN
F 1 "+5V" H 9150 5140 50  0000 C CNN
F 2 "" H 9150 5000 50  0000 C CNN
F 3 "" H 9150 5000 50  0000 C CNN
	1    9150 5000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR028
U 1 1 56F9F910
P 8650 6000
F 0 "#PWR028" H 8650 5750 50  0001 C CNN
F 1 "GND" H 8650 5850 50  0000 C CNN
F 2 "" H 8650 6000 50  0000 C CNN
F 3 "" H 8650 6000 50  0000 C CNN
	1    8650 6000
	-1   0    0    -1  
$EndComp
$Comp
L +5V #PWR029
U 1 1 56F9FAAB
P 8900 5800
F 0 "#PWR029" H 8900 5650 50  0001 C CNN
F 1 "+5V" H 8900 5940 50  0000 C CNN
F 2 "" H 8900 5800 50  0000 C CNN
F 3 "" H 8900 5800 50  0000 C CNN
	1    8900 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 4150 1400 4150
Wire Wire Line
	1150 3750 1150 4150
Wire Wire Line
	850  3850 1400 3850
Connection ~ 1150 3850
Wire Wire Line
	3300 4250 4150 4250
Wire Wire Line
	3300 4150 4150 4150
Wire Wire Line
	1600 7300 2300 7300
Wire Wire Line
	1600 6700 2300 6700
Connection ~ 2050 6700
Connection ~ 2050 7300
Wire Wire Line
	2600 6700 2800 6700
Wire Wire Line
	2800 6700 2800 7300
Wire Wire Line
	2800 7300 2600 7300
Wire Wire Line
	2800 7000 3050 7000
Wire Wire Line
	3050 7000 3050 7150
Connection ~ 2800 7000
Wire Wire Line
	2050 6700 2050 6850
Wire Wire Line
	2050 7300 2050 7150
Wire Wire Line
	3300 4450 4150 4450
Wire Wire Line
	3300 4550 4150 4550
Wire Wire Line
	1150 6150 1400 6150
Connection ~ 1150 6150
Wire Wire Line
	1150 6050 1400 6050
Wire Wire Line
	1150 6050 1150 6250
Wire Wire Line
	1400 4450 1150 4450
Wire Wire Line
	1150 4450 1150 4550
Wire Wire Line
	1150 4850 1150 4950
Wire Wire Line
	3300 5650 4150 5650
Wire Wire Line
	3300 5750 4150 5750
Wire Wire Line
	3300 6050 4150 6050
Wire Wire Line
	3300 6150 4150 6150
Wire Wire Line
	3300 5850 4150 5850
Wire Wire Line
	3300 5950 4150 5950
Wire Wire Line
	3300 4800 4150 4800
Wire Wire Line
	3300 4700 4150 4700
Wire Wire Line
	3300 5000 4150 5000
Wire Wire Line
	3300 4900 4150 4900
Wire Wire Line
	3300 5300 4150 5300
Wire Wire Line
	3300 4350 4150 4350
Wire Wire Line
	3300 3950 4150 3950
Wire Wire Line
	3300 3850 4150 3850
Wire Wire Line
	5250 4800 4950 4800
Wire Wire Line
	5250 4900 4950 4900
Wire Wire Line
	5250 4700 4950 4700
Wire Wire Line
	5750 4800 6050 4800
Wire Wire Line
	5750 4900 5950 4900
Wire Wire Line
	5750 4700 5950 4700
Wire Wire Line
	9450 4050 9100 4050
Wire Wire Line
	9450 4150 9100 4150
Wire Wire Line
	9450 3950 9100 3950
Wire Wire Line
	9450 4350 9100 4350
Wire Wire Line
	9450 4450 9100 4450
Wire Wire Line
	9450 4250 9100 4250
Wire Wire Line
	9450 3750 8950 3750
Wire Wire Line
	9450 3850 8950 3850
Wire Wire Line
	9450 5200 9000 5200
Wire Wire Line
	9450 5100 9000 5100
Wire Wire Line
	9450 5000 9150 5000
Wire Wire Line
	9450 5300 9150 5300
Wire Wire Line
	6200 1350 8350 1350
Wire Wire Line
	8350 1550 8250 1550
Wire Wire Line
	8950 1450 10050 1450
Wire Wire Line
	9200 1450 9200 2100
Connection ~ 9200 1450
Wire Wire Line
	8250 1550 8250 2200
Connection ~ 8250 2100
Wire Wire Line
	8250 2500 8250 2500
Wire Wire Line
	4100 1350 3350 1350
Wire Wire Line
	4400 1350 4700 1350
Wire Wire Line
	4550 1350 4550 1500
Wire Wire Line
	4550 1800 4550 1950
Wire Wire Line
	5000 1350 5300 1350
Wire Wire Line
	5150 1350 5150 1500
Connection ~ 4550 1350
Wire Wire Line
	5150 1800 5150 1950
Wire Wire Line
	8250 2500 8250 2550
Wire Wire Line
	8250 2100 8600 2100
Wire Wire Line
	9200 2100 8900 2100
Wire Wire Line
	5600 1350 5900 1350
Wire Wire Line
	5750 1350 5750 1500
Wire Wire Line
	5750 1800 5750 1950
Wire Wire Line
	6350 1350 6350 1500
Connection ~ 5750 1350
Wire Wire Line
	6350 1800 6350 1950
Connection ~ 5150 1350
Connection ~ 6350 1350
Wire Wire Line
	1100 1950 2200 1950
Connection ~ 1650 1950
Wire Wire Line
	2200 1150 2200 1500
Connection ~ 2200 1350
Wire Wire Line
	1100 1150 1100 1500
Wire Wire Line
	1250 1350 1100 1350
Connection ~ 1100 1350
Wire Wire Line
	1100 1800 1100 1950
Wire Wire Line
	1650 1650 1650 2050
Wire Wire Line
	2200 1950 2200 1800
Wire Wire Line
	850  3850 850  4100
Wire Wire Line
	850  4400 850  4550
Wire Wire Line
	3300 4050 4150 4050
Wire Wire Line
	7300 4900 8050 4900
Wire Wire Line
	7100 4700 8050 4700
Wire Wire Line
	7100 4800 8050 4800
Wire Wire Line
	7300 5900 8050 5900
Wire Wire Line
	7100 5700 8050 5700
Wire Wire Line
	7100 5800 8050 5800
Wire Wire Line
	7700 3850 8050 3850
Wire Wire Line
	7700 3950 8050 3950
Wire Wire Line
	9000 6200 9450 6200
Wire Wire Line
	9000 6300 9450 6300
Wire Wire Line
	9000 6100 9450 6100
Wire Wire Line
	8650 5900 9450 5900
Wire Wire Line
	2050 1350 2200 1350
Wire Wire Line
	8650 5900 8650 6000
Wire Wire Line
	8900 6000 9450 6000
Wire Wire Line
	8900 5800 8900 6000
$Comp
L PWR_FLAG #FLG030
U 1 1 56FA4807
P 7700 3700
F 0 "#FLG030" H 7700 3795 50  0001 C CNN
F 1 "PWR_FLAG" H 7700 3880 50  0000 C CNN
F 2 "" H 7700 3700 50  0000 C CNN
F 3 "" H 7700 3700 50  0000 C CNN
	1    7700 3700
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG031
U 1 1 56FA4962
P 7700 4100
F 0 "#FLG031" H 7700 4195 50  0001 C CNN
F 1 "PWR_FLAG" H 7700 4280 50  0000 C CNN
F 2 "" H 7700 4100 50  0000 C CNN
F 3 "" H 7700 4100 50  0000 C CNN
	1    7700 4100
	-1   0    0    1   
$EndComp
Connection ~ 7850 3850
Connection ~ 7850 3950
Wire Wire Line
	7700 3700 7700 3850
Wire Wire Line
	7700 3950 7700 4100
NoConn ~ 3300 5100
NoConn ~ 3300 5200
NoConn ~ 3300 5450
NoConn ~ 3300 5550
$EndSCHEMATC
