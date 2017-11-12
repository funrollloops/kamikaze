EESchema Schematic File Version 2
LIBS:daughterboard-rescue
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
LIBS:daughterboard-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date "15 nov 2012"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CONN_13X2 P3
U 1 1 50A55ABA
P 8750 1750
F 0 "P3" H 8750 2450 60  0000 C CNN
F 1 "CONN_13X2" V 8750 1750 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x13" H 8750 1750 60  0001 C CNN
F 3 "" H 8750 1750 60  0001 C CNN
	1    8750 1750
	1    0    0    -1  
$EndComp
NoConn ~ 9150 1250
Text Label 7600 1250 0    60   ~ 0
GPIO0(SDA)
Text Label 7600 1350 0    60   ~ 0
GPIO1(SCL)
Text Label 7600 1450 0    60   ~ 0
GPIO4
NoConn ~ 8350 1550
Text Label 7600 1650 0    60   ~ 0
GPIO17
Text Label 7600 1750 0    60   ~ 0
GPIO21
Text Label 7600 1850 0    60   ~ 0
GPIO22
NoConn ~ 8350 1950
Text Label 7600 2050 0    60   ~ 0
GPIO10(MOSI)
Text Label 7600 2150 0    60   ~ 0
GPIO9(MISO)
NoConn ~ 8350 2350
Text Label 9850 1450 2    60   ~ 0
TXD
Text Label 9850 1550 2    60   ~ 0
RXD
Text Label 9850 1650 2    60   ~ 0
GPIO18
NoConn ~ 9150 1750
Text Label 9850 1850 2    60   ~ 0
GPIO23
Text Label 9850 1950 2    60   ~ 0
GPIO24
NoConn ~ 9150 2050
Text Label 9850 2150 2    60   ~ 0
GPIO25
Text Label 9850 2250 2    60   ~ 0
GPIO8(CE0)
Text Label 9850 2350 2    60   ~ 0
GPIO7(CE1)
$Comp
L ATMEGA328P-A IC1
U 1 1 59F54790
P 5100 2250
F 0 "IC1" H 4350 3500 50  0000 L BNN
F 1 "ATMEGA328P-A" V 4800 2000 50  0000 L BNN
F 2 "Housings_QFP:TQFP-32_7x7mm_Pitch0.8mm" V 4900 2300 50  0000 C CIN
F 3 "" H 5100 2250 50  0000 C CNN
	1    5100 2250
	1    0    0    -1  
$EndComp
$Comp
L Crystal Y1
U 1 1 59F55314
P 1800 6050
F 0 "Y1" H 1800 6200 50  0000 C CNN
F 1 "8MHz" V 1800 6050 50  0000 C CNN
F 2 "Crystals:HC-49V" H 1800 6050 50  0001 C CNN
F 3 "" H 1800 6050 50  0000 C CNN
	1    1800 6050
	0    1    1    0   
$EndComp
$Comp
L GNDD #PWR01
U 1 1 59F5540A
P 2050 6400
F 0 "#PWR01" H 2050 6150 50  0001 C CNN
F 1 "GNDD" H 2050 6250 50  0000 C CNN
F 2 "" H 2050 6400 50  0000 C CNN
F 3 "" H 2050 6400 50  0000 C CNN
	1    2050 6400
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR02
U 1 1 59F554A9
P 4000 1000
F 0 "#PWR02" H 4000 850 50  0001 C CNN
F 1 "+3.3V" H 4000 1140 50  0000 C CNN
F 2 "" H 4000 1000 50  0000 C CNN
F 3 "" H 4000 1000 50  0000 C CNN
	1    4000 1000
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR03
U 1 1 59F555D1
P 4050 3550
F 0 "#PWR03" H 4050 3300 50  0001 C CNN
F 1 "GNDD" H 4050 3400 50  0000 C CNN
F 2 "" H 4050 3550 50  0000 C CNN
F 3 "" H 4050 3550 50  0000 C CNN
	1    4050 3550
	1    0    0    -1  
$EndComp
$Comp
L BARREL_JACK CON1
U 1 1 59F556D0
P 6150 5800
F 0 "CON1" H 6150 6050 50  0000 C CNN
F 1 "12V_JACK" H 6150 5600 50  0000 C CNN
F 2 "Connect:BARREL_JACK" H 6150 5800 50  0001 C CNN
F 3 "" H 6150 5800 50  0000 C CNN
	1    6150 5800
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X05 P4
U 1 1 59F55796
P 10750 2850
F 0 "P4" H 10750 3150 50  0000 C CNN
F 1 "AZIMUTH" V 10850 2850 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x05" H 10750 2850 50  0001 C CNN
F 3 "" H 10750 2850 50  0000 C CNN
	1    10750 2850
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X05 P5
U 1 1 59F557FF
P 10750 4400
F 0 "P5" H 10750 4700 50  0000 C CNN
F 1 "ELEVATION" V 10850 4400 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x05" H 10750 4400 50  0001 C CNN
F 3 "" H 10750 4400 50  0000 C CNN
	1    10750 4400
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR04
U 1 1 59F558FB
P 6550 5950
F 0 "#PWR04" H 6550 5750 50  0001 C CNN
F 1 "GNDPWR" H 6550 5820 50  0000 C CNN
F 2 "" H 6550 5900 50  0000 C CNN
F 3 "" H 6550 5900 50  0000 C CNN
	1    6550 5950
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR05
U 1 1 59F559BA
P 6550 5700
F 0 "#PWR05" H 6550 5550 50  0001 C CNN
F 1 "+12V" H 6550 5840 50  0000 C CNN
F 2 "" H 6550 5700 50  0000 C CNN
F 3 "" H 6550 5700 50  0000 C CNN
	1    6550 5700
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR06
U 1 1 59F55B7E
P 10400 3100
F 0 "#PWR06" H 10400 2950 50  0001 C CNN
F 1 "+12V" H 10400 3240 50  0000 C CNN
F 2 "" H 10400 3100 50  0000 C CNN
F 3 "" H 10400 3100 50  0000 C CNN
	1    10400 3100
	-1   0    0    1   
$EndComp
$Comp
L +12V #PWR07
U 1 1 59F55BEE
P 10400 4650
F 0 "#PWR07" H 10400 4500 50  0001 C CNN
F 1 "+12V" H 10400 4790 50  0000 C CNN
F 2 "" H 10400 4650 50  0000 C CNN
F 3 "" H 10400 4650 50  0000 C CNN
	1    10400 4650
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X05 P2
U 1 1 59F56056
P 6350 2300
F 0 "P2" H 6350 2350 50  0000 C CNN
F 1 "EXTRA_PC12345" V 6450 2300 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x05" H 6350 2300 50  0001 C CNN
F 3 "" H 6350 2300 50  0000 C CNN
	1    6350 2300
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P1
U 1 1 59F56392
P 6350 1200
F 0 "P1" H 6350 1350 50  0000 C CNN
F 1 "EXTRA_PB01" V 6450 1200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 6350 1200 50  0001 C CNN
F 3 "" H 6350 1200 50  0000 C CNN
	1    6350 1200
	1    0    0    -1  
$EndComp
$Comp
L MC1417 U1
U 1 1 59F56887
P 7550 3700
F 0 "U1" H 7550 3800 50  0000 C CNN
F 1 "ULN2803A" H 7550 3600 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-18_7.5x11.6mm_Pitch1.27mm" H 7550 3700 50  0001 C CNN
F 3 "" H 7550 3700 50  0000 C CNN
	1    7550 3700
	1    0    0    -1  
$EndComp
$Comp
L Led_Small D9
U 1 1 59F57130
P 8850 3950
F 0 "D9" H 8750 3950 50  0000 L CNN
F 1 " " H 8675 3850 50  0000 L CNN
F 2 "LEDs:LED_0805" V 8850 3950 50  0001 C CNN
F 3 "" V 8850 3950 50  0000 C CNN
	1    8850 3950
	1    0    0    -1  
$EndComp
$Comp
L Led_Small D8
U 1 1 59F57803
P 8850 3850
F 0 "D8" H 8750 3850 50  0000 L CNN
F 1 " " H 8675 3750 50  0000 L CNN
F 2 "LEDs:LED_0805" V 8850 3850 50  0001 C CNN
F 3 "" V 8850 3850 50  0000 C CNN
	1    8850 3850
	1    0    0    -1  
$EndComp
$Comp
L Led_Small D7
U 1 1 59F57A02
P 8850 3750
F 0 "D7" H 8750 3750 50  0000 L CNN
F 1 " " H 8675 3650 50  0000 L CNN
F 2 "LEDs:LED_0805" V 8850 3750 50  0001 C CNN
F 3 "" V 8850 3750 50  0000 C CNN
	1    8850 3750
	1    0    0    -1  
$EndComp
$Comp
L Led_Small D6
U 1 1 59F57A1A
P 8850 3650
F 0 "D6" H 8750 3650 50  0000 L CNN
F 1 " " H 8675 3550 50  0000 L CNN
F 2 "LEDs:LED_0805" V 8850 3650 50  0001 C CNN
F 3 "" V 8850 3650 50  0000 C CNN
	1    8850 3650
	1    0    0    -1  
$EndComp
$Comp
L Led_Small D5
U 1 1 59F57BB4
P 8850 3550
F 0 "D5" H 8750 3550 50  0000 L CNN
F 1 " " H 8675 3450 50  0000 L CNN
F 2 "LEDs:LED_0805" V 8850 3550 50  0001 C CNN
F 3 "" V 8850 3550 50  0000 C CNN
	1    8850 3550
	1    0    0    -1  
$EndComp
$Comp
L Led_Small D4
U 1 1 59F57BCC
P 8850 3450
F 0 "D4" H 8750 3450 50  0000 L CNN
F 1 " " H 8675 3350 50  0000 L CNN
F 2 "LEDs:LED_0805" V 8850 3450 50  0001 C CNN
F 3 "" V 8850 3450 50  0000 C CNN
	1    8850 3450
	1    0    0    -1  
$EndComp
$Comp
L Led_Small D3
U 1 1 59F57BE4
P 8850 3350
F 0 "D3" H 8750 3350 50  0000 L CNN
F 1 " " H 8675 3250 50  0000 L CNN
F 2 "LEDs:LED_0805" V 8850 3350 50  0001 C CNN
F 3 "" V 8850 3350 50  0000 C CNN
	1    8850 3350
	1    0    0    -1  
$EndComp
$Comp
L R_Small R7
U 1 1 59F57BEA
P 9050 3850
F 0 "R7" V 9050 3800 50  0000 L CNN
F 1 "500" H 9080 3810 50  0001 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 9050 3850 50  0001 C CNN
F 3 "" H 9050 3850 50  0000 C CNN
	1    9050 3850
	0    1    1    0   
$EndComp
$Comp
L Led_Small D2
U 1 1 59F57BFC
P 8850 3250
F 0 "D2" H 8750 3250 50  0000 L CNN
F 1 " " H 8675 3150 50  0000 L CNN
F 2 "LEDs:LED_0805" V 8850 3250 50  0001 C CNN
F 3 "" V 8850 3250 50  0000 C CNN
	1    8850 3250
	1    0    0    -1  
$EndComp
$Comp
L R_Small R8
U 1 1 59F57C02
P 9050 3950
F 0 "R8" V 9050 3900 50  0000 L CNN
F 1 "500" H 9080 3910 50  0001 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 9050 3950 50  0001 C CNN
F 3 "" H 9050 3950 50  0000 C CNN
	1    9050 3950
	0    1    1    0   
$EndComp
$Comp
L +12V #PWR08
U 1 1 59F5863B
P 8200 4150
F 0 "#PWR08" H 8200 4000 50  0001 C CNN
F 1 "+12V" H 8200 4290 50  0000 C CNN
F 2 "" H 8200 4150 50  0000 C CNN
F 3 "" H 8200 4150 50  0000 C CNN
	1    8200 4150
	-1   0    0    1   
$EndComp
$Comp
L +12V #PWR09
U 1 1 59F59997
P 9150 3250
F 0 "#PWR09" H 9150 3100 50  0001 C CNN
F 1 "+12V" H 9150 3390 50  0000 C CNN
F 2 "" H 9150 3250 50  0000 C CNN
F 3 "" H 9150 3250 50  0000 C CNN
	1    9150 3250
	1    0    0    -1  
$EndComp
Connection ~ 9800 2150
Connection ~ 9800 2250
Wire Wire Line
	8250 950  8250 1150
Wire Wire Line
	8250 1150 8350 1150
Wire Wire Line
	8350 1250 7600 1250
Wire Wire Line
	8350 1350 7600 1350
Wire Wire Line
	8350 1450 7600 1450
Wire Wire Line
	8350 1650 7600 1650
Wire Wire Line
	8350 1750 7600 1750
Wire Wire Line
	8350 1850 7600 1850
Wire Wire Line
	9150 1450 9850 1450
Wire Wire Line
	9150 1550 9850 1550
Wire Wire Line
	9150 1650 9850 1650
Wire Wire Line
	9150 1850 9850 1850
Wire Wire Line
	9150 1950 9850 1950
Wire Wire Line
	9150 2150 9850 2150
Wire Wire Line
	9150 2250 9850 2250
Wire Wire Line
	9150 2350 9850 2350
Wire Wire Line
	4200 1150 4000 1150
Wire Wire Line
	4000 1000 4000 1750
Wire Wire Line
	4000 1250 4200 1250
Connection ~ 4000 1150
Wire Wire Line
	4000 1450 4200 1450
Connection ~ 4000 1250
Wire Wire Line
	4000 1750 4200 1750
Connection ~ 4000 1450
Wire Wire Line
	4200 3450 4050 3450
Wire Wire Line
	4050 3250 4050 3550
Wire Wire Line
	4200 3350 4050 3350
Connection ~ 4050 3450
Wire Wire Line
	4200 3250 4050 3250
Connection ~ 4050 3350
Wire Wire Line
	6450 5900 6550 5900
Wire Wire Line
	6550 5800 6550 5950
Wire Wire Line
	6450 5800 6650 5800
Connection ~ 6550 5900
Wire Wire Line
	6450 5700 6700 5700
Wire Wire Line
	10550 3050 10400 3050
Wire Wire Line
	10400 3050 10400 3100
Wire Wire Line
	10550 4600 10400 4600
Wire Wire Line
	10400 4600 10400 4650
Wire Wire Line
	6100 2100 6150 2100
Wire Wire Line
	6100 2200 6150 2200
Wire Wire Line
	6100 2300 6150 2300
Wire Wire Line
	6100 2400 6150 2400
Wire Wire Line
	6100 2500 6150 2500
Wire Wire Line
	6100 1150 6150 1150
Wire Wire Line
	6100 1250 6150 1250
Wire Wire Line
	6100 2750 6450 2750
Wire Wire Line
	6450 2750 6450 3250
Wire Wire Line
	6450 3250 6900 3250
Wire Wire Line
	6100 2850 6400 2850
Wire Wire Line
	6400 2850 6400 3350
Wire Wire Line
	6400 3350 6900 3350
Wire Wire Line
	6100 2950 6350 2950
Wire Wire Line
	6350 2950 6350 3450
Wire Wire Line
	6350 3450 6900 3450
Wire Wire Line
	6100 3050 6300 3050
Wire Wire Line
	6300 3050 6300 3550
Wire Wire Line
	6300 3550 6900 3550
Wire Wire Line
	6100 3150 6250 3150
Wire Wire Line
	6250 3150 6250 3650
Wire Wire Line
	6250 3650 6900 3650
Wire Wire Line
	6100 3250 6200 3250
Wire Wire Line
	6200 3250 6200 3750
Wire Wire Line
	6200 3750 6900 3750
Wire Wire Line
	6100 3350 6150 3350
Wire Wire Line
	6150 3350 6150 3850
Wire Wire Line
	6150 3850 6900 3850
Wire Wire Line
	6100 3450 6100 3950
Wire Wire Line
	6100 3950 6900 3950
Wire Wire Line
	8200 3250 8750 3250
Wire Wire Line
	8200 3350 8750 3350
Wire Wire Line
	8200 3450 8750 3450
Wire Wire Line
	8200 3550 8750 3550
Wire Wire Line
	8200 3650 8750 3650
Wire Wire Line
	8200 3750 8750 3750
Wire Wire Line
	8200 3850 8750 3850
Wire Wire Line
	8200 3950 8750 3950
Wire Wire Line
	9150 3250 9150 3950
Connection ~ 9150 3350
Connection ~ 9150 3450
Connection ~ 9150 3550
Connection ~ 9150 3850
Connection ~ 9150 3750
Connection ~ 9150 3650
Wire Wire Line
	10550 2650 9950 2650
Wire Wire Line
	9950 2650 9950 2900
Wire Wire Line
	9950 2900 8200 2900
Wire Wire Line
	8200 2900 8200 3250
Wire Wire Line
	10550 2750 10050 2750
Wire Wire Line
	10050 2750 10050 2950
Wire Wire Line
	10050 2950 8300 2950
Wire Wire Line
	8300 2950 8300 3350
Connection ~ 8300 3350
Wire Wire Line
	10550 2850 10150 2850
Wire Wire Line
	10150 2850 10150 3000
Wire Wire Line
	10150 3000 8400 3000
Wire Wire Line
	8400 3000 8400 3450
Connection ~ 8400 3450
Wire Wire Line
	10550 2950 10250 2950
Wire Wire Line
	10250 2950 10250 3050
Wire Wire Line
	10250 3050 8500 3050
Wire Wire Line
	8500 3050 8500 3550
Connection ~ 8500 3550
Wire Wire Line
	10550 4200 8700 4200
Wire Wire Line
	8700 4200 8700 3650
Connection ~ 8700 3650
Wire Wire Line
	10550 4300 8650 4300
Wire Wire Line
	8650 4300 8650 3750
Connection ~ 8650 3750
Wire Wire Line
	10550 4400 8600 4400
Wire Wire Line
	8600 4400 8600 3850
Connection ~ 8600 3850
Wire Wire Line
	10550 4500 8550 4500
Wire Wire Line
	8550 4500 8550 3950
Connection ~ 8550 3950
$Comp
L C_Small C2
U 1 1 59F5A79B
P 1950 5900
F 0 "C2" H 1960 5970 50  0000 L CNN
F 1 "22pF" H 1900 6050 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1950 5900 50  0001 C CNN
F 3 "" H 1950 5900 50  0000 C CNN
	1    1950 5900
	0    1    1    0   
$EndComp
$Comp
L C_Small C3
U 1 1 59F5A938
P 1950 6200
F 0 "C3" H 1960 6270 50  0000 L CNN
F 1 "22pF" H 1900 6350 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1950 6200 50  0001 C CNN
F 3 "" H 1950 6200 50  0000 C CNN
	1    1950 6200
	0    1    1    0   
$EndComp
Connection ~ 1800 5900
Connection ~ 1800 6200
Wire Wire Line
	2050 5900 2050 6400
Connection ~ 2050 6200
$Comp
L GNDD #PWR010
U 1 1 59F5B399
P 3800 1250
F 0 "#PWR010" H 3800 1000 50  0001 C CNN
F 1 "GNDD" H 3800 1100 50  0000 C CNN
F 2 "" H 3800 1250 50  0000 C CNN
F 3 "" H 3800 1250 50  0000 C CNN
	1    3800 1250
	1    0    0    -1  
$EndComp
$Comp
L C_Small C1
U 1 1 59F5B2F4
P 3900 1250
F 0 "C1" H 3910 1320 50  0000 L CNN
F 1 "100nF" V 4050 1200 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3900 1250 50  0001 C CNN
F 3 "" H 3900 1250 50  0000 C CNN
	1    3900 1250
	0    -1   -1   0   
$EndComp
NoConn ~ 7600 1250
NoConn ~ 7600 1350
NoConn ~ 7600 1450
NoConn ~ 7600 1650
NoConn ~ 7600 1750
NoConn ~ 7600 1850
NoConn ~ 9850 1450
NoConn ~ 9850 1550
NoConn ~ 9850 1650
NoConn ~ 9850 1850
NoConn ~ 9850 1950
$Comp
L +3.3V #PWR011
U 1 1 59F5D92E
P 8250 950
F 0 "#PWR011" H 8250 800 50  0001 C CNN
F 1 "+3.3V" H 8250 1090 50  0000 C CNN
F 2 "" H 8250 950 50  0000 C CNN
F 3 "" H 8250 950 50  0000 C CNN
	1    8250 950 
	1    0    0    -1  
$EndComp
NoConn ~ 9150 1150
NoConn ~ 9850 2350
$Comp
L GNDD #PWR012
U 1 1 59F5DCAF
P 9300 1350
F 0 "#PWR012" H 9300 1100 50  0001 C CNN
F 1 "GNDD" H 9300 1200 50  0000 C CNN
F 2 "" H 9300 1350 50  0000 C CNN
F 3 "" H 9300 1350 50  0000 C CNN
	1    9300 1350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9150 1350 9300 1350
$Comp
L PWR_FLAG #FLG013
U 1 1 59F5F5BB
P 9300 1300
F 0 "#FLG013" H 9300 1395 50  0001 C CNN
F 1 "PWR_FLAG" H 9300 1480 50  0000 C CNN
F 2 "" H 9300 1300 50  0000 C CNN
F 3 "" H 9300 1300 50  0000 C CNN
	1    9300 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9300 1350 9300 1300
$Comp
L PWR_FLAG #FLG014
U 1 1 59F5F749
P 8400 950
F 0 "#FLG014" H 8400 1045 50  0001 C CNN
F 1 "PWR_FLAG" H 8400 1130 50  0000 C CNN
F 2 "" H 8400 950 50  0000 C CNN
F 3 "" H 8400 950 50  0000 C CNN
	1    8400 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	8400 950  8400 1000
Wire Wire Line
	8400 1000 8250 1000
Connection ~ 8250 1000
$Comp
L PWR_FLAG #FLG015
U 1 1 59F5F905
P 6650 5800
F 0 "#FLG015" H 6650 5895 50  0001 C CNN
F 1 "PWR_FLAG" H 6650 5980 50  0000 C CNN
F 2 "" H 6650 5800 50  0000 C CNN
F 3 "" H 6650 5800 50  0000 C CNN
	1    6650 5800
	0    1    1    0   
$EndComp
Connection ~ 6550 5800
$Comp
L CONN_01X02 P6
U 1 1 59F5FAC3
P 3950 2550
F 0 "P6" H 3950 2700 50  0000 C CNN
F 1 "EXTRA_ADC67" V 4050 2550 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 3950 2550 50  0001 C CNN
F 3 "" H 3950 2550 50  0000 C CNN
	1    3950 2550
	-1   0    0    1   
$EndComp
Wire Wire Line
	4150 2500 4200 2500
Wire Wire Line
	4150 2600 4200 2600
$Comp
L GNDPWR #PWR016
U 1 1 59F60267
P 7050 4300
F 0 "#PWR016" H 7050 4100 50  0001 C CNN
F 1 "GNDPWR" H 7050 4170 50  0000 C CNN
F 2 "" H 7050 4250 50  0000 C CNN
F 3 "" H 7050 4250 50  0000 C CNN
	1    7050 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 4250 7050 4250
Wire Wire Line
	7050 4250 7050 4300
$Comp
L PWR_FLAG #FLG017
U 1 1 59F6042B
P 6700 5700
F 0 "#FLG017" H 6700 5795 50  0001 C CNN
F 1 "PWR_FLAG" H 6700 5880 50  0000 C CNN
F 2 "" H 6700 5700 50  0000 C CNN
F 3 "" H 6700 5700 50  0000 C CNN
	1    6700 5700
	1    0    0    -1  
$EndComp
Connection ~ 6550 5700
$Comp
L R_Small R11
U 1 1 59F68F60
P 6700 2750
F 0 "R11" H 6730 2770 50  0000 L CNN
F 1 "10K" H 6730 2710 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 6700 2750 50  0001 C CNN
F 3 "" H 6700 2750 50  0000 C CNN
	1    6700 2750
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR018
U 1 1 59F68FED
P 6700 2900
F 0 "#PWR018" H 6700 2750 50  0001 C CNN
F 1 "+3.3V" H 6700 3040 50  0000 C CNN
F 2 "" H 6700 2900 50  0000 C CNN
F 3 "" H 6700 2900 50  0000 C CNN
	1    6700 2900
	-1   0    0    1   
$EndComp
Wire Wire Line
	6700 2600 6700 2650
Wire Wire Line
	6700 2850 6700 2900
$Comp
L R R10
U 1 1 59F6CFAA
P 3800 6050
F 0 "R10" V 3880 6050 50  0000 C CNN
F 1 "1K" V 3800 6050 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3730 6050 50  0001 C CNN
F 3 "" H 3800 6050 50  0000 C CNN
	1    3800 6050
	0    1    1    0   
$EndComp
$Comp
L TIP120 Q1
U 1 1 59F6D2C6
P 4250 6050
F 0 "Q1" H 4500 6125 50  0000 L CNN
F 1 "TIP120" H 4500 6050 50  0000 L CNN
F 2 "Power_Integrations:TO-220" H 4500 5975 50  0000 L CIN
F 3 "" H 4250 6050 50  0000 L CNN
	1    4250 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 6050 4050 6050
$Comp
L CONN_01X02 P7
U 1 1 59F6D650
P 4300 5300
F 0 "P7" H 4300 5450 50  0000 C CNN
F 1 "VALVE" V 4400 5300 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 4300 5300 50  0001 C CNN
F 3 "" H 4300 5300 50  0000 C CNN
	1    4300 5300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4350 5500 4350 5850
$Comp
L +12V #PWR019
U 1 1 59F6DB2D
P 3950 5500
F 0 "#PWR019" H 3950 5350 50  0001 C CNN
F 1 "+12V" H 3950 5640 50  0000 C CNN
F 2 "" H 3950 5500 50  0000 C CNN
F 3 "" H 3950 5500 50  0000 C CNN
	1    3950 5500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3950 5500 4250 5500
$Comp
L GNDPWR #PWR020
U 1 1 59F6DC7C
P 4350 6350
F 0 "#PWR020" H 4350 6150 50  0001 C CNN
F 1 "GNDPWR" H 4350 6220 50  0000 C CNN
F 2 "" H 4350 6300 50  0000 C CNN
F 3 "" H 4350 6300 50  0000 C CNN
	1    4350 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 6250 4350 6350
$Comp
L D D1
U 1 1 59F6E1A1
P 4150 5650
F 0 "D1" H 4150 5750 50  0000 C CNN
F 1 "1N4007" H 4150 5550 50  0000 C CNN
F 2 "Diodes_ThroughHole:Diode_DO-41_SOD81_Vertical_AnodeUp" H 4150 5650 50  0001 C CNN
F 3 "" H 4150 5650 50  0000 C CNN
	1    4150 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 5650 4350 5650
Connection ~ 4350 5650
Wire Wire Line
	4000 5650 4000 5500
Connection ~ 4000 5500
Wire Wire Line
	9800 2150 10050 2150
Wire Wire Line
	9800 2250 10050 2250
Text Label 7600 2250 0    60   ~ 0
GPIO11(SCLK)
Wire Wire Line
	7450 2050 8350 2050
Wire Wire Line
	7450 2150 8350 2150
Wire Wire Line
	7450 2250 8350 2250
Connection ~ 6700 2600
Text Notes 7400 7500 0    60   ~ 0
Kamikaze Blaster Daughterboard\n
Text Label 6200 1350 0    60   ~ 0
SS
Text Label 6200 1450 0    60   ~ 0
MOSI
Text Label 6200 1550 0    60   ~ 0
MISO
Text Label 6200 1650 0    60   ~ 0
SCLK
Wire Wire Line
	6100 1350 6200 1350
Wire Wire Line
	6100 1450 6200 1450
Wire Wire Line
	6100 1550 6200 1550
Wire Wire Line
	6100 1650 6200 1650
Text Label 10050 2250 0    60   ~ 0
SS
Text Label 10050 2150 0    60   ~ 0
RESET
Text Label 7450 2050 2    60   ~ 0
MOSI
Text Label 7450 2150 2    60   ~ 0
MISO
Text Label 7450 2250 2    60   ~ 0
SCLK
Text Label 6850 2600 0    60   ~ 0
RESET
Wire Wire Line
	6100 2600 6850 2600
Text Label 1600 5900 2    60   ~ 0
XTAL1
Text Label 1600 6200 2    60   ~ 0
XTAL2
Wire Wire Line
	1600 5900 1850 5900
Wire Wire Line
	1600 6200 1850 6200
Text Label 6200 1750 0    60   ~ 0
XTAL1
Wire Wire Line
	6100 1750 6200 1750
Text Label 6200 1850 0    60   ~ 0
XTAL2
Wire Wire Line
	6100 1850 6200 1850
Text Label 6200 2000 0    60   ~ 0
VALVE
Wire Wire Line
	6100 2000 6200 2000
Text Label 3550 6050 2    60   ~ 0
VALVE
Wire Wire Line
	3550 6050 3650 6050
$Comp
L R_Small R2
U 1 1 59F57809
P 9050 3350
F 0 "R2" V 9050 3300 50  0000 L CNN
F 1 "500" V 9050 3450 50  0001 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 9050 3350 50  0001 C CNN
F 3 "" H 9050 3350 50  0000 C CNN
	1    9050 3350
	0    1    1    0   
$EndComp
$Comp
L R_Small R3
U 1 1 59F579FC
P 9050 3450
F 0 "R3" V 9050 3400 50  0000 L CNN
F 1 "500" V 9050 3550 50  0001 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 9050 3450 50  0001 C CNN
F 3 "" H 9050 3450 50  0000 C CNN
	1    9050 3450
	0    1    1    0   
$EndComp
$Comp
L R_Small R4
U 1 1 59F57A08
P 9050 3550
F 0 "R4" V 9050 3500 50  0000 L CNN
F 1 "500" H 9080 3510 50  0001 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 9050 3550 50  0001 C CNN
F 3 "" H 9050 3550 50  0000 C CNN
	1    9050 3550
	0    1    1    0   
$EndComp
$Comp
L R_Small R5
U 1 1 59F57BBA
P 9050 3650
F 0 "R5" V 9050 3600 50  0000 L CNN
F 1 "500" H 9080 3610 50  0001 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 9050 3650 50  0001 C CNN
F 3 "" H 9050 3650 50  0000 C CNN
	1    9050 3650
	0    1    1    0   
$EndComp
$Comp
L R_Small R6
U 1 1 59F57BD2
P 9050 3750
F 0 "R6" V 9050 3700 50  0000 L CNN
F 1 "500" H 9080 3710 50  0001 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 9050 3750 50  0001 C CNN
F 3 "" H 9050 3750 50  0000 C CNN
	1    9050 3750
	0    1    1    0   
$EndComp
$Comp
L R_Small R1
U 1 1 59F56BB4
P 9050 3250
F 0 "R1" V 9050 3200 50  0000 L CNN
F 1 "500" V 9050 3350 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 9050 3250 50  0001 C CNN
F 3 "" H 9050 3250 50  0000 C CNN
	1    9050 3250
	0    1    1    0   
$EndComp
$Comp
L Led_Small D11
U 1 1 59F7A6E8
P 3300 4200
F 0 "D11" H 3250 4325 50  0000 L CNN
F 1 "RED" H 3250 4100 50  0000 L CNN
F 2 "LEDs:LED_0805" V 3300 4200 50  0001 C CNN
F 3 "" V 3300 4200 50  0000 C CNN
	1    3300 4200
	0    -1   -1   0   
$EndComp
Text Label 3300 4400 0    60   ~ 0
RESET
Wire Wire Line
	3300 4300 3300 4400
$Comp
L +3.3V #PWR021
U 1 1 59F7A8FF
P 3300 3800
F 0 "#PWR021" H 3300 3650 50  0001 C CNN
F 1 "+3.3V" H 3300 3940 50  0000 C CNN
F 2 "" H 3300 3800 50  0000 C CNN
F 3 "" H 3300 3800 50  0000 C CNN
	1    3300 3800
	1    0    0    -1  
$EndComp
$Comp
L R_Small R12
U 1 1 59F7A959
P 3300 3950
F 0 "R12" H 3330 3970 50  0000 L CNN
F 1 "100" H 3330 3910 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 3300 3950 50  0001 C CNN
F 3 "" H 3300 3950 50  0000 C CNN
	1    3300 3950
	-1   0    0    1   
$EndComp
Wire Wire Line
	3300 3800 3300 3850
Wire Wire Line
	3300 4050 3300 4100
Text Label 2600 3700 0    60   ~ 0
VALVE
$Comp
L R_Small R9
U 1 1 59F7B06A
P 2600 3950
F 0 "R9" H 2630 3970 50  0000 L CNN
F 1 "100" H 2630 3910 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 2600 3950 50  0001 C CNN
F 3 "" H 2600 3950 50  0000 C CNN
	1    2600 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 3700 2600 3850
$Comp
L Led_Small D10
U 1 1 59F7B1C0
P 2600 4200
F 0 "D10" H 2550 4325 50  0000 L CNN
F 1 "YELLOW" H 2425 4100 50  0000 L CNN
F 2 "LEDs:LED_0805" V 2600 4200 50  0001 C CNN
F 3 "" V 2600 4200 50  0000 C CNN
	1    2600 4200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2600 4050 2600 4100
$Comp
L GNDD #PWR022
U 1 1 59F7B30C
P 2600 4350
F 0 "#PWR022" H 2600 4100 50  0001 C CNN
F 1 "GNDD" H 2600 4200 50  0000 C CNN
F 2 "" H 2600 4350 50  0000 C CNN
F 3 "" H 2600 4350 50  0000 C CNN
	1    2600 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 4300 2600 4350
$EndSCHEMATC