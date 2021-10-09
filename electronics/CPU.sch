EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 4
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
L power:GND #PWR?
U 1 1 6141E58E
P 4400 4350
F 0 "#PWR?" H 4400 4100 50  0001 C CNN
F 1 "GND" H 4405 4177 50  0000 C CNN
F 2 "" H 4400 4350 50  0001 C CNN
F 3 "" H 4400 4350 50  0001 C CNN
	1    4400 4350
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 614250EF
P 4400 1850
F 0 "#PWR?" H 4400 1700 50  0001 C CNN
F 1 "VCC" H 4415 2023 50  0000 C CNN
F 2 "" H 4400 1850 50  0001 C CNN
F 3 "" H 4400 1850 50  0001 C CNN
	1    4400 1850
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 614297CA
P 7100 4700
F 0 "#PWR?" H 7100 4550 50  0001 C CNN
F 1 "VCC" H 7115 4873 50  0000 C CNN
F 2 "" H 7100 4700 50  0001 C CNN
F 3 "" H 7100 4700 50  0001 C CNN
	1    7100 4700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6142A9AF
P 7150 2600
F 0 "#PWR?" H 7150 2350 50  0001 C CNN
F 1 "GND" H 7155 2427 50  0000 C CNN
F 2 "" H 7150 2600 50  0001 C CNN
F 3 "" H 7150 2600 50  0001 C CNN
	1    7150 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 61433C87
P 6850 4850
F 0 "C?" H 6965 4896 50  0000 L CNN
F 1 "C" H 6965 4805 50  0000 L CNN
F 2 "" H 6888 4700 50  0001 C CNN
F 3 "~" H 6850 4850 50  0001 C CNN
	1    6850 4850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 6143342D
P 4550 2050
F 0 "C?" V 4802 2050 50  0000 C CNN
F 1 "100nF" V 4711 2050 50  0000 C CNN
F 2 "" H 4588 1900 50  0001 C CNN
F 3 "~" H 4550 2050 50  0001 C CNN
	1    4550 2050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4400 1850 4400 2050
Connection ~ 4400 2050
Connection ~ 4400 2350
Wire Wire Line
	4400 2050 4400 2350
Wire Wire Line
	4400 2850 4400 2350
Connection ~ 4700 2350
$Comp
L power:GND #PWR?
U 1 1 614497D8
P 4850 2350
F 0 "#PWR?" H 4850 2100 50  0001 C CNN
F 1 "GND" H 4855 2177 50  0000 C CNN
F 2 "" H 4850 2350 50  0001 C CNN
F 3 "" H 4850 2350 50  0001 C CNN
	1    4850 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 2050 4700 2350
$Comp
L Device:CP C?
U 1 1 61434038
P 4550 2350
F 0 "C?" V 4805 2350 50  0000 C CNN
F 1 "4.7uF" V 4714 2350 50  0000 C CNN
F 2 "" H 4588 2200 50  0001 C CNN
F 3 "~" H 4550 2350 50  0001 C CNN
	1    4550 2350
	0    1    1    0   
$EndComp
Wire Wire Line
	4700 2350 4850 2350
Connection ~ 7000 4700
Wire Wire Line
	7000 4700 7100 4700
Wire Wire Line
	7000 4700 6850 4700
Wire Wire Line
	7000 4350 7000 4700
$Comp
L 2021-09-13_09-53-18:STM32G441KBT3 U?
U 1 1 6141B74B
P 4400 2850
AR Path="/6141B74B" Ref="U?"  Part="1" 
AR Path="/613FDA12/6141B74B" Ref="U?"  Part="1" 
F 0 "U?" H 5700 3237 60  0000 C CNN
F 1 "STM32G441KBT3" H 5700 3131 60  0000 C CNN
F 2 "LQFP32_7X7_STM" H 5700 3090 60  0001 C CNN
F 3 "" H 4400 2850 60  0000 C CNN
	1    4400 2850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 614726AF
P 6850 5000
F 0 "#PWR?" H 6850 4750 50  0001 C CNN
F 1 "GND" H 6855 4827 50  0000 C CNN
F 2 "" H 6850 5000 50  0001 C CNN
F 3 "" H 6850 5000 50  0001 C CNN
	1    6850 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 2850 7000 2600
Wire Wire Line
	7000 2600 7150 2600
$EndSCHEMATC
