EESchema Schematic File Version 2
LIBS:agg-kicad
LIBS:linetime-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 6
Title "Linetime"
Date "2016-12-08"
Rev "1"
Comp ""
Comment1 "Drawn By: Adam Greig"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MAX-M8Q IC?
U 1 1 584A2C4A
P 6000 3500
F 0 "IC?" H 5500 4100 50  0000 L CNN
F 1 "MAX-M8Q" H 5500 2900 50  0000 L CNN
F 2 "agg:MAX-M8Q" H 5500 2800 50  0001 L CNN
F 3 "https://www.u-blox.com/sites/default/files/MAX-M8-FW3_DataSheet_%28UBX-15031506%29.pdf" H 5500 2700 50  0001 L CNN
	1    6000 3500
	1    0    0    -1  
$EndComp
$Comp
L COAX P?
U 1 1 584A2CBA
P 7250 4000
F 0 "P?" H 7250 4100 50  0000 C CNN
F 1 "COAX" H 7250 3850 50  0000 C CNN
F 2 "agg:SMA-PTH" H 7250 3790 50  0001 C CNN
F 3 "http://www.farnell.com/cad/1810981.pdf" H 7350 3900 50  0001 C CNN
F 4 "2112448" H 7250 3720 50  0001 C CNN "Farnell"
	1    7250 4000
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 584A2F93
P 6700 3800
F 0 "R?" H 6750 3850 50  0000 C CNN
F 1 "10" H 6750 3750 50  0000 C CNN
F 2 "agg:0603" H 6700 3800 50  0001 C CNN
F 3 "" H 6700 3800 50  0001 C CNN
	1    6700 3800
	1    0    0    -1  
$EndComp
$Comp
L L L?
U 1 1 584A2FCB
P 6950 3850
F 0 "L?" H 7000 3900 50  0000 C CNN
F 1 "27n" H 7000 3800 50  0000 C CNN
F 2 "" H 6950 3850 50  0001 C CNN
F 3 "" H 6950 3850 50  0001 C CNN
	1    6950 3850
	0    1    1    0   
$EndComp
Wire Wire Line
	6600 3800 6700 3800
Wire Wire Line
	6800 3800 6950 3800
Wire Wire Line
	6950 3700 6950 3850
Wire Wire Line
	6600 4000 7150 4000
Wire Wire Line
	6950 4000 6950 3950
Connection ~ 6950 4000
$Comp
L GND #PWR?
U 1 1 584A307E
P 7100 4150
F 0 "#PWR?" H 6970 4190 50  0001 L CNN
F 1 "GND" H 7100 4050 50  0000 C CNN
F 2 "" H 7100 4150 60  0001 C CNN
F 3 "" H 7100 4150 60  0001 C CNN
	1    7100 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 4100 7100 4100
Wire Wire Line
	7100 4100 7100 4150
$Comp
L C C?
U 1 1 584A30AE
P 7000 3700
F 0 "C?" H 7050 3770 50  0000 C CNN
F 1 "10n" H 7050 3630 50  0000 C CNN
F 2 "agg:0603" H 7000 3700 50  0001 C CNN
F 3 "" H 7000 3700 50  0001 C CNN
	1    7000 3700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 584A30E1
P 7150 3700
F 0 "#PWR?" H 7020 3740 50  0001 L CNN
F 1 "GND" H 7150 3600 50  0000 C CNN
F 2 "" H 7150 3700 60  0001 C CNN
F 3 "" H 7150 3700 60  0001 C CNN
	1    7150 3700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7150 3700 7100 3700
Wire Wire Line
	7000 3700 6950 3700
Connection ~ 6950 3800
NoConn ~ 6600 3600
NoConn ~ 6600 3700
NoConn ~ 6600 3000
NoConn ~ 6600 3100
Text HLabel 6650 3300 2    60   Input ~ 0
GPS_RXD
Text HLabel 6650 3400 2    60   Output ~ 0
GPS_TXD
Wire Wire Line
	6650 3400 6600 3400
Wire Wire Line
	6600 3300 6650 3300
$Comp
L 3v3 #PWR?
U 1 1 584A34A9
P 5350 2950
F 0 "#PWR?" H 5350 3060 50  0001 L CNN
F 1 "3v3" H 5350 3040 50  0000 C CNN
F 2 "" H 5350 2950 60  0001 C CNN
F 3 "" H 5350 2950 60  0001 C CNN
	1    5350 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 2950 5350 3200
Wire Wire Line
	5350 3000 5400 3000
Wire Wire Line
	5350 3100 5400 3100
Connection ~ 5350 3000
Wire Wire Line
	5350 3200 5400 3200
Connection ~ 5350 3100
$Comp
L C C?
U 1 1 584A353B
P 5150 3050
F 0 "C?" H 5200 3120 50  0000 C CNN
F 1 "100n" H 5200 2980 50  0000 C CNN
F 2 "agg:0603" H 5150 3050 50  0001 C CNN
F 3 "" H 5150 3050 50  0001 C CNN
	1    5150 3050
	0    1    1    0   
$EndComp
$Comp
L 3v3 #PWR?
U 1 1 584A35BC
P 5150 3000
F 0 "#PWR?" H 5150 3110 50  0001 L CNN
F 1 "3v3" H 5150 3090 50  0000 C CNN
F 2 "" H 5150 3000 60  0001 C CNN
F 3 "" H 5150 3000 60  0001 C CNN
	1    5150 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 3000 5150 3050
$Comp
L GND #PWR?
U 1 1 584A35F4
P 5300 3550
F 0 "#PWR?" H 5170 3590 50  0001 L CNN
F 1 "GND" H 5300 3450 50  0000 C CNN
F 2 "" H 5300 3550 60  0001 C CNN
F 3 "" H 5300 3550 60  0001 C CNN
	1    5300 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 3500 5400 3500
Wire Wire Line
	5300 3300 5300 3550
Wire Wire Line
	5400 3300 5300 3300
Connection ~ 5300 3500
Wire Wire Line
	5400 3400 5300 3400
Connection ~ 5300 3400
$Comp
L GND #PWR?
U 1 1 584A366D
P 5150 3200
F 0 "#PWR?" H 5020 3240 50  0001 L CNN
F 1 "GND" H 5150 3100 50  0000 C CNN
F 2 "" H 5150 3200 60  0001 C CNN
F 3 "" H 5150 3200 60  0001 C CNN
	1    5150 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 3200 5150 3150
Text HLabel 5200 3700 0    60   Input ~ 0
GPS_~RESET
Wire Wire Line
	5200 3700 5400 3700
Text HLabel 5200 3800 0    60   Output ~ 0
GPS_PPS
Wire Wire Line
	5200 3800 5400 3800
NoConn ~ 5400 3900
Wire Wire Line
	4850 4000 5400 4000
$Comp
L CS2100-CP IC?
U 1 1 584A3257
P 6000 5100
F 0 "IC?" H 5400 5500 50  0000 L CNN
F 1 "CS2100-CP" H 5400 4700 50  0000 L CNN
F 2 "agg:MSOP-10" H 5400 4600 50  0001 L CNN
F 3 "https://www.cirrus.com/en/pubs/proDatasheet/CS2100-CP_F3.pdf" H 5400 4500 50  0001 L CNN
F 4 "777-CS2100CP-CZZ" H 5400 4400 50  0001 L CNN "Mouser"
	1    6000 5100
	1    0    0    -1  
$EndComp
$Comp
L TCXO Y?
U 1 1 584A32B2
P 3000 5300
F 0 "Y?" H 2800 5400 50  0000 L CNN
F 1 "26M TCXO" H 2800 5100 50  0000 L CNN
F 2 "agg:TG-5006CG" H 2800 5300 50  0001 C CNN
F 3 "" H 2800 5300 50  0001 C CNN
F 4 "2405785" H 2800 5000 50  0001 L CNN "Farnell"
	1    3000 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 4000 4850 5100
Wire Wire Line
	4850 5100 5300 5100
$Comp
L 3v3 #PWR?
U 1 1 584A3402
P 5250 4750
F 0 "#PWR?" H 5250 4860 50  0001 L CNN
F 1 "3v3" H 5250 4840 50  0000 C CNN
F 2 "" H 5250 4750 60  0001 C CNN
F 3 "" H 5250 4750 60  0001 C CNN
	1    5250 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 4750 5250 4800
Wire Wire Line
	5250 4800 5300 4800
$Comp
L GND #PWR?
U 1 1 584A3457
P 5250 4900
F 0 "#PWR?" H 5120 4940 50  0001 L CNN
F 1 "GND" H 5250 4800 50  0000 C CNN
F 2 "" H 5250 4900 60  0001 C CNN
F 3 "" H 5250 4900 60  0001 C CNN
	1    5250 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 4900 5250 4900
$Comp
L C C?
U 1 1 584A34F0
P 5050 4700
F 0 "C?" H 5100 4770 50  0000 C CNN
F 1 "100n" H 5100 4630 50  0000 C CNN
F 2 "agg:0603" H 5050 4700 50  0001 C CNN
F 3 "" H 5050 4700 50  0001 C CNN
	1    5050 4700
	0    1    1    0   
$EndComp
$Comp
L 3v3 #PWR?
U 1 1 584A34F6
P 5050 4650
F 0 "#PWR?" H 5050 4760 50  0001 L CNN
F 1 "3v3" H 5050 4740 50  0000 C CNN
F 2 "" H 5050 4650 60  0001 C CNN
F 3 "" H 5050 4650 60  0001 C CNN
	1    5050 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 4650 5050 4700
$Comp
L GND #PWR?
U 1 1 584A34FD
P 5050 4850
F 0 "#PWR?" H 4920 4890 50  0001 L CNN
F 1 "GND" H 5050 4750 50  0000 C CNN
F 2 "" H 5050 4850 60  0001 C CNN
F 3 "" H 5050 4850 60  0001 C CNN
	1    5050 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 4850 5050 4800
Text HLabel 6800 4800 2    60   Output ~ 0
CS2100_CLK
Wire Wire Line
	6700 4800 6800 4800
NoConn ~ 6700 4900
$Comp
L GND #PWR?
U 1 1 584A35BA
P 6750 5350
F 0 "#PWR?" H 6620 5390 50  0001 L CNN
F 1 "GND" H 6750 5250 50  0000 C CNN
F 2 "" H 6750 5350 60  0001 C CNN
F 3 "" H 6750 5350 60  0001 C CNN
	1    6750 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 5350 6750 5300
Wire Wire Line
	6750 5300 6700 5300
Text HLabel 6800 5100 2    60   BiDi ~ 0
CS2100_SDA
Text HLabel 6800 5200 2    60   Output ~ 0
CS2100_SCL
Wire Wire Line
	6800 5100 6700 5100
Wire Wire Line
	6700 5200 6800 5200
NoConn ~ 5300 5300
$Comp
L GND #PWR?
U 1 1 584A37D0
P 2650 5450
F 0 "#PWR?" H 2520 5490 50  0001 L CNN
F 1 "GND" H 2650 5350 50  0000 C CNN
F 2 "" H 2650 5450 60  0001 C CNN
F 3 "" H 2650 5450 60  0001 C CNN
	1    2650 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 5450 2650 5400
Wire Wire Line
	2650 5400 2700 5400
Wire Wire Line
	3300 5300 3450 5300
$Comp
L C C?
U 1 1 584A3BFC
P 3450 5300
F 0 "C?" H 3500 5370 50  0000 C CNN
F 1 "1n" H 3500 5230 50  0000 C CNN
F 2 "agg:0603" H 3450 5300 50  0001 C CNN
F 3 "" H 3450 5300 50  0001 C CNN
	1    3450 5300
	1    0    0    -1  
$EndComp
$Comp
L ADP3335 IC?
U 1 1 584AEF0B
P 1450 5400
F 0 "IC?" H 1250 5600 50  0000 L CNN
F 1 "ADP3335" H 1250 5100 50  0000 L CNN
F 2 "agg:MSOP-8" H 1250 5000 50  0001 L CNN
F 3 "" H 1850 5000 50  0001 C CNN
F 4 "2376932" H 1250 4900 50  0001 L CNN "Farnell"
	1    1450 5400
	1    0    0    -1  
$EndComp
$Comp
L 3v3 #PWR?
U 1 1 584AF153
P 1100 5250
F 0 "#PWR?" H 1100 5360 50  0001 L CNN
F 1 "3v3" H 1100 5340 50  0000 C CNN
F 2 "" H 1100 5250 60  0001 C CNN
F 3 "" H 1100 5250 60  0001 C CNN
	1    1100 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1100 5250 1100 5500
Wire Wire Line
	1100 5300 1150 5300
Wire Wire Line
	1100 5400 1150 5400
Connection ~ 1100 5300
Wire Wire Line
	1100 5500 1150 5500
Connection ~ 1100 5400
$Comp
L GND #PWR?
U 1 1 584AF27D
P 1100 5650
F 0 "#PWR?" H 970 5690 50  0001 L CNN
F 1 "GND" H 1100 5550 50  0000 C CNN
F 2 "" H 1100 5650 60  0001 C CNN
F 3 "" H 1100 5650 60  0001 C CNN
	1    1100 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 5600 1100 5600
Wire Wire Line
	1100 5600 1100 5650
$Comp
L C C?
U 1 1 584AF309
P 1850 5600
F 0 "C?" H 1900 5670 50  0000 C CNN
F 1 "10n" H 1900 5530 50  0000 C CNN
F 2 "agg:0603" H 1850 5600 50  0001 C CNN
F 3 "" H 1850 5600 50  0001 C CNN
	1    1850 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 5600 1850 5600
Wire Wire Line
	1750 5300 2700 5300
Wire Wire Line
	1800 5500 1750 5500
Wire Wire Line
	1800 5300 1800 5500
Connection ~ 1800 5300
Wire Wire Line
	1750 5400 1800 5400
Connection ~ 1800 5400
Wire Wire Line
	1950 5600 2000 5600
Wire Wire Line
	2000 5600 2000 5300
Connection ~ 2000 5300
$Comp
L C C?
U 1 1 584AF5B4
P 2150 5400
F 0 "C?" H 2200 5470 50  0000 C CNN
F 1 "1µ" H 2200 5330 50  0000 C CNN
F 2 "agg:0603" H 2150 5400 50  0001 C CNN
F 3 "" H 2150 5400 50  0001 C CNN
	1    2150 5400
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 584AF621
P 2400 5400
F 0 "C?" H 2450 5470 50  0000 C CNN
F 1 "100n" H 2450 5330 50  0000 C CNN
F 2 "agg:0603" H 2400 5400 50  0001 C CNN
F 3 "" H 2400 5400 50  0001 C CNN
	1    2400 5400
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 584AF68F
P 2150 5550
F 0 "#PWR?" H 2020 5590 50  0001 L CNN
F 1 "GND" H 2150 5450 50  0000 C CNN
F 2 "" H 2150 5550 60  0001 C CNN
F 3 "" H 2150 5550 60  0001 C CNN
	1    2150 5550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 584AF6D0
P 2400 5550
F 0 "#PWR?" H 2270 5590 50  0001 L CNN
F 1 "GND" H 2400 5450 50  0000 C CNN
F 2 "" H 2400 5550 60  0001 C CNN
F 3 "" H 2400 5550 60  0001 C CNN
	1    2400 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 5550 2400 5500
Wire Wire Line
	2150 5500 2150 5550
Wire Wire Line
	2400 5300 2400 5400
Wire Wire Line
	2150 5400 2150 5300
Connection ~ 2150 5300
Connection ~ 2400 5300
$Comp
L NC7SZ04 IC?
U 1 1 584AF782
P 3950 5300
F 0 "IC?" H 3950 5490 50  0000 C CNN
F 1 "NC7SZ04" H 3950 5100 50  0000 C CNN
F 2 "agg:SOT-23-5" H 3940 4820 50  0001 C CNN
F 3 "https://www.fairchildsemi.com/datasheets/NC/NC7SZ04.pdf" H 3950 5020 50  0001 C CNN
F 4 "1417667" H 3950 4910 50  0001 C CNN "Farnell"
	1    3950 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 5300 3750 5300
$Comp
L R R?
U 1 1 584AF964
P 3900 4950
F 0 "R?" H 3950 5000 50  0000 C CNN
F 1 "1M" H 3950 4900 50  0000 C CNN
F 2 "" H 3900 4950 50  0001 C CNN
F 3 "" H 3900 4950 50  0001 C CNN
	1    3900 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 4950 3650 4950
Wire Wire Line
	3650 4950 3650 5300
Connection ~ 3650 5300
Wire Wire Line
	4000 4950 4350 4950
$Comp
L GND #PWR?
U 1 1 584AFAF5
P 3700 5450
F 0 "#PWR?" H 3570 5490 50  0001 L CNN
F 1 "GND" H 3700 5350 50  0000 C CNN
F 2 "" H 3700 5450 60  0001 C CNN
F 3 "" H 3700 5450 60  0001 C CNN
	1    3700 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 5400 3700 5400
Wire Wire Line
	3700 5400 3700 5450
$Comp
L C C?
U 1 1 584AFCBA
P 3500 4800
F 0 "C?" H 3550 4870 50  0000 C CNN
F 1 "100n" H 3550 4730 50  0000 C CNN
F 2 "agg:0603" H 3500 4800 50  0001 C CNN
F 3 "" H 3500 4800 50  0001 C CNN
	1    3500 4800
	0    1    1    0   
$EndComp
$Comp
L 3v3 #PWR?
U 1 1 584AFE50
P 4200 5150
F 0 "#PWR?" H 4200 5260 50  0001 L CNN
F 1 "3v3" H 4200 5240 50  0000 C CNN
F 2 "" H 4200 5150 60  0001 C CNN
F 3 "" H 4200 5150 60  0001 C CNN
	1    4200 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 5200 4200 5200
Wire Wire Line
	4200 5200 4200 5150
$Comp
L 3v3 #PWR?
U 1 1 584B00B4
P 3500 4750
F 0 "#PWR?" H 3500 4860 50  0001 L CNN
F 1 "3v3" H 3500 4840 50  0000 C CNN
F 2 "" H 3500 4750 60  0001 C CNN
F 3 "" H 3500 4750 60  0001 C CNN
	1    3500 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 4750 3500 4800
$Comp
L GND #PWR?
U 1 1 584B0154
P 3500 4950
F 0 "#PWR?" H 3370 4990 50  0001 L CNN
F 1 "GND" H 3500 4850 50  0000 C CNN
F 2 "" H 3500 4950 60  0001 C CNN
F 3 "" H 3500 4950 60  0001 C CNN
	1    3500 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 4950 3500 4900
Wire Wire Line
	4350 4950 4350 5400
Connection ~ 4350 5400
Text Notes 1400 5200 0    40   ~ 0
1v8 LDO for TCXO
Text Notes 3800 5700 0    40   ~ 0
0.8Vpp clipped sine\nto CMOS inverter
Wire Wire Line
	4150 5400 5300 5400
Wire Notes Line
	4500 4450 4500 5300
Wire Notes Line
	4500 4450 850  4450
Wire Notes Line
	850  4450 850  6000
Wire Notes Line
	850  6000 4500 6000
Wire Notes Line
	4500 6000 4500 5500
Text Notes 900  5950 0    50   ~ 0
Low-Jitter Timing Reference from TCXO
Wire Notes Line
	7800 6000 7800 4450
Wire Notes Line
	7800 4450 4950 4450
Text Notes 4600 5950 0    50   ~ 0
PLL to remove jitter from GPS 8MHz
Wire Notes Line
	4550 2700 7800 2700
Wire Notes Line
	7800 2700 7800 4400
Wire Notes Line
	7800 4400 4950 4400
Wire Notes Line
	4550 6000 7800 6000
Wire Notes Line
	4550 5500 4550 6000
Wire Notes Line
	4550 5300 4550 4450
Wire Notes Line
	4550 4450 4750 4450
Wire Notes Line
	4750 4400 4550 4400
Wire Notes Line
	4550 4400 4550 2700
Text Notes 4600 2800 0    50   ~ 0
GPS Receiver
Text Notes 6600 4200 0    40   ~ 0
Active antenna\npower supply
$EndSCHEMATC
