EESchema Schematic File Version 2
LIBS:agg-kicad
LIBS:linetime-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 6
Title "Linetime"
Date "2016-12-14"
Rev "1"
Comp ""
Comment1 "Drawn By: Adam Greig"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L KSZ8081RNx IC402
U 1 1 584B4C22
P 6750 3650
F 0 "IC402" H 6250 4550 50  0000 L CNN
F 1 "KSZ8081RNx" H 6250 2750 50  0000 L CNN
F 2 "agg:QFN-24-EP-MICREL" H 6250 2650 50  0001 L CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/KSZ8081RNA_RND.pdf" H 6250 2550 50  0001 L CNN
F 4 "2509802" H 6250 2450 50  0001 L CNN "Farnell"
	1    6750 3650
	1    0    0    -1  
$EndComp
$Comp
L 749010012A T401
U 1 1 584B1299
P 4200 4000
F 0 "T401" H 4000 4600 50  0000 L CNN
F 1 "749013011A" H 4000 3400 50  0000 L CNN
F 2 "agg:749010012A" H 4000 3300 50  0001 L CNN
F 3 "http://www.farnell.com/datasheets/1816500.pdf" H 4000 3200 50  0001 L CNN
F 4 "2422553" H 4000 3100 50  0001 L CNN "Farnell"
	1    4200 4000
	1    0    0    -1  
$EndComp
$Comp
L R R404
U 1 1 584B947D
P 7450 2850
F 0 "R404" H 7500 2900 50  0000 C CNN
F 1 "33" H 7500 2800 50  0000 C CNN
F 2 "agg:0603" H 7450 2850 50  0001 C CNN
F 3 "" H 7450 2850 50  0001 C CNN
	1    7450 2850
	1    0    0    -1  
$EndComp
$Comp
L R R407
U 1 1 584B94E9
P 7600 2950
F 0 "R407" H 7650 3000 50  0000 C CNN
F 1 "33" H 7650 2900 50  0000 C CNN
F 2 "agg:0603" H 7600 2950 50  0001 C CNN
F 3 "" H 7600 2950 50  0001 C CNN
	1    7600 2950
	1    0    0    -1  
$EndComp
$Comp
L R R405
U 1 1 584B9511
P 7450 3050
F 0 "R405" H 7500 3100 50  0000 C CNN
F 1 "33" H 7500 3000 50  0000 C CNN
F 2 "agg:0603" H 7450 3050 50  0001 C CNN
F 3 "" H 7450 3050 50  0001 C CNN
	1    7450 3050
	1    0    0    -1  
$EndComp
$Comp
L R R408
U 1 1 584B953C
P 7600 3150
F 0 "R408" H 7650 3200 50  0000 C CNN
F 1 "33" H 7650 3100 50  0000 C CNN
F 2 "agg:0603" H 7600 3150 50  0001 C CNN
F 3 "" H 7600 3150 50  0001 C CNN
	1    7600 3150
	1    0    0    -1  
$EndComp
NoConn ~ 7350 3250
Text HLabel 7800 2850 2    40   Output ~ 0
RXD1
Text HLabel 7800 2950 2    40   Output ~ 0
RXD0
Text HLabel 7800 3050 2    40   Output ~ 0
CRS_DV
Text HLabel 7800 3150 2    40   Output ~ 0
REF_CLK
Text HLabel 7800 3350 2    40   Input ~ 0
TXEN
Text HLabel 7800 3450 2    40   Input ~ 0
TXD0
Text HLabel 7800 3550 2    40   Input ~ 0
TXD1
Text HLabel 7800 3750 2    40   BiDi ~ 0
MDIO
Text HLabel 7800 3850 2    40   Input ~ 0
MDC
$Comp
L R R406
U 1 1 584B9CB0
P 7450 4050
F 0 "R406" H 7500 4100 50  0000 C CNN
F 1 "6k49" H 7500 4000 50  0000 C CNN
F 2 "agg:0603" H 7450 4050 50  0001 C CNN
F 3 "" H 7450 4050 50  0001 C CNN
	1    7450 4050
	1    0    0    -1  
$EndComp
NoConn ~ 7350 4150
$Comp
L R R409
U 1 1 584B9D5B
P 8100 3650
F 0 "R409" H 8150 3700 50  0000 C CNN
F 1 "1k" H 8150 3600 50  0000 C CNN
F 2 "agg:0603" H 8100 3650 50  0001 C CNN
F 3 "" H 8100 3650 50  0001 C CNN
	1    8100 3650
	1    0    0    -1  
$EndComp
$Comp
L 3v3 #PWR086
U 1 1 584B9E07
P 4750 2500
F 0 "#PWR086" H 4750 2610 50  0001 L CNN
F 1 "3v3" H 4750 2590 50  0000 C CNN
F 2 "" H 4750 2500 60  0001 C CNN
F 3 "" H 4750 2500 60  0001 C CNN
	1    4750 2500
	1    0    0    -1  
$EndComp
$Comp
L 3v3 #PWR087
U 1 1 584B9E72
P 8250 3600
F 0 "#PWR087" H 8250 3710 50  0001 L CNN
F 1 "3v3" H 8250 3690 50  0000 C CNN
F 2 "" H 8250 3600 60  0001 C CNN
F 3 "" H 8250 3600 60  0001 C CNN
	1    8250 3600
	1    0    0    -1  
$EndComp
$Comp
L D D401
U 1 1 584B9F3E
P 8400 4200
F 0 "D401" H 8450 4270 50  0000 C CNN
F 1 "1N4148" H 8450 4130 50  0000 C CNN
F 2 "agg:SOD-323" H 8400 4200 50  0001 C CNN
F 3 "" H 8400 4200 50  0001 C CNN
F 4 "1466524" H 8400 4200 60  0001 C CNN "Farnell"
	1    8400 4200
	0    1    1    0   
$EndComp
$Comp
L R R410
U 1 1 584B9FA9
P 8600 4200
F 0 "R410" H 8650 4250 50  0000 C CNN
F 1 "10k" H 8650 4150 50  0000 C CNN
F 2 "agg:0603" H 8600 4200 50  0001 C CNN
F 3 "" H 8600 4200 50  0001 C CNN
	1    8600 4200
	0    1    1    0   
$EndComp
$Comp
L C C412
U 1 1 584BA030
P 8500 4400
F 0 "C412" H 8550 4470 50  0000 C CNN
F 1 "10µ" H 8550 4330 50  0000 C CNN
F 2 "agg:0603" H 8500 4400 50  0001 C CNN
F 3 "" H 8500 4400 50  0001 C CNN
	1    8500 4400
	0    1    1    0   
$EndComp
$Comp
L GND #PWR088
U 1 1 584BA2FE
P 8500 4550
F 0 "#PWR088" H 8370 4590 50  0001 L CNN
F 1 "GND" H 8500 4450 50  0000 C CNN
F 2 "" H 8500 4550 60  0001 C CNN
F 3 "" H 8500 4550 60  0001 C CNN
	1    8500 4550
	1    0    0    -1  
$EndComp
$Comp
L 3v3 #PWR089
U 1 1 584BA361
P 8500 4100
F 0 "#PWR089" H 8500 4210 50  0001 L CNN
F 1 "3v3" H 8500 4190 50  0000 C CNN
F 2 "" H 8500 4100 60  0001 C CNN
F 3 "" H 8500 4100 60  0001 C CNN
	1    8500 4100
	1    0    0    -1  
$EndComp
$Comp
L XTAL Y401
U 1 1 584BAA66
P 5950 4150
F 0 "Y401" H 6000 4220 50  0000 C CNN
F 1 "XTAL" H 6000 4080 50  0000 C CNN
F 2 "agg:XTAL-50x32" H 5950 4150 50  0001 C CNN
F 3 "" H 5950 4150 50  0001 C CNN
F 4 "2101328" H 5950 4150 60  0001 C CNN "Farnell"
	1    5950 4150
	0    1    1    0   
$EndComp
$Comp
L C C410
U 1 1 584BABC9
P 5750 4300
F 0 "C410" H 5800 4370 50  0000 C CNN
F 1 "30p" H 5800 4230 50  0000 C CNN
F 2 "agg:0603" H 5750 4300 50  0001 C CNN
F 3 "" H 5750 4300 50  0001 C CNN
	1    5750 4300
	-1   0    0    1   
$EndComp
$Comp
L C C409
U 1 1 584BACDB
P 5750 4100
F 0 "C409" H 5800 4170 50  0000 C CNN
F 1 "30p" H 5800 4030 50  0000 C CNN
F 2 "agg:0603" H 5750 4100 50  0001 C CNN
F 3 "" H 5750 4100 50  0001 C CNN
	1    5750 4100
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR090
U 1 1 584BAE67
P 5500 4200
F 0 "#PWR090" H 5370 4240 50  0001 L CNN
F 1 "GND" H 5500 4100 50  0000 C CNN
F 2 "" H 5500 4200 60  0001 C CNN
F 3 "" H 5500 4200 60  0001 C CNN
	1    5500 4200
	0    1    1    0   
$EndComp
$Comp
L GND #PWR091
U 1 1 584BB204
P 6100 3500
F 0 "#PWR091" H 5970 3540 50  0001 L CNN
F 1 "GND" H 6100 3400 50  0000 C CNN
F 2 "" H 6100 3500 60  0001 C CNN
F 3 "" H 6100 3500 60  0001 C CNN
	1    6100 3500
	1    0    0    -1  
$EndComp
$Comp
L C C411
U 1 1 584BB338
P 5900 3300
F 0 "C411" H 5950 3370 50  0000 C CNN
F 1 "100n" H 5950 3230 50  0000 C CNN
F 2 "agg:0603" H 5900 3300 50  0001 C CNN
F 3 "" H 5900 3300 50  0001 C CNN
	1    5900 3300
	0    -1   -1   0   
$EndComp
$Comp
L C C408
U 1 1 584BB41C
P 5650 3300
F 0 "C408" H 5700 3370 50  0000 C CNN
F 1 "2µ2" H 5700 3230 50  0000 C CNN
F 2 "agg:0603" H 5650 3300 50  0001 C CNN
F 3 "" H 5650 3300 50  0001 C CNN
	1    5650 3300
	0    -1   -1   0   
$EndComp
$Comp
L C C402
U 1 1 584BBD90
P 4500 2850
F 0 "C402" H 4550 2920 50  0000 C CNN
F 1 "22µ" H 4550 2780 50  0000 C CNN
F 2 "agg:0603" H 4500 2850 50  0001 C CNN
F 3 "" H 4500 2850 50  0001 C CNN
	1    4500 2850
	0    -1   -1   0   
$EndComp
$Comp
L C C405
U 1 1 584BBE24
P 4750 2850
F 0 "C405" H 4800 2920 50  0000 C CNN
F 1 "100n" H 4800 2780 50  0000 C CNN
F 2 "agg:0603" H 4750 2850 50  0001 C CNN
F 3 "" H 4750 2850 50  0001 C CNN
	1    4750 2850
	0    -1   -1   0   
$EndComp
$Comp
L L L401
U 1 1 584BBE80
P 4750 2650
F 0 "L401" H 4800 2700 50  0000 C CNN
F 1 "FB" H 4800 2600 50  0000 C CNN
F 2 "agg:0603" H 4750 2650 50  0001 C CNN
F 3 "" H 4750 2650 50  0001 C CNN
F 4 "1669708" H 4750 2650 60  0001 C CNN "Farnell"
	1    4750 2650
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR092
U 1 1 584BC166
P 4500 2900
F 0 "#PWR092" H 4370 2940 50  0001 L CNN
F 1 "GND" H 4500 2800 50  0000 C CNN
F 2 "" H 4500 2900 60  0001 C CNN
F 3 "" H 4500 2900 60  0001 C CNN
	1    4500 2900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR093
U 1 1 584BC26D
P 5900 3350
F 0 "#PWR093" H 5770 3390 50  0001 L CNN
F 1 "GND" H 5900 3250 50  0000 C CNN
F 2 "" H 5900 3350 60  0001 C CNN
F 3 "" H 5900 3350 60  0001 C CNN
	1    5900 3350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR094
U 1 1 584BC2B7
P 5650 3350
F 0 "#PWR094" H 5520 3390 50  0001 L CNN
F 1 "GND" H 5650 3250 50  0000 C CNN
F 2 "" H 5650 3350 60  0001 C CNN
F 3 "" H 5650 3350 60  0001 C CNN
	1    5650 3350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR095
U 1 1 584BC475
P 4750 2900
F 0 "#PWR095" H 4620 2940 50  0001 L CNN
F 1 "GND" H 4750 2800 50  0000 C CNN
F 2 "" H 4750 2900 60  0001 C CNN
F 3 "" H 4750 2900 60  0001 C CNN
	1    4750 2900
	1    0    0    -1  
$EndComp
$Comp
L PWR #FLG096
U 1 1 584BC5A0
P 4500 2650
F 0 "#FLG096" H 4500 2810 50  0001 C CNN
F 1 "PWR" H 4500 2740 50  0000 C CNN
F 2 "" H 4500 2650 50  0001 C CNN
F 3 "" H 4500 2650 50  0001 C CNN
	1    4500 2650
	1    0    0    -1  
$EndComp
$Comp
L C C406
U 1 1 584BCA95
P 5100 3100
F 0 "C406" H 5150 3170 50  0000 C CNN
F 1 "22µ" H 5150 3030 50  0000 C CNN
F 2 "agg:0603" H 5100 3100 50  0001 C CNN
F 3 "" H 5100 3100 50  0001 C CNN
	1    5100 3100
	0    -1   -1   0   
$EndComp
$Comp
L C C407
U 1 1 584BCA9B
P 5350 3100
F 0 "C407" H 5400 3170 50  0000 C CNN
F 1 "100n" H 5400 3030 50  0000 C CNN
F 2 "agg:0603" H 5350 3100 50  0001 C CNN
F 3 "" H 5350 3100 50  0001 C CNN
	1    5350 3100
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR097
U 1 1 584BCAA1
P 5100 3150
F 0 "#PWR097" H 4970 3190 50  0001 L CNN
F 1 "GND" H 5100 3050 50  0000 C CNN
F 2 "" H 5100 3150 60  0001 C CNN
F 3 "" H 5100 3150 60  0001 C CNN
	1    5100 3150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR098
U 1 1 584BCAA8
P 5350 3150
F 0 "#PWR098" H 5220 3190 50  0001 L CNN
F 1 "GND" H 5350 3050 50  0000 C CNN
F 2 "" H 5350 3150 60  0001 C CNN
F 3 "" H 5350 3150 60  0001 C CNN
	1    5350 3150
	1    0    0    -1  
$EndComp
$Comp
L 3v3 #PWR099
U 1 1 584BCC0A
P 5050 2900
F 0 "#PWR099" H 5050 3010 50  0001 L CNN
F 1 "3v3" H 5050 2990 50  0000 C CNN
F 2 "" H 5050 2900 60  0001 C CNN
F 3 "" H 5050 2900 60  0001 C CNN
	1    5050 2900
	1    0    0    -1  
$EndComp
$Comp
L C C403
U 1 1 584BDA5C
P 4600 3600
F 0 "C403" H 4650 3670 50  0000 C CNN
F 1 "100n" H 4650 3530 50  0000 C CNN
F 2 "agg:0603" H 4600 3600 50  0001 C CNN
F 3 "" H 4600 3600 50  0001 C CNN
	1    4600 3600
	1    0    0    -1  
$EndComp
$Comp
L C C404
U 1 1 584BDB40
P 4600 4000
F 0 "C404" H 4650 4070 50  0000 C CNN
F 1 "100n" H 4650 3930 50  0000 C CNN
F 2 "agg:0603" H 4600 4000 50  0001 C CNN
F 3 "" H 4600 4000 50  0001 C CNN
	1    4600 4000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR0100
U 1 1 584BDBA9
P 4750 3600
F 0 "#PWR0100" H 4620 3640 50  0001 L CNN
F 1 "GND" H 4750 3500 50  0000 C CNN
F 2 "" H 4750 3600 60  0001 C CNN
F 3 "" H 4750 3600 60  0001 C CNN
	1    4750 3600
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR0101
U 1 1 584BDCAE
P 4750 4000
F 0 "#PWR0101" H 4620 4040 50  0001 L CNN
F 1 "GND" H 4750 3900 50  0000 C CNN
F 2 "" H 4750 4000 60  0001 C CNN
F 3 "" H 4750 4000 60  0001 C CNN
	1    4750 4000
	0    -1   -1   0   
$EndComp
Text Label 3850 3600 2    40   ~ 0
CT1
Text Label 3850 4000 2    40   ~ 0
CT2
$Comp
L GND #PWR0102
U 1 1 584C02A5
P 7600 4050
F 0 "#PWR0102" H 7470 4090 50  0001 L CNN
F 1 "GND" H 7600 3950 50  0000 C CNN
F 2 "" H 7600 4050 60  0001 C CNN
F 3 "" H 7600 4050 60  0001 C CNN
	1    7600 4050
	0    -1   -1   0   
$EndComp
$Comp
L R R401
U 1 1 584C06BE
P 1750 3500
F 0 "R401" H 1800 3550 50  0000 C CNN
F 1 "1k" H 1800 3450 50  0000 C CNN
F 2 "agg:0603" H 1750 3500 50  0001 C CNN
F 3 "" H 1750 3500 50  0001 C CNN
	1    1750 3500
	-1   0    0    -1  
$EndComp
$Comp
L 3v3 #PWR0103
U 1 1 584C07D6
P 1550 3450
F 0 "#PWR0103" H 1550 3560 50  0001 L CNN
F 1 "3v3" H 1550 3540 50  0000 C CNN
F 2 "" H 1550 3450 60  0001 C CNN
F 3 "" H 1550 3450 60  0001 C CNN
	1    1550 3450
	-1   0    0    -1  
$EndComp
Text Label 3150 4800 0    40   ~ 0
CT1
Text Label 3400 4800 0    40   ~ 0
CT2
$Comp
L R R402
U 1 1 584C35FC
P 3150 4900
F 0 "R402" H 3200 4950 50  0000 C CNN
F 1 "75" H 3200 4850 50  0000 C CNN
F 2 "agg:0603" H 3150 4900 50  0001 C CNN
F 3 "" H 3150 4900 50  0001 C CNN
F 4 "2059571" H 3150 4900 60  0001 C CNN "Farnell"
	1    3150 4900
	0    1    1    0   
$EndComp
$Comp
L R R403
U 1 1 584C3681
P 3400 4900
F 0 "R403" H 3450 4950 50  0000 C CNN
F 1 "75" H 3450 4850 50  0000 C CNN
F 2 "agg:0603" H 3400 4900 50  0001 C CNN
F 3 "" H 3400 4900 50  0001 C CNN
F 4 "2059571" H 3400 4900 60  0001 C CNN "Farnell"
	1    3400 4900
	0    1    1    0   
$EndComp
Text Notes 2900 5200 0    40   ~ 0
Bob Smith terminations
$Comp
L C C401
U 1 1 584C55E5
P 2900 5100
F 0 "C401" H 2950 5170 50  0000 C CNN
F 1 "1n 2kV" H 2950 5030 50  0000 C CNN
F 2 "agg:0805" H 2900 5100 50  0001 C CNN
F 3 "" H 2900 5100 50  0001 C CNN
	1    2900 5100
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR0104
U 1 1 584C57B0
P 2750 5150
F 0 "#PWR0104" H 2620 5190 50  0001 L CNN
F 1 "GND" H 2750 5050 50  0000 C CNN
F 2 "" H 2750 5150 60  0001 C CNN
F 3 "" H 2750 5150 60  0001 C CNN
	1    2750 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 2850 7450 2850
Wire Wire Line
	7350 2950 7600 2950
Wire Wire Line
	7350 3050 7450 3050
Wire Wire Line
	7350 3150 7600 3150
Wire Wire Line
	7550 2850 7800 2850
Wire Wire Line
	7700 2950 7800 2950
Wire Wire Line
	7550 3050 7800 3050
Wire Wire Line
	7700 3150 7800 3150
Wire Wire Line
	7350 3350 7800 3350
Wire Wire Line
	7350 3450 7800 3450
Wire Wire Line
	7350 3550 7800 3550
Wire Wire Line
	7350 3750 7800 3750
Wire Wire Line
	7350 3850 7800 3850
Wire Wire Line
	7450 4050 7350 4050
Wire Wire Line
	8100 3650 7700 3650
Wire Wire Line
	7700 3650 7700 3750
Connection ~ 7700 3750
Wire Wire Line
	8200 3650 8250 3650
Wire Wire Line
	8250 3650 8250 3600
Wire Wire Line
	7350 4350 8600 4350
Wire Wire Line
	8600 4350 8600 4300
Wire Wire Line
	8500 4400 8500 4350
Connection ~ 8500 4350
Wire Wire Line
	8400 4350 8400 4300
Connection ~ 8400 4350
Wire Wire Line
	8500 4550 8500 4500
Wire Wire Line
	8500 4100 8500 4150
Wire Wire Line
	8400 4150 8600 4150
Wire Wire Line
	8400 4150 8400 4200
Wire Wire Line
	8600 4150 8600 4200
Connection ~ 8500 4150
Wire Wire Line
	7350 4250 7500 4250
Wire Wire Line
	5750 4100 6050 4100
Wire Wire Line
	6050 4100 6050 4150
Wire Wire Line
	6050 4150 6150 4150
Wire Wire Line
	6150 4250 6050 4250
Wire Wire Line
	6050 4250 6050 4300
Wire Wire Line
	6050 4300 5750 4300
Wire Wire Line
	5950 4250 5950 4300
Connection ~ 5950 4300
Wire Wire Line
	5950 4150 5950 4100
Connection ~ 5950 4100
Wire Wire Line
	5650 4100 5550 4100
Wire Wire Line
	5550 4100 5550 4300
Wire Wire Line
	5550 4300 5650 4300
Wire Wire Line
	5500 4200 5550 4200
Connection ~ 5550 4200
Wire Wire Line
	6150 3350 6100 3350
Wire Wire Line
	6100 3350 6100 3500
Wire Wire Line
	5650 3150 6150 3150
Wire Wire Line
	5650 3150 5650 3200
Wire Wire Line
	5900 3200 5900 3150
Connection ~ 5900 3150
Wire Wire Line
	4750 2650 4750 2750
Connection ~ 4750 2700
Wire Wire Line
	4500 2650 4500 2750
Wire Wire Line
	6100 3450 6150 3450
Connection ~ 6100 3450
Wire Wire Line
	5900 3350 5900 3300
Wire Wire Line
	5650 3350 5650 3300
Wire Wire Line
	4500 2900 4500 2850
Wire Wire Line
	4750 2850 4750 2900
Wire Wire Line
	4750 2500 4750 2550
Connection ~ 4500 2700
Wire Wire Line
	5100 3150 5100 3100
Wire Wire Line
	5350 3100 5350 3150
Wire Wire Line
	5050 2950 6150 2950
Wire Wire Line
	5100 2950 5100 3000
Wire Wire Line
	5350 3000 5350 2950
Connection ~ 5350 2950
Wire Wire Line
	5050 2900 5050 2950
Connection ~ 5100 2950
Wire Wire Line
	6150 2850 5250 2850
Wire Wire Line
	5250 2850 5250 2700
Wire Wire Line
	5250 2700 4500 2700
Wire Wire Line
	4500 3700 4550 3700
Wire Wire Line
	4550 3700 4550 3750
Wire Wire Line
	4550 3750 6150 3750
Wire Wire Line
	4500 3500 5250 3500
Wire Wire Line
	5250 3500 5250 3650
Wire Wire Line
	5250 3650 6150 3650
Wire Wire Line
	4500 3900 4550 3900
Wire Wire Line
	4550 3900 4550 3850
Wire Wire Line
	4550 3850 6150 3850
Wire Wire Line
	4500 4100 5250 4100
Wire Wire Line
	5250 4100 5250 3950
Wire Wire Line
	5250 3950 6150 3950
Wire Wire Line
	4750 4000 4700 4000
Wire Wire Line
	4600 4000 4500 4000
Wire Wire Line
	4600 3600 4500 3600
Wire Wire Line
	4700 3600 4750 3600
Wire Wire Line
	3300 4100 3900 4100
Wire Wire Line
	3300 4000 3650 4000
Wire Wire Line
	3650 4000 3650 3900
Wire Wire Line
	3650 3900 3900 3900
Wire Wire Line
	3300 3900 3550 3900
Wire Wire Line
	3550 3900 3550 3700
Wire Wire Line
	3550 3700 3900 3700
Wire Wire Line
	3300 3600 3650 3600
Wire Wire Line
	3650 3600 3650 3500
Wire Wire Line
	3650 3500 3900 3500
Wire Wire Line
	3900 4000 3850 4000
Wire Wire Line
	3900 3600 3850 3600
Wire Wire Line
	7550 4050 7600 4050
Wire Wire Line
	1750 3500 2300 3500
Wire Wire Line
	1650 3500 1550 3500
Wire Wire Line
	1550 3500 1550 3450
Wire Wire Line
	3150 4800 3150 4900
Wire Wire Line
	3400 4800 3400 4900
Wire Wire Line
	3400 5100 3400 5000
Wire Wire Line
	3150 5100 3150 5000
Connection ~ 3150 5100
Wire Notes Line
	4200 2800 4200 5400
Text Notes 2900 2950 0    50   ~ 0
ETHERNET ISOLATION
Wire Wire Line
	2800 5100 2750 5100
Wire Wire Line
	2750 5100 2750 5150
Text Notes 6500 2750 0    50   ~ 0
Ethernet PHY
NoConn ~ 3300 3400
NoConn ~ 3300 3500
NoConn ~ 3300 3700
NoConn ~ 3300 3800
Wire Wire Line
	2900 5100 3400 5100
$Comp
L RJHSE-538x IC401
U 1 1 584B6674
P 2800 3800
F 0 "IC401" H 2400 4300 50  0000 L CNN
F 1 "RJHSE-5381" H 2400 3300 50  0000 L CNN
F 2 "agg:RJHSE-538X" H 2400 3200 50  0001 L CNN
F 3 "http://www.farnell.com/cad/2167247.pdf" H 2400 3100 50  0001 L CNN
F 4 "1462758" H 2400 3000 50  0001 L CNN "Farnell"
	1    2800 3800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR0105
U 1 1 584B6EFC
P 2050 4050
F 0 "#PWR0105" H 1920 4090 50  0001 L CNN
F 1 "GND" H 2050 3950 50  0000 C CNN
F 2 "" H 2050 4050 60  0001 C CNN
F 3 "" H 2050 4050 60  0001 C CNN
	1    2050 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 4000 2050 4000
Wire Wire Line
	2050 4000 2050 4050
Wire Notes Line
	4200 2800 2850 2800
Wire Notes Line
	2850 2800 2850 5400
Wire Notes Line
	2850 5400 4200 5400
Text Label 7500 4250 0    60   ~ 0
LINK_LED
Text Label 2200 3400 2    60   ~ 0
LINK_LED
Wire Wire Line
	2200 3400 2300 3400
NoConn ~ 2300 3700
NoConn ~ 2300 3800
$EndSCHEMATC