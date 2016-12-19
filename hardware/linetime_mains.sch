EESchema Schematic File Version 2
LIBS:agg-kicad
LIBS:linetime-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 6
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
L CONN_01x03 J201
U 1 1 5849DAE5
P 3850 3550
F 0 "J201" H 3800 3650 50  0000 L CNN
F 1 "FUSED INLET" V 3750 3450 50  0000 C CNN
F 2 "agg:DD21.01xx.1111" H 3850 3550 50  0001 C CNN
F 3 "http://www.farnell.com/datasheets/2097461.pdf" H 3850 3550 50  0001 C CNN
F 4 "1517985" H 3850 3550 60  0001 C CNN "Farnell"
	1    3850 3550
	1    0    0    -1  
$EndComp
$Comp
L TRANSFORMER_2x2 T201
U 1 1 5849DBA5
P 4800 4400
F 0 "T201" H 4800 4775 50  0000 C CNN
F 1 "AVB0.35/2/6" H 4800 4125 50  0000 C CNN
F 2 "agg:AVB0.35_2_X" H 4600 4650 50  0001 C CNN
F 3 "http://www.farnell.com/datasheets/1968818.pdf" H 4600 4650 50  0001 C CNN
F 4 "1131455" H 4800 4400 60  0001 C CNN "Farnell"
	1    4800 4400
	1    0    0    -1  
$EndComp
$Comp
L VTX-214-003-1xx IC201
U 1 1 5849DC00
P 4800 3650
F 0 "IC201" H 4500 3850 50  0000 L CNN
F 1 "VTX-214-003-105" H 4500 3450 50  0000 L CNN
F 2 "agg:VTX-214-003-1xx" H 4500 3350 50  0001 L CNN
F 3 "http://www.farnell.com/datasheets/2060418.pdf" H 4500 3250 50  0001 L CNN
F 4 "2401030" H 4500 3150 50  0001 L CNN "Farnell"
	1    4800 3650
	1    0    0    -1  
$EndComp
$Comp
L PART X201
U 1 1 5849DC48
P 3450 3400
F 0 "X201" H 3500 3500 50  0000 L CNN
F 1 "FUSEHOLDER" H 3450 3400 50  0000 L CNN
F 2 "" H 3450 3400 50  0001 C CNN
F 3 "http://www.farnell.com/datasheets/2096561.pdf" H 3450 3400 50  0001 C CNN
F 4 "1144906" H 3450 3400 60  0001 C CNN "Farnell"
	1    3450 3400
	0    1    1    0   
$EndComp
$Comp
L GND #PWR201
U 1 1 5849E0DD
P 4000 5100
F 0 "#PWR201" H 3870 5140 50  0001 L CNN
F 1 "GND" H 4000 5000 50  0000 C CNN
F 2 "" H 4000 5100 60  0001 C CNN
F 3 "" H 4000 5100 60  0001 C CNN
	1    4000 5100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR204
U 1 1 5849E2FC
P 5350 3700
F 0 "#PWR204" H 5220 3740 50  0001 L CNN
F 1 "GND" H 5350 3600 50  0000 C CNN
F 2 "" H 5350 3700 60  0001 C CNN
F 3 "" H 5350 3700 60  0001 C CNN
	1    5350 3700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR217
U 1 1 5849F658
P 7800 4350
F 0 "#PWR217" H 7670 4390 50  0001 L CNN
F 1 "GND" H 7800 4250 50  0000 C CNN
F 2 "" H 7800 4350 60  0001 C CNN
F 3 "" H 7800 4350 60  0001 C CNN
	1    7800 4350
	1    0    0    -1  
$EndComp
Text Notes 3300 4950 0    50   ~ 0
MAINS ISOLATION
$Comp
L R_DIVIDER R202
U 1 1 584A02AB
P 7800 4200
F 0 "R202" H 7800 4150 50  0000 C CNN
F 1 "1k/1k" H 7800 4080 50  0000 C CNN
F 2 "agg:SOT-23" H 7800 4010 50  0001 C CNN
F 3 "" H 7790 4250 50  0001 C CNN
F 4 "1203390" H 7800 4200 60  0001 C CNN "Farnell"
	1    7800 4200
	0    -1   1    0   
$EndComp
$Comp
L R_DIVIDER R201
U 1 1 584A06BD
P 6400 4100
F 0 "R201" H 6400 4050 50  0000 C CNN
F 1 "1k/5k" H 6400 3980 50  0000 C CNN
F 2 "agg:SOT-23" H 6400 3910 50  0001 C CNN
F 3 "" H 6390 4150 50  0001 C CNN
F 4 "2325454" H 6400 4100 60  0001 C CNN "Farnell"
	1    6400 4100
	0    1    -1   0   
$EndComp
$Comp
L GND #PWR213
U 1 1 584A0845
P 7400 4650
F 0 "#PWR213" H 7270 4690 50  0001 L CNN
F 1 "GND" H 7400 4550 50  0000 C CNN
F 2 "" H 7400 4650 60  0001 C CNN
F 3 "" H 7400 4650 60  0001 C CNN
	1    7400 4650
	1    0    0    -1  
$EndComp
$Comp
L C C211
U 1 1 584A09BD
P 8100 4400
F 0 "C211" H 8150 4470 50  0000 C CNN
F 1 "100n" H 8150 4330 50  0000 C CNN
F 2 "agg:0603" H 8100 4400 50  0001 C CNN
F 3 "" H 8100 4400 50  0001 C CNN
	1    8100 4400
	0    1    1    0   
$EndComp
$Comp
L GND #PWR223
U 1 1 584A0B05
P 8100 4550
F 0 "#PWR223" H 7970 4590 50  0001 L CNN
F 1 "GND" H 8100 4450 50  0000 C CNN
F 2 "" H 8100 4550 60  0001 C CNN
F 3 "" H 8100 4550 60  0001 C CNN
	1    8100 4550
	1    0    0    -1  
$EndComp
$Comp
L C C208
U 1 1 584A0C18
P 6600 4400
F 0 "C208" H 6650 4470 50  0000 C CNN
F 1 "100n" H 6650 4330 50  0000 C CNN
F 2 "agg:0603" H 6600 4400 50  0001 C CNN
F 3 "" H 6600 4400 50  0001 C CNN
	1    6600 4400
	0    1    1    0   
$EndComp
$Comp
L GND #PWR207
U 1 1 584A0C89
P 6600 4550
F 0 "#PWR207" H 6470 4590 50  0001 L CNN
F 1 "GND" H 6600 4450 50  0000 C CNN
F 2 "" H 6600 4550 60  0001 C CNN
F 3 "" H 6600 4550 60  0001 C CNN
	1    6600 4550
	1    0    0    -1  
$EndComp
$Comp
L C C203
U 1 1 584A11DF
P 4400 4300
F 0 "C203" H 4450 4370 50  0000 C CNN
F 1 "X2" H 4450 4230 50  0000 C CNN
F 2 "agg:B32921" H 4400 4300 50  0001 C CNN
F 3 "" H 4400 4300 50  0001 C CNN
F 4 "2112770" H 4400 4300 60  0001 C CNN "Farnell"
	1    4400 4300
	0    1    1    0   
$EndComp
$Comp
L C C201
U 1 1 584A1414
P 4150 4100
F 0 "C201" H 4200 4170 50  0000 C CNN
F 1 "Y2" H 4200 4030 50  0000 C CNN
F 2 "agg:B32921" H 4150 4100 50  0001 C CNN
F 3 "" H 4150 4100 50  0001 C CNN
F 4 "2395807" H 4150 4100 60  0001 C CNN "Farnell"
	1    4150 4100
	-1   0    0    1   
$EndComp
$Comp
L C C202
U 1 1 584A15E0
P 4150 4450
F 0 "C202" H 4200 4520 50  0000 C CNN
F 1 "Y2" H 4200 4380 50  0000 C CNN
F 2 "agg:B32921" H 4150 4450 50  0001 C CNN
F 3 "" H 4150 4450 50  0001 C CNN
F 4 "2395807" H 4150 4450 60  0001 C CNN "Farnell"
	1    4150 4450
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR211
U 1 1 584A1EDE
P 7350 2300
F 0 "#PWR211" H 7220 2340 50  0001 L CNN
F 1 "GND" H 7350 2200 50  0000 C CNN
F 2 "" H 7350 2300 60  0001 C CNN
F 3 "" H 7350 2300 60  0001 C CNN
	1    7350 2300
	1    0    0    -1  
$EndComp
$Comp
L C C209
U 1 1 584A213E
P 6950 1400
F 0 "C209" H 7000 1470 50  0000 C CNN
F 1 "10n" H 7000 1330 50  0000 C CNN
F 2 "agg:0603" H 6950 1400 50  0001 C CNN
F 3 "" H 6950 1400 50  0001 C CNN
	1    6950 1400
	0    1    1    0   
$EndComp
$Comp
L GND #PWR209
U 1 1 584A214B
P 6950 1550
F 0 "#PWR209" H 6820 1590 50  0001 L CNN
F 1 "GND" H 6950 1450 50  0000 C CNN
F 2 "" H 6950 1550 60  0001 C CNN
F 3 "" H 6950 1550 60  0001 C CNN
	1    6950 1550
	1    0    0    -1  
$EndComp
Text HLabel 8050 4850 2    60   Output ~ 0
MAINS_BIAS
Text HLabel 8050 1950 2    60   Output ~ 0
MAINS_WAVE
$Comp
L L L201
U 1 1 584A2DCA
P 4000 4800
F 0 "L201" H 4050 4850 50  0000 C CNN
F 1 "FB" H 4050 4750 50  0000 C CNN
F 2 "agg:0603" H 4000 4800 50  0001 C CNN
F 3 "" H 4000 4800 50  0001 C CNN
F 4 "1669708" H 4000 4800 60  0001 C CNN "Farnell"
	1    4000 4800
	0    -1   -1   0   
$EndComp
Text Label 3950 3550 0    40   ~ 0
LIVE
Text Label 3950 3650 0    40   ~ 0
NEUTRAL
Text Label 3950 3750 0    40   ~ 0
EARTH
$Comp
L ADA4528-2 IC204
U 2 1 584A19D7
P 7400 4300
F 0 "IC204" H 7500 4500 50  0000 L CNN
F 1 "ADA4528-2" H 7500 4100 50  0000 L CNN
F 2 "agg:MSOP-8" H 7400 3700 50  0001 C CNN
F 3 "http://www.analog.com/media/en/technical-documentation/data-sheets/ADA4528-1_4528-2.pdf" H 7400 3900 50  0001 C CNN
F 4 "2213573" H 7400 3800 50  0001 C CNN "Farnell"
	2    7400 4300
	-1   0    0    -1  
$EndComp
$Comp
L ADA4528-2 IC204
U 1 1 584A1B71
P 7350 1950
F 0 "IC204" H 7450 2150 50  0000 L CNN
F 1 "ADA4528-2" H 7450 1750 50  0000 L CNN
F 2 "agg:MSOP-8" H 7350 1350 50  0001 C CNN
F 3 "http://www.analog.com/media/en/technical-documentation/data-sheets/ADA4528-1_4528-2.pdf" H 7350 1550 50  0001 C CNN
F 4 "2213573" H 7350 1450 50  0001 C CNN "Farnell"
	1    7350 1950
	1    0    0    -1  
$EndComp
$Comp
L C C207
U 1 1 584A3157
P 6000 4050
F 0 "C207" H 6050 4120 50  0000 C CNN
F 1 "1n" H 6050 3980 50  0000 C CNN
F 2 "agg:0603" H 6000 4050 50  0001 C CNN
F 3 "" H 6000 4050 50  0001 C CNN
	1    6000 4050
	0    1    1    0   
$EndComp
$Comp
L ADCMP601 IC203
U 1 1 584A21B3
P 7250 3250
F 0 "IC203" H 7510 3490 50  0000 L CNN
F 1 "ADCMP601" H 7520 3140 50  0000 L CNN
F 2 "agg:SC-70-6" H 7250 2830 50  0001 C CNN
F 3 "http://www.analog.com/media/en/technical-documentation/data-sheets/ADCMP600_601_602.pdf" H 7250 2900 50  0001 C CNN
F 4 "1331025" H 7250 2970 50  0001 C CNN "Farnell"
	1    7250 3250
	1    0    0    -1  
$EndComp
$Comp
L R R204
U 1 1 584A2335
P 8100 3750
F 0 "R204" H 8150 3800 50  0000 C CNN
F 1 "DNS" H 8150 3700 50  0000 C CNN
F 2 "agg:0603" H 8100 3750 50  0001 C CNN
F 3 "" H 8100 3750 50  0001 C CNN
	1    8100 3750
	0    1    1    0   
$EndComp
$Comp
L R R203
U 1 1 584A2387
P 8100 3550
F 0 "R203" H 8150 3600 50  0000 C CNN
F 1 "DNS" H 8150 3500 50  0000 C CNN
F 2 "agg:0603" H 8100 3550 50  0001 C CNN
F 3 "" H 8100 3550 50  0001 C CNN
	1    8100 3550
	0    1    1    0   
$EndComp
$Comp
L GND #PWR215
U 1 1 584A23EB
P 7450 3500
F 0 "#PWR215" H 7320 3540 50  0001 L CNN
F 1 "GND" H 7450 3400 50  0000 C CNN
F 2 "" H 7450 3500 60  0001 C CNN
F 3 "" H 7450 3500 60  0001 C CNN
	1    7450 3500
	1    0    0    -1  
$EndComp
$Comp
L C C212
U 1 1 584A25F4
P 8150 2850
F 0 "C212" H 8200 2920 50  0000 C CNN
F 1 "10n" H 8200 2780 50  0000 C CNN
F 2 "agg:0603" H 8150 2850 50  0001 C CNN
F 3 "" H 8150 2850 50  0001 C CNN
	1    8150 2850
	0    1    1    0   
$EndComp
$Comp
L GND #PWR225
U 1 1 584A2601
P 8150 3000
F 0 "#PWR225" H 8020 3040 50  0001 L CNN
F 1 "GND" H 8150 2900 50  0000 C CNN
F 2 "" H 8150 3000 60  0001 C CNN
F 3 "" H 8150 3000 60  0001 C CNN
	1    8150 3000
	1    0    0    -1  
$EndComp
Text HLabel 8250 3250 2    60   Output ~ 0
MAINS_ZC
$Comp
L C C210
U 1 1 584A2BD3
P 7900 2850
F 0 "C210" H 7950 2920 50  0000 C CNN
F 1 "100n" H 7950 2780 50  0000 C CNN
F 2 "agg:0603" H 7900 2850 50  0001 C CNN
F 3 "" H 7900 2850 50  0001 C CNN
	1    7900 2850
	0    1    1    0   
$EndComp
$Comp
L GND #PWR219
U 1 1 584A2BE0
P 7900 3000
F 0 "#PWR219" H 7770 3040 50  0001 L CNN
F 1 "GND" H 7900 2900 50  0000 C CNN
F 2 "" H 7900 3000 60  0001 C CNN
F 3 "" H 7900 3000 60  0001 C CNN
	1    7900 3000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR221
U 1 1 584A40DE
P 8100 3900
F 0 "#PWR221" H 7970 3940 50  0001 L CNN
F 1 "GND" H 8100 3800 50  0000 C CNN
F 2 "" H 8100 3900 60  0001 C CNN
F 3 "" H 8100 3900 60  0001 C CNN
	1    8100 3900
	1    0    0    -1  
$EndComp
Text Notes 8200 3800 0    40   ~ 0
Hysteresis adjust.\nLeave floating for 2mV,\nshort to VCC to 0mV,\nresistor to GND for other
$Comp
L PWR #FLG201
U 1 1 584A4D3C
P 4350 3500
F 0 "#FLG201" H 4350 3660 50  0001 C CNN
F 1 "PWR" H 4350 3590 50  0000 C CNN
F 2 "" H 4350 3500 50  0001 C CNN
F 3 "" H 4350 3500 50  0001 C CNN
	1    4350 3500
	1    0    0    -1  
$EndComp
$Comp
L PWR #FLG202
U 1 1 584A4D90
P 4350 3700
F 0 "#FLG202" H 4350 3860 50  0001 C CNN
F 1 "PWR" H 4350 3790 50  0000 C CNN
F 2 "" H 4350 3700 50  0001 C CNN
F 3 "" H 4350 3700 50  0001 C CNN
	1    4350 3700
	-1   0    0    1   
$EndComp
Text Notes 9050 1250 2    50   ~ 0
Mains waveform buffer
Text Notes 9050 2650 2    50   ~ 0
Mains zero-crossing detector
Text Notes 9050 4150 2    50   ~ 0
Mains biasing
Text HLabel 5350 3550 2    60   Output ~ 0
MAINS_5V
$Comp
L ADP3335 IC202
U 1 1 584B188F
P 5150 1950
F 0 "IC202" H 4950 2150 50  0000 L CNN
F 1 "ADP3335" H 4950 1650 50  0000 L CNN
F 2 "agg:MSOP-8" H 4950 1550 50  0001 L CNN
F 3 "" H 5550 1550 50  0001 C CNN
F 4 "2067775" H 4950 1450 50  0001 L CNN "Farnell"
	1    5150 1950
	1    0    0    -1  
$EndComp
$Comp
L AVCC #PWR208
U 1 1 584B1AAE
P 6950 1350
F 0 "#PWR208" H 6950 1460 50  0001 L CNN
F 1 "AVCC" H 6950 1440 50  0000 C CNN
F 2 "" H 6950 1350 60  0001 C CNN
F 3 "" H 6950 1350 60  0001 C CNN
	1    6950 1350
	1    0    0    -1  
$EndComp
$Comp
L AVCC #PWR210
U 1 1 584B1B05
P 7350 1600
F 0 "#PWR210" H 7350 1710 50  0001 L CNN
F 1 "AVCC" H 7350 1690 50  0000 C CNN
F 2 "" H 7350 1600 60  0001 C CNN
F 3 "" H 7350 1600 60  0001 C CNN
	1    7350 1600
	1    0    0    -1  
$EndComp
$Comp
L AVCC #PWR224
U 1 1 584B1B55
P 8150 2800
F 0 "#PWR224" H 8150 2910 50  0001 L CNN
F 1 "AVCC" H 8150 2890 50  0000 C CNN
F 2 "" H 8150 2800 60  0001 C CNN
F 3 "" H 8150 2800 60  0001 C CNN
	1    8150 2800
	1    0    0    -1  
$EndComp
$Comp
L AVCC #PWR218
U 1 1 584B1BA5
P 7900 2800
F 0 "#PWR218" H 7900 2910 50  0001 L CNN
F 1 "AVCC" H 7900 2890 50  0000 C CNN
F 2 "" H 7900 2800 60  0001 C CNN
F 3 "" H 7900 2800 60  0001 C CNN
	1    7900 2800
	1    0    0    -1  
$EndComp
$Comp
L AVCC #PWR214
U 1 1 584B1BF5
P 7450 3000
F 0 "#PWR214" H 7450 3110 50  0001 L CNN
F 1 "AVCC" H 7450 3090 50  0000 C CNN
F 2 "" H 7450 3000 60  0001 C CNN
F 3 "" H 7450 3000 60  0001 C CNN
	1    7450 3000
	1    0    0    -1  
$EndComp
$Comp
L AVCC #PWR220
U 1 1 584B1C45
P 8100 3500
F 0 "#PWR220" H 8100 3610 50  0001 L CNN
F 1 "AVCC" H 8100 3590 50  0000 C CNN
F 2 "" H 8100 3500 60  0001 C CNN
F 3 "" H 8100 3500 60  0001 C CNN
	1    8100 3500
	1    0    0    -1  
$EndComp
$Comp
L AVCC #PWR222
U 1 1 584B1CF1
P 8100 4350
F 0 "#PWR222" H 8100 4460 50  0001 L CNN
F 1 "AVCC" H 8100 4440 50  0000 C CNN
F 2 "" H 8100 4350 60  0001 C CNN
F 3 "" H 8100 4350 60  0001 C CNN
	1    8100 4350
	1    0    0    -1  
$EndComp
$Comp
L AVCC #PWR216
U 1 1 584B1D41
P 7800 4050
F 0 "#PWR216" H 7800 4160 50  0001 L CNN
F 1 "AVCC" H 7800 4140 50  0000 C CNN
F 2 "" H 7800 4050 60  0001 C CNN
F 3 "" H 7800 4050 60  0001 C CNN
	1    7800 4050
	1    0    0    -1  
$EndComp
$Comp
L AVCC #PWR212
U 1 1 584B1D91
P 7400 3950
F 0 "#PWR212" H 7400 4060 50  0001 L CNN
F 1 "AVCC" H 7400 4040 50  0000 C CNN
F 2 "" H 7400 3950 60  0001 C CNN
F 3 "" H 7400 3950 60  0001 C CNN
	1    7400 3950
	1    0    0    -1  
$EndComp
$Comp
L AVCC #PWR206
U 1 1 584B2519
P 5900 1800
F 0 "#PWR206" H 5900 1910 50  0001 L CNN
F 1 "AVCC" H 5900 1890 50  0000 C CNN
F 2 "" H 5900 1800 60  0001 C CNN
F 3 "" H 5900 1800 60  0001 C CNN
	1    5900 1800
	1    0    0    -1  
$EndComp
$Comp
L C C206
U 1 1 584B25DC
P 5800 1900
F 0 "C206" H 5850 1970 50  0000 C CNN
F 1 "1µ" H 5850 1830 50  0000 C CNN
F 2 "agg:0603" H 5800 1900 50  0001 C CNN
F 3 "" H 5800 1900 50  0001 C CNN
	1    5800 1900
	0    1    1    0   
$EndComp
$Comp
L C C205
U 1 1 584B27D7
P 5500 2150
F 0 "C205" H 5550 2220 50  0000 C CNN
F 1 "10n" H 5550 2080 50  0000 C CNN
F 2 "agg:0603" H 5500 2150 50  0001 C CNN
F 3 "" H 5500 2150 50  0001 C CNN
	1    5500 2150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR205
U 1 1 584B2AFC
P 5800 2050
F 0 "#PWR205" H 5670 2090 50  0001 L CNN
F 1 "GND" H 5800 1950 50  0000 C CNN
F 2 "" H 5800 2050 60  0001 C CNN
F 3 "" H 5800 2050 60  0001 C CNN
	1    5800 2050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR203
U 1 1 584B2E5F
P 4800 2200
F 0 "#PWR203" H 4670 2240 50  0001 L CNN
F 1 "GND" H 4800 2100 50  0000 C CNN
F 2 "" H 4800 2200 60  0001 C CNN
F 3 "" H 4800 2200 60  0001 C CNN
	1    4800 2200
	1    0    0    -1  
$EndComp
Text Notes 5650 5100 0    50   ~ 0
Mains filtering
$Comp
L C C204
U 1 1 584B397D
P 4650 1900
F 0 "C204" H 4700 1970 50  0000 C CNN
F 1 "1µ" H 4700 1830 50  0000 C CNN
F 2 "agg:0603" H 4650 1900 50  0001 C CNN
F 3 "" H 4650 1900 50  0001 C CNN
	1    4650 1900
	0    1    1    0   
$EndComp
Text Notes 6050 2400 2    50   ~ 0
3v3 LDO for analogue
$Comp
L VCC #PWR202
U 1 1 584C85C7
P 4550 1800
F 0 "#PWR202" H 4550 1910 50  0001 L CNN
F 1 "VCC" H 4550 1890 50  0000 C CNN
F 2 "" H 4550 1800 60  0001 C CNN
F 3 "" H 4550 1800 60  0001 C CNN
	1    4550 1800
	1    0    0    -1  
$EndComp
NoConn ~ 5000 4400
NoConn ~ 5000 4600
$Comp
L R R205
U 1 1 5852D944
P 8050 3250
F 0 "R205" H 8100 3300 50  0000 C CNN
F 1 "33" H 8100 3200 50  0000 C CNN
F 2 "agg:0603" H 8050 3250 50  0001 C CNN
F 3 "" H 8050 3250 50  0001 C CNN
	1    8050 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 3750 4000 3750
Wire Wire Line
	4000 3750 4000 4100
Wire Wire Line
	4000 4100 4000 4450
Wire Wire Line
	4000 4450 4000 4700
Wire Wire Line
	3950 3550 4300 3550
Wire Wire Line
	4300 3550 4350 3550
Wire Wire Line
	4350 3550 4400 3550
Wire Wire Line
	3950 3650 4200 3650
Wire Wire Line
	4200 3650 4350 3650
Wire Wire Line
	4350 3650 4400 3650
Wire Wire Line
	5200 3650 5350 3650
Wire Wire Line
	5350 3650 5350 3700
Wire Wire Line
	4300 4100 4400 4100
Wire Wire Line
	4400 4100 4600 4100
Wire Wire Line
	4300 4100 4300 3550
Connection ~ 4300 3550
Wire Wire Line
	4600 4300 4550 4300
Wire Wire Line
	4550 4300 4550 4400
Wire Wire Line
	4550 4400 4600 4400
Wire Wire Line
	4200 4600 4400 4600
Wire Wire Line
	4400 4600 4600 4600
Wire Wire Line
	4200 3650 4200 4100
Wire Wire Line
	4200 4100 4200 4600
Connection ~ 4200 3650
Wire Wire Line
	6400 4300 6400 4200
Connection ~ 6400 4300
Wire Wire Line
	7800 4050 7800 4100
Wire Wire Line
	7800 4300 7800 4350
Wire Notes Line
	4800 3200 3250 3200
Wire Wire Line
	5200 3550 5350 3550
Wire Wire Line
	7400 4600 7400 4650
Wire Wire Line
	7600 4400 7650 4400
Wire Wire Line
	7650 4400 7650 4850
Wire Wire Line
	6900 4850 7650 4850
Wire Wire Line
	7650 4850 8050 4850
Wire Wire Line
	6900 4300 6900 4850
Wire Wire Line
	7400 3950 7400 4000
Wire Wire Line
	7700 4200 7600 4200
Wire Wire Line
	8100 4350 8100 4400
Wire Wire Line
	8100 4500 8100 4550
Wire Wire Line
	6600 4550 6600 4500
Wire Wire Line
	6600 4300 6600 4400
Connection ~ 6900 4300
Connection ~ 6600 4300
Connection ~ 4400 4100
Connection ~ 4400 4600
Wire Notes Line
	3250 3200 3250 5000
Wire Notes Line
	3250 5000 4800 5000
Wire Notes Line
	4800 5000 4800 3200
Wire Wire Line
	7350 2300 7350 2250
Wire Wire Line
	7100 2450 7800 2450
Wire Wire Line
	7800 2450 7800 1950
Wire Wire Line
	7650 1950 7800 1950
Wire Wire Line
	7800 1950 8050 1950
Wire Wire Line
	6900 1850 7150 1850
Wire Wire Line
	7350 1650 7350 1600
Wire Wire Line
	6950 1350 6950 1400
Wire Wire Line
	6950 1500 6950 1550
Connection ~ 7800 1950
Connection ~ 7650 4850
Wire Wire Line
	4000 4800 4000 5100
Wire Wire Line
	4400 4100 4400 4300
Wire Wire Line
	4150 4100 4200 4100
Connection ~ 4200 4100
Wire Wire Line
	4000 4100 4050 4100
Connection ~ 4000 4100
Wire Wire Line
	4050 4450 4000 4450
Connection ~ 4000 4450
Wire Wire Line
	4150 4450 4400 4450
Connection ~ 4400 4450
Wire Wire Line
	4400 4400 4400 4450
Wire Wire Line
	4400 4450 4400 4600
Wire Wire Line
	7100 2450 7100 2050
Wire Wire Line
	7100 2050 7150 2050
Wire Wire Line
	7450 3500 7450 3450
Wire Wire Line
	7450 3000 7450 3050
Wire Wire Line
	8150 2800 8150 2850
Wire Wire Line
	8150 2950 8150 3000
Wire Wire Line
	7650 3250 8050 3250
Wire Wire Line
	7900 2800 7900 2850
Wire Wire Line
	7900 2950 7900 3000
Wire Wire Line
	6900 1850 6900 3150
Wire Wire Line
	6900 3150 6900 4100
Wire Wire Line
	6900 3150 7150 3150
Wire Wire Line
	7150 3350 7000 3350
Wire Wire Line
	7000 3350 7000 4300
Connection ~ 7000 4300
Connection ~ 6900 3150
Wire Wire Line
	7350 3450 7350 3700
Wire Wire Line
	7350 3700 8100 3700
Wire Wire Line
	8100 3550 8100 3500
Wire Wire Line
	8100 3650 8100 3700
Wire Wire Line
	8100 3700 8100 3750
Connection ~ 8100 3700
Wire Wire Line
	8100 3900 8100 3850
Wire Wire Line
	4350 3700 4350 3650
Connection ~ 4350 3650
Wire Wire Line
	4350 3500 4350 3550
Connection ~ 4350 3550
Wire Notes Line
	9100 2550 9100 4000
Wire Notes Line
	9100 4050 9100 5150
Wire Notes Line
	9100 5150 6800 5150
Wire Notes Line
	9100 2500 9100 1150
Wire Notes Line
	9100 1150 6800 1150
Wire Notes Line
	9100 4050 7850 4050
Wire Notes Line
	9100 4000 8200 4000
Wire Notes Line
	6800 5150 6800 4400
Wire Notes Line
	9100 2500 7000 2500
Wire Notes Line
	7000 2550 9100 2550
Wire Notes Line
	6800 1150 6800 4000
Wire Wire Line
	5450 1850 5500 1850
Wire Wire Line
	5500 1850 5650 1850
Wire Wire Line
	5650 1850 5800 1850
Wire Wire Line
	5800 1850 5900 1850
Wire Wire Line
	5900 1850 5900 1800
Wire Wire Line
	5800 1900 5800 1850
Connection ~ 5800 1850
Wire Wire Line
	5800 2050 5800 2000
Wire Wire Line
	5600 2150 5650 2150
Wire Wire Line
	5650 2150 5650 1850
Connection ~ 5650 1850
Wire Wire Line
	5450 2150 5500 2150
Wire Wire Line
	5500 2050 5450 2050
Wire Wire Line
	5500 1850 5500 1950
Wire Wire Line
	5500 1950 5500 2050
Connection ~ 5500 1850
Wire Wire Line
	5450 1950 5500 1950
Connection ~ 5500 1950
Wire Wire Line
	4800 2200 4800 2150
Wire Wire Line
	4650 2150 4800 2150
Wire Wire Line
	4800 2150 4850 2150
Wire Wire Line
	5000 4300 6000 4300
Wire Wire Line
	6000 4300 6400 4300
Wire Wire Line
	6400 4300 6600 4300
Wire Wire Line
	6600 4300 6900 4300
Wire Wire Line
	6900 4300 7000 4300
Wire Wire Line
	7000 4300 7100 4300
Wire Wire Line
	6900 4100 6500 4100
Wire Notes Line
	5600 5150 6750 5150
Wire Notes Line
	6750 5150 6750 4400
Wire Notes Line
	5600 3750 6750 3750
Wire Notes Line
	6750 3750 6750 4000
Wire Wire Line
	4650 2150 4650 2000
Connection ~ 4800 2150
Wire Wire Line
	4650 1850 4650 1900
Wire Wire Line
	4800 1850 4800 1950
Wire Wire Line
	4800 1950 4800 2050
Wire Wire Line
	4800 2050 4850 2050
Connection ~ 4800 1850
Wire Wire Line
	4800 1950 4850 1950
Connection ~ 4800 1950
Connection ~ 4650 1850
Wire Notes Line
	5600 3750 5600 4000
Wire Notes Line
	5600 4400 5600 5150
Wire Notes Line
	4400 2450 4400 1550
Wire Notes Line
	4400 1550 6100 1550
Wire Notes Line
	6100 1550 6100 2450
Wire Notes Line
	6100 2450 4400 2450
Wire Wire Line
	4550 1800 4550 1850
Wire Wire Line
	4550 1850 4650 1850
Wire Wire Line
	4650 1850 4800 1850
Wire Wire Line
	4800 1850 4850 1850
Wire Wire Line
	8150 3250 8250 3250
Wire Wire Line
	6000 4150 6000 4300
Connection ~ 6000 4300
Wire Wire Line
	6400 3900 6400 4000
Connection ~ 6000 3900
Wire Wire Line
	6000 3900 6000 4050
Wire Wire Line
	5850 3900 6400 3900
$Comp
L R R206
U 1 1 58590EF7
P 5850 3900
F 0 "R206" H 5900 3950 50  0000 C CNN
F 1 "100" H 5900 3850 50  0000 C CNN
F 2 "agg:0603" H 5850 3900 50  0001 C CNN
F 3 "" H 5850 3900 50  0001 C CNN
	1    5850 3900
	-1   0    0    1   
$EndComp
Wire Wire Line
	5650 3900 5750 3900
Wire Wire Line
	5650 4100 5650 3900
Wire Wire Line
	5000 4100 5650 4100
$EndSCHEMATC
