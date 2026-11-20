// Lab15_ADC_sound_main.c
// Runs on MSP432, RSLK1.1
// Test the operation of the microphone sensor
// by repeatedly taking measurements.  The
// results are sent through the UART to a computer running
// TExaSdisplay or another terminal program.
// Daniel Valvano
// May 27, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/
#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/TimerA1.h"
#include "../inc/UART0.h"
#include "../inc/LaunchPad.h"
#include "../inc/ADC14.h"
#include "../inc/BumpInt.h"
#include "../inc/SSD1306.h"
#include "../inc/Motor.h"
#include "../inc/LPF.h"

// MSP432 P8.2 (J5) (analog input to MSP432) connected to microphone circuit

//**************Program 15.1*******************
volatile int32_t ADCvalue;
volatile int32_t ADCflag;
#define NN 500  // 5ms window
volatile int32_t Raw,nn;
volatile int16_t Sound[NN];
#define DC (8192+51)    // calibrate this for DC offset
void Program15_1_ISR(void){  // runs at fs = 10 kHz
  P1->OUT ^= 0x01;           // profile
  P1->OUT ^= 0x01;           // profile
  ADCvalue = ADC_In23()-DC;  // sample P8.2/channel 23
  ADCflag = 1;               // semaphore
  P1->OUT ^= 0x01;           // profile
}


#define MM 16
int32_t Scale=0; // scale so all data fits in Histogram of size MM
int16_t Histogram[MM];
int32_t Min,Max,Range;
int32_t Sum;      // sum of data
int32_t Sum2;     // sum of (data-average)^2
int32_t Average;  // average of data = sum/N
int32_t Variance; // =sum2/(N-1)
int32_t Sigma;    // standard deviation = sqrt(Variance)
// Newton's method
// s is an integer
// sqrt(s) is an integer
uint32_t sqrt(uint32_t s){
uint32_t t;   // t*t will become s
int n;             // loop counter
  t = s/16+1;      // initial guess
  for(n = 16; n; --n){ // will finish
    t = ((t*t+s)/t)/2;
  }
  return t;
}

void Collect(void){int32_t n;
  for(n=0; n<256; n++){
    while(ADCflag == 0){};
    ADCflag = 0; // skip the first 256
  }
  Sum = Sum2 = 0;
  for(n=0; n<NN; n++){
    while(ADCflag == 0){};
    Sum = Sum+ADCvalue;            // 14bits*256 = 22 bits
    Sound[n] = ADCvalue;
    ADCflag = 0;
  }
  Average = Sum/NN;
}
void Statistics(void){int32_t n;
  for(n=0; n<NN; n++){
    Sum2 = Sum2+(Sound[n]-Average)*(Sound[n]-Average);
  }
  Variance = Sum2/(NN-1);
  Sigma = sqrt(Variance);
  Scale = 0;
  Min = 16384; Max = -16384;
  for(n=0; n<NN; n++){
    if(Sound[n] < Min)
      Min = Sound[n]; // smallest
    if(Sound[n] > Max)
      Max = Sound[n]; // largest
  }
  Range = Max-Min;
  while(Range >= (MM<<Scale)){
    Scale++;
  }
 // Min = (Min>>Scale)>>Scale; // round to make pretty
  for(n=0;n<MM;n++){
    Histogram[n] = 0;
  }
  for(n=0; n<NN; n++){
    int32_t j=(Sound[n]-Min)>>Scale;
    if(j >= MM) j = MM-1;
    Histogram[j]++;
  }
  UART0_OutString("Probability Mass Function (PMF)\n Data \tCount\n");
  for(n=0;n<MM;n++){
    if(Histogram[n]){
      UART0_OutUDec5((n<<Scale)+Min+DC); UART0_OutString(" \t");UART0_OutUDec(Histogram[n]);UART0_OutChar('\n');
    }
  }
  UART0_OutString("Statistics ");    UART0_OutUDec(NN); UART0_OutString(" samples, fs = 10 kHz\n");
  UART0_OutString("Average      ="); UART0_OutSDec(Average); UART0_OutChar('\n');
  UART0_OutString("Range        ="); UART0_OutUDec(Range); UART0_OutChar('\n');
  UART0_OutString("Variance     ="); UART0_OutUDec(Variance); UART0_OutChar('\n');
  UART0_OutString("Sigma        ="); UART0_OutUDec(Sigma); UART0_OutChar('\n');
}
int Program15_1(void){//Program15_1(void){ // example program 15.1, RSLK1.1
  Clock_Init48MHz();
  ADCflag = 0;
  ADC0_InitSWTriggerCh23();   // initialize ADC P8.2/channel23
  UART0_Init();               // initialize UART0 115,200 baud rate
  LaunchPad_Init();
  TimerA1_Init(&Program15_1_ISR,50);    // 10000 Hz sampling
  EnableInterrupts();
  while(1){
    UART0_OutString("\nProgram 15.1 noise test, no filter\nValvano May 2019, RSLK1.1\n10000Hz, 14-bit sampling\nConnect analog signal to P8.2\n");
    Collect();
    Statistics();
    while(LaunchPad_Input()==0x00){}; // wait for touch
    if(LaunchPad_Input()==0x01){ // button 1
      uint32_t n;
      for(n=0;n<NN;n++){
         UART0_OutSDec(Sound[n]);UART0_OutChar('\n');
      }
    }
    while(LaunchPad_Input()!=0x00){}; // wait for release
  }
}

//**************Program 15.2*******************
// fs = 10kHz

int32_t t0,t1,t2;  // last save inputs/32
int32_t Rxx2;        // autocorrelation
#define K 128 // how fast it responds
#define M 128

int32_t NoiseReject(int32_t x){
  t2 = t1; t1 = t0; t0 = x/32;
  Rxx2 = ((K-1)*Rxx2 + t0*t2)/K;
  if(Rxx2 < -M) Rxx2 = -M;
  if(Rxx2 >  M) Rxx2 =  M;
  return (Rxx2*x)/M;
}
void Program15_2_ISR(void){  // runs at fs=10kHz
  P1->OUT ^= 0x01;           // profile
  P1->OUT ^= 0x01;           // profile
  Raw = ADC_In23()-DC;          // sample P8.2
  ADCvalue = NoiseReject(Raw);  // improves SNR
  ADCflag = 1;               // semaphore
  P1->OUT ^= 0x01;           // profile
}


// Testing of noise reject filter, RSLK1.1
int Program15_2(void){ // example program 15.2
  Clock_Init48MHz();
  ADCflag = 0;
  ADC0_InitSWTriggerCh23();  // initialize ADC P8.2/channel23
  UART0_Init();              // initialize UART0 115,200 baud rate
  LaunchPad_Init();
  TimerA1_Init(&Program15_2_ISR,50);    // 10000 Hz sampling
  EnableInterrupts();
  while(1){
    UART0_OutString("\nProgram 15.2 Noise rejection test\nValvano May 2019\nConnect analog signal to P8.2\n");
    Collect();
    Statistics();
    while(LaunchPad_Input()==0x00){}; // wait for touch
    if(LaunchPad_Input()==0x01){ // button 1
      uint32_t n;
      for(n=0;n<NN;n++){
         UART0_OutSDec(Sound[n]);UART0_OutChar('\n');
      }
    }
    while(LaunchPad_Input()!=0x00){}; // wait for release

  }
}

void Lab15_ISR(void){   // runs at fs=10kHz
  int32_t filtered;
  P1->OUT ^= 0x01;       // profile
  P1->OUT ^= 0x01;       // profile
  Raw = ADC_In23()-DC;   // sample P8.2
  filtered = NoiseReject(Raw);  // improves SNR
  if(ADCflag == 0){
    Sound[nn] = filtered;
    if(nn>NN){
      ADCflag = 1;      // semaphore
    }else{
      nn++;
    }
  }
  P1->OUT ^= 0x01;          // profile
}

struct template{
  const int16_t *Wave;
  int32_t Size;
};
typedef const struct template template_t;
#define NUMPATTERNS 6
// Count = 242, Freq= 330 Hz
const int16_t T330[242]={2000, 1957, 1830, 1625, 1351, 1018, 642, 238, -176, -582, -964, -1304, -1588, -1804, -1943, -1999, -1969, -1855, -1661, -1396, -1072, -701, -300, 113, 522, 908, 1255, 1549, 1776, 1927, 1996, 1979, 1877, 1695, 1441, 1124, 760, 362, -50, -461, -852, -1206, -1509, -1747, -1910, -1991, -1987, -1898, -1728, -1483, -1176, -817, -424, -13, 399, 794, 1155, 1467, 1715, 1890, 1984, 1993, 1917, 1759, 1525, 1226, 874, 485, 75, -338, -736, -1103, -1423, -1682, -1869, -1975, -1997, -1934, -1788, -1565, -1275, -930, -546, -138, 276, 677, 1050, 1378, 1647, 1845, 1965, 2000, 1949, 1815, 1603, 1323, 985, 606, 201, -213, -618, -996, -1332, -1611, -1820, -1952, -2000, -1962, -1840, -1640, -1369, -1040, -666, -263, 151, 558, 941, 1285, 1573, 1793, 1937, 1998, 1973, 1864, 1675, 1414, 1093, 725, 325, -88, -497, -886, -1236, -1533, -1765, -1921, -1994, -1983, -1886, -1709, -1458, -1145, -783, -387, 25, 436, 829, 1186, 1492, 1734, 1902, 1989, 1990, 1906, 1740, 1500, 1196, 840, 449, 38, -375, -771, -1135, -1449, -1702, -1882, -1981, -1995, -1924, -1770, -1541, -1246, -897, -510, -100, 313, 713, 1082, 1405, 1668, 1860, 1971, 1999, 1940, 1799, 1580, 1294, 952, 570, 163, -251, -654, -1029, -1360, -1633, -1836, -1960, -2000, -1955, -1825, -1618, -1341, -1007, -630, -226, 188, 594, 975, 1313, 1596, 1810, 1946, 1999, 1967, 1850, 1654, 1387, 1061, 689, 288, -126, -534, -919, -1265, -1557, -1782, -1931, -1997, -1977, -1873, -1689, -1432, -1114, -748, -350, 63, 473, 863, 1216, 1517, 1753, 1913};
// Count = 182, Freq= 440 Hz
//const int16_t T440[182]={ -1458, -1995, -1912, -2266, -1931, -1998, -1388, -1160, -366, -218, 626, 609, 1185, 1131, 1764, 1639, 1899, 1566, 1548, 1115, 1030, 219, 37, -479, -679, -1353, -1312, -1718, -1529, -1873, -1606, -1757, -1014, -881, 86, 248, 1040, 1164, 1893, 1798, 2316, 2040, 2198, 1721, 1672, 1067, 1101, 480, 109, -651, -685, -1291, -1323, -1746, -1657, -1912, -1341, -1440, -798, -700, -46, 127, 987, 975, 1687, 1761, 2218, 1821, 1895, 1333, 1369, 805, 698, -161, -195, -807, -971, -1540, -1603, -2150, -1988, -2202, -1799, -1842, -996, -752, 117, 292, 1068, 1038, 1673, 1512, 1865, 1510, 1942, 1534, 1372, 698, 524, -75, -259, -958, -1074, -1684, -1595, -2186, -1933, -2148, -1644, -1434, -509, -151, 622, 763, 1453, 1519, 2040, 1904, 1984, 1749, 1894, 1367, 1521, 809, 626, -124, -336, -943, -1008, -1551, -1526, -1955, -1790, -2029, -1588, -1364, -659, -433, 427, 674, 1387, 1478, 1971, 1672, 1974, 1433, 1709, 1154, 1042, 285, 253, -433, -627, -1325, -1373, -1909, -1869, -2093, -1716, -1900, -1317, -1237, -465, -334, 545, 689, 1394, 1639, 2163, 1824, 2036, 1497, 1487, 859, 976, 331, 164, -578, -771, -1340, -1540, -2084};
const int16_t T440[182]={2000, 1924, 1702, 1351, 897, 375, -176, -713, -1196, -1588, -1860, -1990, -1969, -1799, -1492, -1072, -570, -25, 522, 1029, 1458, 1776, 1960, 1994, 1877, 1618, 1236, 760, 226, -325, -852, -1313, -1675, -1910, -1999, -1937, -1728, -1387, -941, -424, 126, 666, 1155, 1557, 1840, 1984, 1977, 1820, 1525, 1114, 618, 75, -473, -985, -1423, -1753, -1949, -1997, -1894, -1647, -1275, -806, -276, 276, 806, 1275, 1647, 1894, 1997, 1949, 1753, 1423, 985, 473, -75, -618, -1114, -1525, -1820, -1977, -1984, -1840, -1557, -1155, -666, -126, 424, 941, 1387, 1728, 1937, 1999, 1910, 1675, 1313, 852, 325, -226, -760, -1236, -1618, -1877, -1994, -1960, -1776, -1458, -1029, -522, 25, 570, 1072, 1492, 1799, 1969, 1990, 1860, 1588, 1196, 713, 176, -375, -897, -1351, -1702, -1924, -2000, -1924, -1702, -1351, -897, -375, 176, 713, 1196, 1588, 1860, 1990, 1969, 1799, 1492, 1072, 570, 25, -522, -1029, -1458, -1776, -1960, -1994, -1877, -1618, -1236, -760, -226, 325, 852, 1313, 1675, 1910, 1999, 1937, 1728, 1387, 941, 424, -126, -666, -1155, -1557, -1840, -1984, -1977, -1820, -1525, -1114, -618, -75, 473, 985, 1423, 1753, 1949};
// Count = 162, Freq= 494 Hz
const int16_t T494[162]={2000, 1904, 1627, 1194, 647, 38, -575, -1132, -1582, -1880, -1999, -1926, -1670, -1253, -718, -113, 502, 1070, 1535, 1853, 1994, 1945, 1710, 1311, 787, 188, -429, -1005, -1485, -1823, -1987, -1961, -1748, -1367, -856, -263, 355, 939, 1434, 1791, 1977, 1975, 1783, 1421, 924, 338, -281, -872, -1380, -1756, -1965, -1985, -1816, -1473, -990, -412, 206, 804, 1325, 1719, 1949, 1993, 1846, 1523, 1055, 485, -131, -734, -1267, -1679, -1931, -1998, -1874, -1571, -1118, -558, 55, 663, 1208, 1637, 1910, 2000, 1899, 1617, 1180, 630, 20, -592, -1147, -1593, -1886, -1999, -1921, -1660, -1240, -701, -95, 519, 1084, 1546, 1860, 1996, 1941, 1701, 1298, 771, 171, -446, -1020, -1497, -1830, -1989, -1958, -1739, -1354, -840, -246, 372, 955, 1446, 1799, 1980, 1972, 1775, 1409, 908, 320, -298, -888, -1393, -1765, -1968, -1983, -1809, -1461, -975, -394, 223, 820, 1338, 1728, 1953, 1991, 1839, 1512, 1040, 468, -148, -750, -1281, -1689, -1935, -1997, -1868, -1560, -1103, -541, 73, 680, 1222, 1647, 1915};
// Count = 153, Freq= 523 Hz
//const int16_t T523[153]={ 1159, 527, 272, -427, -641, -1477, -1429, -2017, -1823, -2338, -1818, -2116, -1585, -1421, -737, -480, 288, 541, 1202, 1406, 1963, 1618, 1753, 1465, 1612, 994, 924, 318, 113, -718, -997, -1668, -1588, -2274, -1984, -2393, -2110, -2330, -1712, -1508, -719, -563, 383, 762, 1465, 1375, 1700, 1480, 1849, 1352, 1378, 835, 1030, 344, -72, -890, -928, -1730, -1699, -2228, -2088, -2385, -1996, -2067, -1485, -1273, -507, -276, 429, 594, 1362, 1549, 1938, 1428, 1449, 1079, 1271, 711, 520, -191, -404, -983, -1117, -1870, -1753, -2343, -2019, -2092, -1716, -1898, -1158, -892, -159, 89, 1016, 1242, 1924, 1711, 2054, 1872, 2187, 1647, 1590, 1024, 925, 229, 55, -668, -789, -1445, -1408, -1839, -1580, -1776, -1340, -1300, -510, -366, 417, 770, 1686, 1722, 2348, 2290, 2620, 2231, 2326, 2012, 2151, 1613, 1453, 651, 440, -518, -616, -1193, -1033, -1527, -1140, -1473, -1189, -1166, -386, -148, 734, 953, 1675, 1732, 2354, 2024, 2246, 2001, 2032 };
const int16_t T523[153]={2000, 1893, 1583, 1104, 507, -144, -780, -1333, -1743, -1966, -1979, -1780, -1391, -853, -223, 430, 1037, 1534, 1866, 1998, 1917, 1630, 1169, 583, -65, -707, -1273, -1703, -1950, -1989, -1815, -1447, -924, -302, 353, 969, 1482, 1836, 1994, 1938, 1675, 1233, 659, 14, -632, -1211, -1660, -1931, -1996, -1847, -1500, -993, -380, 274, 899, 1427, 1803, 1986, 1956, 1717, 1294, 733, 93, -557, -1147, -1614, -1909, -1999, -1876, -1551, -1061, -457, 196, 828, 1371, 1768, 1975, 1971, 1756, 1353, 806, 172, -480, -1081, -1566, -1884, -2000, -1902, -1600, -1127, -534, 117, 755, 1312, 1729, 1961, 1983, 1793, 1411, 878, 251, -403, -1014, -1516, -1856, -1997, -1925, -1646, -1192, -610, 38, 681, 1251, 1688, 1944, 1992, 1826, 1466, 948, 329, -325, -945, -1463, -1825, -1991, -1945, -1690, -1254, -685, -41, 606, 1189, 1644, 1924, 1997, 1857, 1518, 1017, 407, -247, -874, -1408, -1791, -1982, -1962, -1731, -1315, -758, -121, 530, 1124, 1598, 1901};
// Count = 144, Freq= 554 Hz
const int16_t T554[144]={2000, 1880, 1535, 1005, 355, -338, -990, -1523, -1874, -2000, -1886, -1546, -1020, -372, 320, 975, 1512, 1868, 2000, 1892, 1557, 1035, 390, -303, -959, -1500, -1861, -1999, -1897, -1568, -1050, -407, 286, 944, 1489, 1855, 1999, 1903, 1579, 1065, 424, -268, -928, -1477, -1848, -1998, -1908, -1590, -1080, -441, 251, 912, 1465, 1841, 1997, 1913, 1600, 1095, 458, -233, -897, -1453, -1835, -1996, -1918, -1611, -1110, -475, 216, 881, 1441, 1827, 1995, 1923, 1621, 1124, 493, -198, -865, -1428, -1820, -1994, -1928, -1631, -1139, -510, 181, 849, 1416, 1813, 1992, 1933, 1641, 1153, 527, -163, -833, -1404, -1805, -1991, -1937, -1651, -1167, -543, 146, 817, 1391, 1798, 1989, 1941, 1661, 1182, 560, -128, -801, -1378, -1790, -1987, -1946, -1671, -1196, -577, 111, 785, 1365, 1782, 1985, 1950, 1681, 1210, 594, -93, -769, -1353, -1774, -1983, -1953, -1690, -1224, -611, 75, 753, 1340, 1766};
// Count = 121, Freq= 659 Hz
//const int16_t T659[121]={1737, 1794, 1944, 1280, 1130, 246, 3, -848, -921, -1415, -1358, -1777, -1366, -1096, -73, 256, 1148, 1344, 1951, 1645, 1564, 819, 642, -47, -340, -1162, -1283, -1851, -1503, -1676, -921, -607, 384, 636, 1517, 1696, 2038, 1548, 1292, 518, 179, -636, -921, -1713, -1587, -1773, -1260, -1199, -333, 71, 983, 1220, 2004, 1677, 1713, 1035, 786, -91, -179, -948, -982, -1571, -1333, -1456, -721, -417, 445, 702, 1452, 1437, 1855, 1667, 1565, 790, 590, -290, -473, -1231, -1438, -1981, -1483, -1330, -402, -88, 822, 1062, 1741, 1599, 1844, 1214, 1045, 290, -57, -1001, -1239, -2060, -1987, -2146, -1485, -1445, -146, 434, 1207, 1270, 1597, 1147, 1068, 289, 349, -296, -703, -1508, -1530, -2138, -1776, -1762, -1094, -582, 558, 750, 1430 };
const int16_t T659[121]={2000, 1831, 1353, 645, -171, -958, -1583, -1941, -1971, -1667, -1082, -314, 507, 1243, 1768, 1995, 1884, 1455, 780, -26, -829, -1491, -1901, -1990, -1743, -1201, -456, 366, 1126, 1696, 1979, 1928, 1551, 911, 118, -695, -1391, -1852, -1999, -1809, -1313, -595, 223, 1004, 1615, 1953, 1961, 1638, 1037, 262, -558, -1284, -1792, -1998, -1866, -1419, -732, 79, 876, 1526, 1917, 1984, 1716, 1158, 404, -418, -1169, -1723, -1986, -1913, -1517, -864, -65, 744, 1428, 1871, 1997, 1786, 1273, 545, -276, -1049, -1646, -1964, -1950, -1607, -992, -209, 608, 1324, 1815, 2000, 1846, 1381, 682, -132, -924, -1559, -1931, -1977, -1689, -1115, -353, 469, 1212, 1750, 1992, 1897, 1482, 816, 13, -793, -1465, -1889, -1994, -1762, -1232, -494, 328, 1094, 1675};
// Count = 108, Freq=  740 Hz
const int16_t T740[108]={2000, 1788, 1196, 350, -570, -1369, -1877, -1987, -1675, -1007, -126, 783, 1525, 1943, 1949, 1541, 806, -100, -985, -1661, -1984, -1886, -1387, -594, 325, 1176, 1776, 2000, 1799, 1216, 375, -546, -1351, -1869, -1990, -1689, -1029, -151, 760, 1509, 1937, 1955, 1557, 829, -75, -964, -1647, -1981, -1894, -1405, -618, 300, 1155, 1765, 1999, 1810, 1236, 399, -522, -1332, -1860, -1992, -1702, -1050, -176, 736, 1492, 1931, 1960, 1573, 852, -50, -941, -1633, -1977, -1902, -1423, -642, 276, 1135, 1753, 1999, 1820, 1255, 424, -497, -1313, -1850, -1994, -1715, -1072, -201, 713, 1475, 1924, 1965, 1588, 874, -25, -919, -1618, -1973, -1910, -1441, -666, 251, 1114, 1740};
// Count = 102, Freq= 784 Hz
//const int16_t T784[102]={-2196, -1585, -1218, -110, 397, 1225, 1347, 1851, 1336, 1219, 488, 398, -390, -570, -1264, -1446, -1966, -1647, -1585, -757, -489, 515, 820, 1698, 1584, 1739, 1267, 1179, 503, 271, -489, -685, -1320, -1219, -1626, -1164, -1004, -136, 137, 1063, 1346, 2054, 1742, 1642, 869, 758, -7, -122, -850, -870, -1447, -1231, -1316, -622, -323, 549, 802, 1542, 1712, 2017, 1443, 1291, 606, 390, -499, -670, -1376, -1415, -1829, -1458, -1511, -653, -313, 656, 900, 1737, 1584, 1811, 1112, 746, -246, -391, -935, -1086, -1738, -1424, -1734, -1123, -831, 75, 312, 1122, 1170, 1639, 1035, 1176, 440, 239, -645, -818, -1607, -1613};
const int16_t T784[102]={2000, 1762, 1105, 186, -778, -1557, -1966, -1907, -1395, -551, 424, 1298, 1863, 1985, 1636, 897, -55, -994, -1697, -1996, -1820, -1212, -315, 656, 1472, 1937, 1942, 1485, 675, -295, -1196, -1812, -1997, -1707, -1012, -75, 879, 1624, 1983, 1870, 1313, 444, -531, -1380, -1901, -1969, -1569, -797, 166, 1089, 1753, 2000, 1772, 1122, 206, -760, -1544, -1962, -1913, -1409, -570, 404, 1283, 1856, 1988, 1647, 915, -35, -977, -1686, -1994, -1828, -1228, -335, 637, 1458, 1932, 1947, 1499, 694, -276, -1180, -1803, -1998, -1718, -1029, -95, 861, 1612, 1980, 1877, 1328, 463, -512, -1365, -1894, -1973, -1582, -815, 146, 1072, 1743};
// Count = 91, Freq= 880 Hz
//const int16_t T880[91]={523, 1038, 2158, 2307, 2305, 1509, 1035, -380, -939, -2127, -2595, -2698, -1935, -1456, -240, 385, 1502, 1841, 2200, 1589, 1151, -139, -751, -1925, -2504, -3093, -2456, -2093, -653, 64, 1279, 1545, 2281, 1773, 1588, 661, 141, -911, -1626, -2659, -2393, -2134, -990, -316, 994, 1511, 2579, 2474, 2320, 1109, 368, -845, -1305, -2446, -2325, -2336, -1535, -848, 634, 1217, 2297, 2528, 2479, 1792, 1277, 18, -653, -1937, -2434, -2726, -1988, -1135, 310, 847, 1825, 2186, 2627, 2087, 1607, 308, -431, -1537, -2085, -2718, -2280, -1859, -453, 226, 1511, 1990, 2881};
const int16_t T880[91]={2000, 1702, 897, -176, -1196, -1860, -1969, -1492, -570, 522, 1458, 1960, 1877, 1236, 226, -852, -1675, -1999, -1728, -941, 126, 1155, 1840, 1977, 1525, 618, -473, -1423, -1949, -1894, -1275, -276, 806, 1647, 1997, 1753, 985, -75, -1114, -1820, -1984, -1557, -666, 424, 1387, 1937, 1910, 1313, 325, -760, -1618, -1994, -1776, -1029, 25, 1072, 1799, 1990, 1588, 713, -375, -1351, -1924, -1924, -1351, -375, 713, 1588, 1990, 1799, 1072, 25, -1029, -1776, -1994, -1618, -760, 325, 1313, 1910, 1937, 1387, 424, -666, -1557, -1984, -1820, -1114, -75, 985, 1753};
// Count = 76, Freq= 1047 Hz
//const int16_t T1047[76]={1983, 1285, 1172, 48, -402, -1249, -1290, -1393, -502, -6, 927, 1118, 1736, 1276, 1159, 411, -233, -922, -1088, -1759, -1458, -1066, -112, 312, 1135, 1076, 1388, 715, 309, -803, -1094, -1917, -1402, -1332, -180, 186, 1027, 1248, 1661, 1028, 908, -80, -582, -1343, -1231, -1635, -1038, -647, 220, 640, 1415, 1335, 1330, 591, 172, -791, -1080, -1807, -1500, -1126, 47, 586, 1444, 1413, 1712, 1009, 765, -345, -792, -1443, -1191, -1441, -358, -133, 827, 1192};
const int16_t T1047[76]={2000, 1583, 505, -784, -1745, -1978, -1385, -214, 1046, 1870, 1913, 1158, -80, -1285, -1954, -1807, -906, 374, 1497, 1995, 1661, 634, -659, -1676, -1994, -1479, -348, 929, 1818, 1948, 1265, 54, -1180, -1921, -1860, -1023, 241, 1404, 1982, 1732, 760, -530, -1599, -2000, -1566, -479, 808, 1758, 1974, 1366, 188, -1068, -1879, -1906, -1137, 107, 1306, 1959, 1796, 882, -399, -1514, -1997, -1646, -608, 683, 1690, 1991, 1461, 322, -952, -1829, -1942, -1245, -28, 1201};
// Count = 68, Freq= 1175 Hz
//const int16_t T1175[68]={ -1361, -2531, -2360, -1542, 310, 1265, 2234, 2478, 2194, 711, -239, -1559, -1911, -2221, -755, 491, 1741, 2096, 2768, 1890, 891, -870, -1478, -2444, -1820, -721, 1012, 1555, 2468, 2222, 1552, -254, -1084, -2130, -2548, -1964, -198, 896, 2314, 2545, 2449, 977, -189, -1594, -2118, -2719, -1608, -100, 1487, 1805, 2673, 1834, 776, -828, -1476, -2739, -2317, -1510, 123, 1217, 2211, 2398, 2116, 560, -607, -1832, -2187, -2412};
const int16_t T1175[68]={ 2000, 1479, 188, -1201, -1965, -1705, -558, 880, 1860, 1871, 908, -528, -1689, -1970, -1226, 157, 1458, 2000, 1500, 219, -1176, -1958, -1721, -588, 852, 1848, 1882, 936, -497, -1672, -1975, -1250, 126, 1436, 1999, 1521, 251, -1150, -1952, -1737, -618, 823, 1836, 1892, 964, -467, -1654, -1980, -1275, 94, 1414, 1998, 1541, 282, -1124, -1945, -1753, -648, 794, 1823, 1902, 991, -436, -1636, -1984, -1299, 63, 1392};
template_t Patterns[NUMPATTERNS]={
  {T330,242},  // forward
  {T440,182},  // backward
  {T494,162},  // left
  {T554,144},  // left
  {T659,121},  // right
  {T740,108}   // right
//  {T880,91},
//  {T1047,76},
//  {T1175,68}
};
int32_t RxyBuf[NUMPATTERNS];
// assumes x[] and y[] have zero or approximately zero averages
// assumes nx>ny
int32_t Rxy(volatile int16_t *x, int32_t nx, const int16_t *y, int32_t ny){
  int32_t rxy,maxRxy = 0;
  int32_t sum; uint32_t i,j;
  for(i=0; i<nx-ny; i++){
    sum = 0;
    for(j=0; j<ny; j++){
      sum += x[i+j]*y[j];
    }
    rxy = sum/ny;
    if(rxy>maxRxy) maxRxy=rxy;
  }
  return maxRxy/10000;
}
#define THRESHOLD 100

int main1(void){int32_t rxy,match;
  Clock_Init48MHz();
  ADCflag = 0;
  ADC0_InitSWTriggerCh23();  // initialize ADC P8.2/channel23
  UART0_Init();              // initialize UART0 115,200 baud rate
  LaunchPad_Init();
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_SetCursor(0,0); SSD1306_OutString("Lab 15 Microphone");
  SSD1306_SetCursor(0,1); SSD1306_OutString(" 330:");
  SSD1306_SetCursor(0,2); SSD1306_OutString(" 440:");
  SSD1306_SetCursor(0,3); SSD1306_OutString(" 659:");
  SSD1306_SetCursor(0,4); SSD1306_OutString(" 880:");
  SSD1306_SetCursor(0,5); SSD1306_OutString(" 1175");

  TimerA1_Init(&Lab15_ISR,50);    // 10000 Hz sampling
  UART0_OutString("Audio tone detection, RSLK1.1\nValvano May 2019\nConnect analog signal to P8.2\n");
  EnableInterrupts();
  while(1){
    while(ADCflag == 0){};
    // find pitch
    match = -1;
    for(int i=0; i<NUMPATTERNS; i++){
      rxy = Rxy(Sound,NN,Patterns[i].Wave,Patterns[i].Size);
      RxyBuf[i] = rxy;
      SSD1306_SetCursor(6,i+1); SSD1306_OutSDec(rxy);
      if(rxy > THRESHOLD){
        match = i;
      }
    }
    match = Median(match); // 3-wide median, discards out-lier;
    if(match>=0){
      SSD1306_SetCursor(0,match+1); SSD1306_OutChar('*');
    }else{
      for(int i=0; i<NUMPATTERNS; i++){
        SSD1306_SetCursor(0,i+1); SSD1306_OutChar(' ');
      }
    }
    nn = 0;
    ADCflag = 0; // show every 2000th point
  }
}
uint8_t CollisionData, CollisionFlag;  // mailbox
void HandleCollision(uint8_t bumpSensor){
   Motor_Stop();
   CollisionData = bumpSensor;
   CollisionFlag = 1;
}
#define POWER 3000
int main(void){int32_t rxy,match,lastmatch;
  Clock_Init48MHz();
  ADCflag = 0;
  ADC0_InitSWTriggerCh23();  // initialize ADC P8.2/channel23
  UART0_Init();              // initialize UART0 115,200 baud rate
  LaunchPad_Init();
  BumpInt_Init(&HandleCollision);
  Motor_Init();
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_SetCursor(0,0); SSD1306_OutString("Lab 15 Microphone");
  SSD1306_SetCursor(0,1); SSD1306_OutString(" 330:");
  SSD1306_SetCursor(0,2); SSD1306_OutString(" 440:");
  SSD1306_SetCursor(0,3); SSD1306_OutString(" 659:");
  SSD1306_SetCursor(0,4); SSD1306_OutString(" 880:");
  SSD1306_SetCursor(0,5); SSD1306_OutString(" 1175");

  TimerA1_Init(&Lab15_ISR,50);    // 10000 Hz sampling
  UART0_OutString("Audio tone detection, RSLK1.1\nValvano May 2019\nConnect analog signal to P8.2\n");
  EnableInterrupts();
  lastmatch = -1;
  while(1){
    while(ADCflag == 0){};
    // find pitch
    match = -1;
    for(int i=0; i<NUMPATTERNS; i++){
      rxy =Rxy(Sound,NN,Patterns[i].Wave,Patterns[i].Size);
      RxyBuf[i] = rxy;
      SSD1306_SetCursor(6,i+1); SSD1306_OutSDec(rxy);
      if(rxy > THRESHOLD){
        match = i;
      }
    }
    match = Median(match); // 3-wide median, discards out-lier;
    if((match==0)&&(lastmatch!= 0)){
      SSD1306_SetCursor(0,1); SSD1306_OutChar('*');
      Motor_Forward(POWER,POWER);
    }else if((match==1)&&(lastmatch!= 1)){
      SSD1306_SetCursor(0,2); SSD1306_OutChar('*');
      Motor_Backward(POWER,POWER);
    }else if((match==2)&&(lastmatch!= 2)){
      SSD1306_SetCursor(0,3); SSD1306_OutChar('*');
      Motor_Left(POWER,POWER);
    }else if((match==3)&&(lastmatch!= 3)){
      SSD1306_SetCursor(0,4); SSD1306_OutChar('*');
      Motor_Left(POWER,POWER);
    }else if((match==4)&&(lastmatch!= 4)){
      SSD1306_SetCursor(0,4); SSD1306_OutChar('*');
      Motor_Right(POWER,POWER);
    }else if((match==5)&&(lastmatch!= 5)){
      SSD1306_SetCursor(0,4); SSD1306_OutChar('*');
      Motor_Right(POWER,POWER);
    }else if((match==-1)&&(lastmatch!= -1)){
      Motor_Stop();
      for(int i=0; i<NUMPATTERNS; i++){
        SSD1306_SetCursor(0,i+1); SSD1306_OutChar(' ');
      }
    }
    lastmatch = match;
    nn = 0;
    ADCflag = 0; // show every 2000th point
  }
}
