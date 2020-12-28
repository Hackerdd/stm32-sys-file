/***
	Copyright (c) 2019
	All rights reserved.

	File name           :sys_file.h
	file identification :
	Subject             :Say something about the file

	Current Version     :V2.0.0
	Author              :Hacker
	Date                :

	Instead Version     :
	Author              :Hacker
	Date                :
***/
#ifndef __SYS_FILE_H
#define __SYS_FILE_H value

#include <main.h>

#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
#define bit_read_write(data, bit) BIT_ADDR((unsigned long)(&data), bit)

#ifdef USE_STM32F1xx	

#define PA_L8_MODE(n)   BIT_ADDR(&(GPIOA->CRL),n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
#define PB_L8_MODE(n)   BIT_ADDR(&(GPIOB->CRL),n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
#define PC_L8_MODE(n)   BIT_ADDR(&(GPIOC->CRL),n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
#define PD_L8_MODE(n)   BIT_ADDR(&(GPIOD->CRL),n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
#define PE_L8_MODE(n)   BIT_ADDR(&(GPIOE->CRL),n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
#define PF_L8_MODE(n)   BIT_ADDR(&(GPIOF->CRL),n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
#define PH_L8_MODE(n)   BIT_ADDR(&(GPIOH->CRL),n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
#define PI_L8_MODE(n)   BIT_ADDR(&(GPIOI->CRL),n*4)  //1:OUTPUT MODE;0:INTPUT MODE 

#define PA_H8_MODE(n)   BIT_ADDR(&(GPIOA->CRH),n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
#define PB_H8_MODE(n)   BIT_ADDR(&(GPIOB->CRH),n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
#define PC_H8_MODE(n)   BIT_ADDR(&(GPIOC->CRH),n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
#define PD_H8_MODE(n)   BIT_ADDR(&(GPIOD->CRH),n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
#define PE_H8_MODE(n)   BIT_ADDR(&(GPIOE->CRH),n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
#define PF_H8_MODE(n)   BIT_ADDR(&(GPIOF->CRH),n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
#define PH_H8_MODE(n)   BIT_ADDR(&(GPIOH->CRH),n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
#define PI_H8_MODE(n)   BIT_ADDR(&(GPIOI->CRH),n*4)  //1:OUTPUT MODE;0:INTPUT MODE 

#else if USE_STM32F4xx

#define PAMODE(n)   BIT_ADDR(&(GPIOA->MODER),n*2)  //1:OUTPUT MODE;0:INTPUT MODE 
#define PBMODE(n)   BIT_ADDR(&(GPIOB->MODER),n*2)  //1:OUTPUT MODE;0:INTPUT MODE 
#define PCMODE(n)   BIT_ADDR(&(GPIOC->MODER),n*2)  //1:OUTPUT MODE;0:INTPUT MODE 
#define PDMODE(n)   BIT_ADDR(&(GPIOD->MODER),n*2)  //1:OUTPUT MODE;0:INTPUT MODE 
#define PEMODE(n)   BIT_ADDR(&(GPIOE->MODER),n*2)  //1:OUTPUT MODE;0:INTPUT MODE 
#define PFMODE(n)   BIT_ADDR(&(GPIOF->MODER),n*2)  //1:OUTPUT MODE;0:INTPUT MODE 
#define PHMODE(n)   BIT_ADDR(&(GPIOH->MODER),n*2)  //1:OUTPUT MODE;0:INTPUT MODE 
#define PIMODE(n)   BIT_ADDR(&(GPIOI->MODER),n*2)  //1:OUTPUT MODE;0:INTPUT MODE 

#endif
/**
 * [PAout description]
 * @param  n [0-15]
 * @return   [none]
 * EG:  PAout(1) = 1;//set PA1 to HIGH
 * 		PAout(1) = 0;//set PA1 to LOW
 */
#define PAout(n)   BIT_ADDR(&(GPIOA->ODR),n)  //OUTPUT 
#define PBout(n)   BIT_ADDR(&(GPIOB->ODR),n)  //OUTPUT 
#define PCout(n)   BIT_ADDR(&(GPIOC->ODR),n)  //OUTPUT 
#define PDout(n)   BIT_ADDR(&(GPIOD->ODR),n)  //OUTPUT 
#define PEout(n)   BIT_ADDR(&(GPIOE->ODR),n)  //OUTPUT 
#define PFout(n)   BIT_ADDR(&(GPIOF->ODR),n)  //OUTPUT 
#define PGout(n)   BIT_ADDR(&(GPIOG->ODR),n)  //OUTPUT 
#define PHout(n)   BIT_ADDR(&(GPIOH->ODR),n)  //OUTPUT 
#define PIout(n)   BIT_ADDR(&(GPIOI->ODR),n)  //OUTPUT

/**
 * [PAin description]
 * @param  n [0-15]
 * @return   [none]
 * EG:  a = PAin(1);//get PA1 bit to value a
 */
#define PAin(n)    BIT_ADDR(&(GPIOA->IDR),n)  //输入 
#define PBin(n)    BIT_ADDR(&(GPIOB->IDR),n)  //输入 
#define PCin(n)    BIT_ADDR(&(GPIOC->IDR),n)  //输入 
#define PDin(n)    BIT_ADDR(&(GPIOD->IDR),n)  //输入 
#define PEin(n)    BIT_ADDR(&(GPIOE->IDR),n)  //输入
#define PFin(n)    BIT_ADDR(&(GPIOF->IDR),n)  //输入
#define PGin(n)    BIT_ADDR(&(GPIOG->IDR),n)  //输入
#define PHin(n)    BIT_ADDR(&(GPIOH->IDR),n)  //输入
#define PIin(n)    BIT_ADDR(&(GPIOI->IDR),n)  //输入


#define HIGH 0x1
#define LOW  0x0

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352

//#define SERIAL  0x0
#define DISPLAY 0x1

#define LSBFIRST 0
#define MSBFIRST 1

#define CHANGE 1
#define FALLING 2
#define RISING 3


#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
// #define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<=(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

#define low8Byte(w) ((uint8_t) ((w) & 0xff))
#define high8Byte(w) ((uint8_t) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#define bitToggle(value, bit) bitWrite(value, bit, !bitRead(value, bit))

#define swap(a, b) { uint32_t t = a; a = b; b = t; }
#define bit(b) (1UL << (b))


#define map(x, in_min, in_max, out_min, out_max) \
  ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

#define cycle_increase(a, b) a=(a+1)%(b)

/* 向上取整 */
#define CEIL_DIV(a, b) (((a)+(b)-1)/(b))

#endif

