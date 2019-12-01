/***
	Copyright (c) 2018
	All rights reserved.

	File name           :sys.h
	file identification :
	Subject             :bit band operation For STM32F4
                          add cycle_increase()

	Current Version     :V1.9.1
	Author              :Hacker
	Date                :2018.12.04

	Instead Version     :
	Author              :Hacker
	Date                :
***/

#ifndef __SYS_H
#define __SYS_H

	 
//For F4xx
#ifdef USE_STM32F4xx	 
	#include "stm32f4xx_hal.h" 

	#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
	#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
	#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
	#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
	#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
	#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
	#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
	#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
	#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014     

	#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
	#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
	#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
	#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
	#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
	#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
	#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
	#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
	#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 

	#define GPIOA_MODER_Addr    (GPIOA_BASE)  //0x40020000
	#define GPIOB_MODER_Addr    (GPIOB_BASE)
	#define GPIOC_MODER_Addr    (GPIOC_BASE)
	#define GPIOD_MODER_Addr    (GPIOD_BASE)
	#define GPIOE_MODER_Addr    (GPIOE_BASE)
	#define GPIOF_MODER_Addr    (GPIOF_BASE)
	#define GPIOG_MODER_Addr    (GPIOG_BASE)
	#define GPIOH_MODER_Addr    (GPIOH_BASE)
	#define GPIOI_MODER_Addr    (GPIOI_BASE)

/***************************************************/
	#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
	#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
	#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 



	#define PAMODE(n)   BIT_ADDR(GPIOA_MODER_Addr,n*2)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PBMODE(n)   BIT_ADDR(GPIOB_MODER_Addr,n*2)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PCMODE(n)   BIT_ADDR(GPIOC_MODER_Addr,n*2)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PDMODE(n)   BIT_ADDR(GPIOD_MODER_Addr,n*2)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PEMODE(n)   BIT_ADDR(GPIOE_MODER_Addr,n*2)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PFMODE(n)   BIT_ADDR(GPIOF_MODER_Addr,n*2)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PGMODE(n)   BIT_ADDR(GPIOG_MODER_Addr,n*2)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PHMODE(n)   BIT_ADDR(GPIOH_MODER_Addr,n*2)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PIMODE(n)   BIT_ADDR(GPIOI_MODER_Addr,n*2)  //1:OUTPUT MODE;0:INTPUT MODE 

	#define bit_read_write(data, bit) BIT_ADDR((unsigned long)(&data), bit)

	/**
	 * [PAout description]
	 * @param  n [0-15]
	 * @return   [none]
	 * EG:  PAout(1) = 1;//set PA1 to HIGH
	 * 		PAout(1) = 0;//set PA1 to LOW
	 */
	#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //OUTPUT 
	#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //OUTPUT 
	#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //OUTPUT 
	#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //OUTPUT 
	#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //OUTPUT 
	#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //OUTPUT 
	#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //OUTPUT 
	#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //OUTPUT 
	#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //OUTPUT

	/**
	 * [PAin description]
	 * @param  n [0-15]
	 * @return   [none]
	 * EG:  a = PAin(1);//set PA1 bit to value a
	 */
	#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 
	#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 
	#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 
	#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 
	#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入
	#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入
	#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入
	#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //输入
	#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //输入

#endif																	    

/***************************************************/


//For F1xx
#ifdef USE_STM32F1xx	 
	#include "stm32f1xx_hal.h" 

	#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
	#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
	#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
	#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
	#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
	#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
	#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

	#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
	#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
	#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
	#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
	#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
	#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
	#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 

	#define GPIOA_MODER_Addr    (GPIOA_BASE)  //0x40020000
	#define GPIOB_MODER_Addr    (GPIOB_BASE)
	#define GPIOC_MODER_Addr    (GPIOC_BASE)
	#define GPIOD_MODER_Addr    (GPIOD_BASE)
	#define GPIOE_MODER_Addr    (GPIOE_BASE)
	#define GPIOF_MODER_Addr    (GPIOF_BASE)
	#define GPIOG_MODER_Addr    (GPIOG_BASE)
	#define GPIOH_MODER_Addr    (GPIOH_BASE)
	#define GPIOI_MODER_Addr    (GPIOI_BASE)

	#define PA_LOW_MODE(n)   BIT_ADDR(GPIOA_MODER_Addr,n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PB_LOW_MODE(n)   BIT_ADDR(GPIOB_MODER_Addr,n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PC_LOW_MODE(n)   BIT_ADDR(GPIOC_MODER_Addr,n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PD_LOW_MODE(n)   BIT_ADDR(GPIOD_MODER_Addr,n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PE_LOW_MODE(n)   BIT_ADDR(GPIOE_MODER_Addr,n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PF_LOW_MODE(n)   BIT_ADDR(GPIOF_MODER_Addr,n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PG_LOW_MODE(n)   BIT_ADDR(GPIOG_MODER_Addr,n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PH_LOW_MODE(n)   BIT_ADDR(GPIOH_MODER_Addr,n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PI_LOW_MODE(n)   BIT_ADDR(GPIOI_MODER_Addr,n*4)  //1:OUTPUT MODE;0:INTPUT MODE 

	#define PA_HIGH_MODE(n)   BIT_ADDR(GPIOA_MODER_Addr+4,n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PB_HIGH_MODE(n)   BIT_ADDR(GPIOB_MODER_Addr+4,n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PC_HIGH_MODE(n)   BIT_ADDR(GPIOC_MODER_Addr+4,n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PD_HIGH_MODE(n)   BIT_ADDR(GPIOD_MODER_Addr+4,n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PE_HIGH_MODE(n)   BIT_ADDR(GPIOE_MODER_Addr+4,n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PF_HIGH_MODE(n)   BIT_ADDR(GPIOF_MODER_Addr+4,n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PG_HIGH_MODE(n)   BIT_ADDR(GPIOG_MODER_Addr+4,n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PH_HIGH_MODE(n)   BIT_ADDR(GPIOH_MODER_Addr+4,n*4)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PI_HIGH_MODE(n)   BIT_ADDR(GPIOI_MODER_Addr+4,n*4)  //1:OUTPUT MODE;0:INTPUT MODE 

	/***************************************************/
	
	#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
	#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
	#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 



	#define PAMODE(n)   BIT_ADDR(GPIOA_MODER_Addr,n*2)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PBMODE(n)   BIT_ADDR(GPIOB_MODER_Addr,n*2)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PCMODE(n)   BIT_ADDR(GPIOC_MODER_Addr,n*2)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PDMODE(n)   BIT_ADDR(GPIOD_MODER_Addr,n*2)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PEMODE(n)   BIT_ADDR(GPIOE_MODER_Addr,n*2)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PFMODE(n)   BIT_ADDR(GPIOF_MODER_Addr,n*2)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PGMODE(n)   BIT_ADDR(GPIOG_MODER_Addr,n*2)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PHMODE(n)   BIT_ADDR(GPIOH_MODER_Addr,n*2)  //1:OUTPUT MODE;0:INTPUT MODE 
	#define PIMODE(n)   BIT_ADDR(GPIOI_MODER_Addr,n*2)  //1:OUTPUT MODE;0:INTPUT MODE 

	#define bit_read_write(data, bit) BIT_ADDR((unsigned long)(&data), bit)

	/**
	 * [PAout description]
	 * @param  n [0-15]
	 * @return   [none]
	 * EG:  PAout(1) = 1;//set PA1 to HIGH
	 * 		PAout(1) = 0;//set PA1 to LOW
	 */
	#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //OUTPUT 
	#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //OUTPUT 
	#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //OUTPUT 
	#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //OUTPUT 
	#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //OUTPUT 
	#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //OUTPUT 
	#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //OUTPUT 
	#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //OUTPUT 
	#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //OUTPUT

	/**
	 * [PAin description]
	 * @param  n [0-15]
	 * @return   [none]
	 * EG:  a = PAin(1);//set PA1 bit to value a
	 */
	#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 
	#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 
	#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 
	#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 
	#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入
	#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入
	#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入
	#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //输入
	#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //输入

#endif																	    


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


// // undefine stdlib's abs if encountered
// #ifndef abs
// #undef abs
// #endif

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
// #define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<=(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

#define interrupts() sei()
#define noInterrupts() cli()

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#define bitToggle(value, bit) bitWrite(value, bit, !bitRead(value, bit))

#define swap(a, b) { uint8_t t = a; a = b; b = t; }
#define bit(b) (1UL << (b))


#define map(x, in_min, in_max, out_min, out_max) \
  ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

#define cycle_increase(a, b) a=(a+1)%(b)
    
typedef unsigned char u8;
typedef unsigned int u16;
typedef unsigned long u32;

#endif

