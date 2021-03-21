/* 
 * File:   UTIL.h
 * Author: George Ayman
 *
 * Created on August 9, 2020, 1:07 PM
 */

#ifndef UTIL_H
#define	UTIL_H


#define Delay_Ms(t) __delay_ms(t)
#define Delay_Us(t) __delay_us(t)
#define get_bit(reg,bitNum) ((reg>>bitNum)&1)
#define reset_bit(reg,bitNum) reg&=(~(1<<bitNum))
#define SET_BIT(reg,bitNum) reg|=(1<<bitNum)

#endif	/* UTIL_H */

