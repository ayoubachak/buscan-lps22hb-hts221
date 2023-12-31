/*
 * util.h
 *
 *  Created on: Dec 12, 2018
 *      Author: kerhoas
 */

#ifndef INC_UTIL_H_
#define INC_UTIL_H_

#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};

#include "main.h"

void num2str(char *s, unsigned int number, unsigned int base, unsigned int size, int sp);
unsigned int str2num(char *s, unsigned base);
void reverse(char *str, int len);
int intToStr(int x, char str[], int d);
void float2str( char *res, float n, int afterpoint);
double myPow(double x, int n) ;
void flush_ch(char* ch, int ch_size);
int size_ch(char* ch, int ch_size_max);


#endif /* INC_UTIL_H_ */
