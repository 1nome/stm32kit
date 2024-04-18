/**
  ******************************************************************************
  * @file     example_09-donut2.c
  * @author   a1k0n, Adam Wiszczor
  * @version  2.0
  * @date     26-February-2024 [v2.0]
  * @brief    Optimalizovany donut bez floatu, pouze port kodu od tvurce
  *
  * !!! PROZATIM FUNKCNI JEN PRO F407 !!!
  *
  ******************************************************************************
  * @attention
  *
  * Otestovano na: F407
  *
  * Netestovano: F401, F411, G071, L152
  *
  ******************************************************************************
*/

#include "stm32_kit.h"
#include "stm32_kit/uart.h"
#include "stm32_kit/overclock.h"
#include <stdio.h>
#include <string.h>

BOARD_SETUP void setup(void) {
	//clock168Mhz();
	UART_setup();
};


#define R(mul,shift,x,y) \
  _=x; \
  x -= mul*y>>shift; \
  y += mul*_>>shift; \
  _ = 3145728-x*x-y*y>>11; \
  x = x*_>>10; \
  y = y*_>>10;

int8_t b[1760], z[1760];
unsigned int count = 0;
char counter[10];

int main() {
	UART_write("\x1b[2J",4);
  int sA=1024,cA=0,sB=1024,cB=0,_;
  for (;;) {
	snprintf(counter, 10, "%8d\n", count++);
	UART_write(counter, 9);
    memset(b, 32, 1760);  // text buffer
    memset(z, 127, 1760);   // z buffer
    int sj=0, cj=1024;
    for (int j = 0; j < 90; j++) {
      int si = 0, ci = 1024;  // sine and cosine of angle i
      for (int i = 0; i < 324; i++) {
        int R1 = 1, R2 = 2048, K2 = 5120*1024;

        int x0 = R1*cj + R2,
            x1 = ci*x0 >> 10,
            x2 = cA*sj >> 10,
            x3 = si*x0 >> 10,
            x4 = R1*x2 - (sA*x3 >> 10),
            x5 = sA*sj >> 10,
            x6 = K2 + R1*1024*x5 + cA*x3,
            x7 = cj*si >> 10,
            x = 40 + 30*(cB*x1 - sB*x4)/x6,
            y = 12 + 15*(cB*x4 + sB*x1)/x6,
            N = (-cA*x7 - cB*((-sA*x7>>10) + x2) - ci*(cj*sB >> 10) >> 10) - x5 >> 7;

        int o = x + 80 * y;
        int8_t zz = (x6-K2)>>15;
        if (22 > y && y > 0 && x > 0 && 80 > x && zz < z[o]) {
          z[o] = zz;
          b[o] = ".,-~:;=!*#$@"[N > 0 ? N : 0];
        }
        R(5, 8, ci, si)  // rotate i
      }
      R(9, 7, cj, sj)  // rotate j
    }
    UART_write("\x1b[d",3);
    for (int k = 0; 1761 > k; k++){
		UART_putc(k % 80 ? b[k] : '\r');
		if(!(k%80)){
			UART_putc('\n');
		}
	}
    R(5, 7, cA, sA);
    R(5, 8, cB, sB);
  }
}

