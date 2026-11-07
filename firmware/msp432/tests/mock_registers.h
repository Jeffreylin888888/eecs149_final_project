#include <stdbool.h>
#include <stdint.h>
#define BIT0 1u
#define BIT2 4u
#define BIT3 8u
#define BIT4 16u
#define BIT5 32u
#define BIT6 64u
#define BIT7 128u
typedef struct { uint8_t IN, OUT, DIR, SEL0, SEL1; } port;
typedef struct { uint16_t CTL, CCR[5], CCTL[5]; } timer;
extern port p1,p2,p3,p4,p5;
extern timer ta0;
#define P1 (&p1)
#define P2 (&p2)
#define P3 (&p3)
#define P4 (&p4)
#define P5 (&p5)
#define TIMER_A0 (&ta0)
#define TIMER_A_CCTLN_OUTMOD_7 0x00e0u
#define TIMER_A_CTL_SSEL__SMCLK 0x0200u
#define TIMER_A_CTL_MC__UP 0x0010u
#define TIMER_A_CTL_CLR 0x0004u
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)

#define __get_PRIMASK() 0u
#define __set_PRIMASK(x) ((void)(x))
