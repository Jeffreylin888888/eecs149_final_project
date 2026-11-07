#define BOARD_H
#include "mock_registers.h"
typedef struct { uint16_t CTL, CCR[5], CCTL[5], EX0; } Timer_A_Type;
extern Timer_A_Type ta1,ta2;
#define TIMER_A1 (&ta1)
#define TIMER_A2 (&ta2)
typedef struct { uint16_t KEYID; } pmap;
typedef struct { uint8_t PMAP_REGISTER4,PMAP_REGISTER5; } mapping;
extern pmap map;
extern mapping p2map,p3map;
#define PMAP (&map)
#define P2MAP (&p2map)
#define P3MAP (&p3map)
#define PMAP_KEYID_VAL 0x2d52
#define PMAP_TA1CCR1A 1
#define PMAP_TA1CCR2A 2
#define TIMER_A_CCTLN_CCIE 16
#define TIMER_A_CCTLN_CCIFG 1
#define TA1_0_IRQn 0
#define TA2_0_IRQn 1
#define NVIC_SetPriority(a,b) ((void)(a),(void)(b))
#define NVIC_EnableIRQ(a) ((void)(a))
