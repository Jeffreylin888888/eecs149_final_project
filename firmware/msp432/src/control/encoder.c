#include "board.h"
#include "core.h"
#include "encoder.h"
/* Two A edges/cycle: calibration must use this 2x count, not a 4x CPR value. */
static volatile uint32_t counts[2], overruns;
void encoder_init(void) {
    GPIO_setAsInputPin(GPIO_PORT_P5, BIT0|BIT2);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P10, BIT4|BIT5, GPIO_PRIMARY_MODULE_FUNCTION);
    TIMER_A3->CTL=TIMER_A_CTL_SSEL__SMCLK|TIMER_A_CTL_MC__CONTINUOUS|TIMER_A_CTL_CLR;
    TIMER_A3->CCTL[0]=TIMER_A_CCTLN_CM_3|TIMER_A_CCTLN_CAP|TIMER_A_CCTLN_SCS|TIMER_A_CCTLN_CCIE;
    TIMER_A3->CCTL[1]=TIMER_A3->CCTL[0];
    NVIC_EnableIRQ(TA3_0_IRQn); NVIC_EnableIRQ(TA3_N_IRQn);
}
static void capture(unsigned side, unsigned channel, unsigned bpin) {
    uint16_t status=TIMER_A3->CCTL[channel];
    (void)TIMER_A3->CCR[channel];
    TIMER_A3->CCTL[channel]&=~(TIMER_A_CCTLN_CCIFG|TIMER_A_CCTLN_COV);
    if (status&TIMER_A_CCTLN_COV) { ++overruns; return; }
    counts[side]+=(uint32_t)encoder_delta((status&TIMER_A_CCTLN_CCI)!=0,(P5->IN&bpin)!=0);
}
void TA3_0_IRQHandler(void) { capture(1,0,BIT0); }
void TA3_N_IRQHandler(void) { if (TIMER_A3->CCTL[1]&TIMER_A_CCTLN_CCIFG) capture(0,1,BIT2); }
void encoder_read(int32_t *l,int32_t *r,uint32_t *errors) {
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    *l=(int32_t)counts[0]; *r=(int32_t)counts[1]; *errors=overruns;
    __set_PRIMASK(mask);
}
