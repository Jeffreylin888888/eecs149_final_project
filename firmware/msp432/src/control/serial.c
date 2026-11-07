#include "board.h"
#include "serial.h"
static volatile uint8_t head,tail;
static volatile bool error;
static uint8_t rx[128];
void serial_init(void) {
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,BIT2|BIT3,GPIO_PRIMARY_MODULE_FUNCTION);
    const eUSCI_UART_ConfigV1 cfg={EUSCI_A_UART_CLOCKSOURCE_SMCLK,19,8,0x55,
        EUSCI_A_UART_NO_PARITY,EUSCI_A_UART_LSB_FIRST,EUSCI_A_UART_ONE_STOP_BIT,
        EUSCI_A_UART_MODE,EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,EUSCI_A_UART_8_BIT_LEN};
    UART_initModule(EUSCI_A0_BASE,&cfg); UART_enableModule(EUSCI_A0_BASE);
    UART_enableInterrupt(EUSCI_A0_BASE,EUSCI_A_UART_RECEIVE_INTERRUPT);
    NVIC_EnableIRQ(EUSCIA0_IRQn);
}
void EUSCIA0_IRQHandler(void) {
    uint16_t status=EUSCI_A0->STATW;
    uint8_t c=(uint8_t)EUSCI_A0->RXBUF;
    uint8_t next=(head+1u)&127u;
    if ((status&(EUSCI_A_STATW_OE|EUSCI_A_STATW_FE|EUSCI_A_STATW_PE)) || next==tail) { error=true; return; }
    rx[head]=c; head=next;
}
int serial_get(void) {
    if(tail==head) return -1;
    int c=rx[tail]; tail=(tail+1u)&127u; return c;
}
bool serial_error(void) {
    __disable_irq(); bool e=error; error=false; __enable_irq(); return e;
}
void serial_put(const char *s) {
    while(*s) { while(!(EUSCI_A0->IFG&EUSCI_A_IFG_TXIFG)) {} EUSCI_A0->TXBUF=*s++; }
}
void serial_number(int32_t n) {
    char b[12]; unsigned i=0; uint32_t u=n<0 ? 0u-(uint32_t)n : (uint32_t)n;
    do { b[i++]=(char)('0'+u%10); u/=10; } while(u);
    if(n<0) serial_put("-");
    while(i) { char s[2]={b[--i],0}; serial_put(s); }
}
