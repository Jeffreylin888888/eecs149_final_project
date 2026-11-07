#include "board.h"
#include "diagnostics.h"
#ifndef WHEEL_TEST
#define WHEEL_TEST 0
#endif
static volatile uint32_t ticks;
static volatile uint16_t stop_events;
/* Inspect with tools/status_msp432.py; masks: bit8=S2, low byte=P4 bumps. */
volatile uint16_t stop_inputs_now, stop_inputs_seen, abort_reason;
volatile uint32_t start_presses, motor_runs_started;
volatile uint32_t firmware_version = 2;
volatile diagnostic diagnostic_status;
volatile uint8_t encoder_levels;
void SysTick_Handler(void) {
    ++ticks;
#if WHEEL_TEST
    const uint16_t reasons = motor_stop_inputs();
    stop_inputs_now = reasons;
    stop_inputs_seen |= reasons;
    stop_events |= reasons;
    motor_tick();
#endif
}
static void fault_stop(void) {
    __disable_irq();
    motor_init();
    for (;;) { }
}
void HardFault_Handler(void) { fault_stop(); }
void NMI_Handler(void) { fault_stop(); }
int main(void) {
    motor_init();
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_3);
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_setReferenceOscillatorFrequency(CS_REFO_32KHZ);
    CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
#if WHEEL_TEST
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, BUMPS);
    GPIO_setAsInputPin(GPIO_PORT_P10, GPIO_PIN4 | GPIO_PIN5);
    GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN0 | GPIO_PIN2);
#endif
    WDT_A_initWatchdogTimer(WDT_A_CLOCKSOURCE_ACLK, WDT_A_CLOCKITERATIONS_8192);
    WDT_A_startTimer();
    SysTick_Config(3000);
    __enable_irq();
    diagnostic d = { WAIT_RELEASE, 0 };
    uint32_t previous = 0;
#if WHEEL_TEST
    bool start_was_down = false;
#endif
    for (;;) {
        const uint32_t now = ticks;
        if (now == previous) continue;
        previous = now;
        WDT_A_clearTimer();
        if ((now % 500) == 0) P1->OUT ^= BIT0;
#if WHEEL_TEST
        __disable_irq();
        const uint16_t reasons = stop_events | motor_stop_inputs();
        stop_events = 0;
        __enable_irq();
        const bool start_down = !(P1->IN & BIT1);
        if (start_down && !start_was_down) ++start_presses;
        start_was_down = start_down;
        const diag_state before = d.state;
        diagnostic_step(&d, now, start_down, reasons != 0);
        if (d.state != before) {
            motor_stop();
            if (d.state == ABORTED) abort_reason |= reasons;
            if (d.state == LEFT_RUN || d.state == RIGHT_RUN) {
                /* Capture any stop edge since the state-machine sample. */
                __disable_irq();
                const uint16_t late_stop = stop_events | motor_stop_inputs();
                if (late_stop) {
                    d.state = ABORTED;
                    abort_reason |= late_stop;
                } else if (motor_run(d.state == LEFT_RUN)) {
                    motor_runs_started |= d.state == LEFT_RUN ? 1u : 2u;
                } else {
                    d.state = ABORTED;
                    abort_reason |= 0x200u | motor_stop_inputs();
                }
                __enable_irq();
            }
        }
        /* RGB: red=inhibited/aborted, green=ready, blue=sequence, cyan=done.
         * Preserve P2.6/7 motor output latches. */
        uint8_t color = BIT0;
        if (!reasons && (d.state == WAIT_PRESS || d.state == WAIT_START_RELEASE)) color = BIT1;
        else if (d.state == LEFT_RUN || d.state == GAP || d.state == RIGHT_RUN) color = BIT2;
        else if (d.state == DONE) color = BIT1 | BIT2;
        P2->OUT = (P2->OUT & ~(BIT0 | BIT1 | BIT2)) | color;
        encoder_levels = ((P10->IN >> 4) & 3) | ((P5->IN & BIT0) << 2) | ((P5->IN & BIT2) << 1);
#endif
        diagnostic_status = d;
    }
}
/* TI startup copies .data and calls SystemInit. Freestanding C runtime clears .bss. */
extern uint32_t __bss_start__, __bss_end__;
void _mainCRTStartup(void) {
    for (uint32_t *p = &__bss_start__; p < &__bss_end__; ++p) *p = 0;
    (void)main();
    fault_stop();
}
