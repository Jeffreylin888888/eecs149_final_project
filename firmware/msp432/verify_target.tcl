# Register definitions and revision classification: OpenOCD 0.12.0 msp432.c.
proc verify_msp432_target {} {
    set device [expr {[lindex [read_memory 0x0020100c 32 1] 0] & 0xffff}]
    set revision [expr {[lindex [read_memory 0x00201010 32 1] 0] & 0xff}]
    set flashsize [lindex [read_memory 0xe0043020 32 1] 0]
    if {$device < 0xa000 || $device > 0xa005 || $revision < 0x43 || $revision > 0x49 || $flashsize != 0x40000} {
        error "Refusing write: not a supported production MSP432P401x with 256 KiB main flash"
    }
    echo [format "Target guard: device=%04x silicon=%02x main_flash=%d" $device $revision $flashsize]
}
