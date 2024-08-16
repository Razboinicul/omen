#include <omen/libraries/std/stddef.h>

// Read a byte from the serial port
static inline uint8_t inb(uint16_t port) {
    uint8_t data;
    __asm__ volatile ("inb %1, %0" : "=a" (data) : "Nd" (port));
    return data;
}

// Write a byte to the serial port
static inline void outb(uint16_t port, uint8_t data) {
    __asm__ volatile ("outb %0, %1" : : "a" (data), "Nd" (port));
}