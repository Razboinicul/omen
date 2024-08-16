#ifndef SERIAL_H
#define SERIAL_H

#include <sys/ioctl.h> // For IOCTL commands
#include <omen/libraries/std/stddef.h>
#include <omen/managers/dev/devices.h>
#include <omen/hal/arch/x86/io.c>

// Base addresses for COM ports
#define COM1_BASE 0x3F8
#define COM2_BASE 0x2F8
#define COM3_BASE 0x3E8
#define COM4_BASE 0x2E8

// Define IOCTL commands
#define SERIAL_IOCTL_FLUSH_OUTPUT _IO('S', 1)       // Flush output buffer
#define SERIAL_IOCTL_SET_BAUDRATE _IOW('S', 2, int) // Set baud rate

// Define port address range for serial communication
#define MIN_PORT_ADDRESS 0x3F8
#define MAX_PORT_ADDRESS 0x3FF

// Define maximum buffer size (e.g., 1KB)
#define BUFFER_MAX_SIZE 1024

// Define the serial port structure
typedef struct {
    uint16_t port;
} serial_port_t;

// File operations structure for serial devices
extern struct file_operations serial_fops;

// Function prototypes for serial port operations
size_t serial_read(uint64_t port_id, uint64_t size, uint64_t offset, uint8_t *buffer);
size_t serial_write(uint64_t port_id, uint64_t size, uint64_t offset, uint8_t *buffer);
int serial_ioctl(uint64_t port_id, uint64_t request, void *arg);

// Function to initialize and register the serial device
void init_serial_device(uint8_t com, uint16_t port_base);
void poll_and_register_serial_devices();

static inline uint8_t inb(uint16_t port);
static inline void outb(uint16_t port, uint8_t data);

#endif // SERIAL_H