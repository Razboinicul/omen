#include "serial.h"

// Define the file operations structure for the serial device
struct file_operations serial_fops = {
    .read = (size_t (*)(uint64_t, uint64_t, uint8_t *))serial_read,
    .write = (size_t (*)(uint64_t, uint64_t, uint8_t *))serial_write,
    .ioctl = (int (*)(uint64_t, uint64_t, void *))serial_ioctl
};

// Define and initialize the serial device structure
static serial_port_t serial_device = {
    .port = COM1_BASE // Use the base address for COM1
};

// Initialize the serial device and register it
void init_serial_device(uint8_t com, uint16_t port_base) {
    // Set the port address for the serial device
    serial_device.port = port_base;

    // Register the serial device with the given major number
    status_t status = register_char(com, "serial", &serial_fops);
    if (status != SUCCESS) {
        kprintf("Failed to register serial device with major number %d\n", com);
        return;
    }

    // Optionally create the device entry
    char *device_name = device_create(&serial_device, com, 0);
    if (device_name == NULL) {
        kprintf("Failed to create device entry for serial device\n");
        return;
    }

    // Verify that the device was created and is valid
    device_t *device = device_search(device_name);
    if (device == NULL || !device->valid) {
        kprintf("Error: Device %s was not created successfully\n", device_name);
        return;
    }

    kprintf("Serial device %s initialized and registered successfully\n", device_name);
}

void poll_and_register_serial_devices() {
    uint16_t possible_ports[] = {COM1_BASE, COM2_BASE, COM3_BASE, COM4_BASE};
    uint8_t num_ports = sizeof(possible_ports) / sizeof(possible_ports[0]);

    for (uint8_t i = 0; i < num_ports; i++) {
        uint16_t port_base = possible_ports[i];

        // Initialize and register the serial device
        init_serial_device(possible_ports[i], port_base);  // Assigning major numbers starting from 1
    }
}

// Write data to the serial port
size_t serial_write(uint64_t port_id, uint64_t size, uint64_t offset, uint8_t *buffer) {
    // Sanity checks
    if (buffer == NULL) {
        kprintf("Error: Buffer is NULL\n");
        return 0; // or an appropriate error code
    }

    if (size == 0) {
        kprintf("Error: Size is zero\n");
        return 0; // or an appropriate error code
    }

    if (offset + size > BUFFER_MAX_SIZE) { // Replace BUFFER_MAX_SIZE with actual buffer size if needed
        kprintf("Error: Size exceeds buffer bounds\n");
        return 0; // or an appropriate error code
    }

    serial_port_t *serial = (serial_port_t *)port_id;

    // Additional sanity check for port address (example range check)
    if (serial->port < MIN_PORT_ADDRESS || serial->port > MAX_PORT_ADDRESS) { // Define MIN_PORT_ADDRESS and MAX_PORT_ADDRESS
        kprintf("Error: Invalid port address 0x%x\n", serial->port);
        return 0; // or an appropriate error code
    }

    size_t written = 0;

    while (written < size) {
        // Wait for THRE (Transmit Holding Register Empty) bit
        while ((inb(serial->port + 5) & 0x20) == 0) { }

        // Write byte to the serial port
        outb(serial->port, buffer[written]);
        written++;
    }

    return written;
}

// Read data from the serial port
size_t serial_read(uint64_t port_id, uint64_t size, uint64_t offset, uint8_t *buffer) {
    // Sanity checks
    if (buffer == NULL) {
        kprintf("Error: Buffer is NULL\n");
        return 0; // or an appropriate error code
    }

    if (size == 0) {
        kprintf("Error: Size is zero\n");
        return 0; // or an appropriate error code
    }

    if (offset + size > BUFFER_MAX_SIZE) { // Replace BUFFER_MAX_SIZE with actual buffer size if needed
        kprintf("Error: Size exceeds buffer bounds\n");
        return 0; // or an appropriate error code
    }

    serial_port_t *serial = (serial_port_t *)port_id;

    // Additional sanity check for port address (example range check)
    if (serial->port < MIN_PORT_ADDRESS || serial->port > MAX_PORT_ADDRESS) { // Define MIN_PORT_ADDRESS and MAX_PORT_ADDRESS
        kprintf("Error: Invalid port address 0x%x\n", serial->port);
        return 0; // or an appropriate error code
    }

    size_t read = 0;

    while (read < size) {
        // Wait for DR (Data Ready) bit
        while ((inb(serial->port + 5) & 0x01) == 0) { }

        // Read byte from the serial port
        buffer[read] = inb(serial->port);
        read++;
    }

    return read;
}

// Handle IOCTL operations
int serial_ioctl(uint64_t port_id, uint64_t request, void *arg) {
    serial_port_t *serial = (serial_port_t *)port_id;
    switch (request) {
        case SERIAL_IOCTL_FLUSH_OUTPUT:
            while ((inb(serial->port + 5) & 0x40) == 0) { } // Wait for TEMT
            return 0;

        case SERIAL_IOCTL_SET_BAUDRATE: {
            int baud_rate = *(int *)arg;
            int divisor = 115200 / baud_rate;

            outb(serial->port + 3, 0x80); // Enable DLAB
            outb(serial->port, divisor & 0xFF); // Set divisor low byte
            outb(serial->port + 1, (divisor >> 8) & 0xFF); // Set divisor high byte
            outb(serial->port + 3, 0x03); // Disable DLAB
            return 0;
        }

        default:
            return -1; // Invalid request
    }
}