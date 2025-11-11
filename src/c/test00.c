#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h> // For serial port settings
#include <unistd.h>  // For write()

int main() {
    int serial_port = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);

    if (serial_port < 0) {
        perror("Error opening serial port");
        return 1;
    }

    // Configure serial port settings (e.g., baud rate, data bits, parity)
    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0) {
        perror("Error getting serial port attributes");
        close(serial_port);
        return 1;
    }

    cfsetospeed(&tty, B9600); // Set output baud rate to 9600
    cfsetispeed(&tty, B9600); // Set input baud rate to 9600

    tty.c_cflag &= ~PARENB;   // No parity
    tty.c_cflag &= ~CSTOPB;   // 1 stop bit
    tty.c_cflag &= ~CSIZE;    // Clear data size bits
    tty.c_cflag |= CS8;       // 8 data bits
    tty.c_cflag &= ~CRTSCTS;  // Disable hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

    tty.c_lflag &= ~ICANON;   // Disable canonical mode (raw input)
    tty.c_lflag &= ~ECHO;     // Disable echo
    tty.c_lflag &= ~ECHOE;    // Disable erase character echo
    tty.c_lflag &= ~ECHONL;   // Disable newline echo

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable special handling of received bytes

    tty.c_oflag &= ~OPOST;    // Disable output processing

    tty.c_cc[VMIN] = 0;       // Non-blocking read
    tty.c_cc[VTIME] = 15;      // 1.5 seconds read timeout

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        perror("Error setting serial port attributes");
        close(serial_port);
        return 1;
    }

    // Binary data to send

    unsigned char data[] = {0x0A, 0x03, 0x75, 0x30, 0x00, 0x1B, 0x1E, 0xB9};
    int data_len = sizeof(data);

    // Write the binary data
    int bytes_written = write(serial_port, data, data_len);

    if (bytes_written < 0) {
        perror("Error writing to serial port");
    } else if (bytes_written != data_len) {
        fprintf(stderr, "Warning: Only %d of %d bytes written.\n", bytes_written, data_len);
    } else {
        printf("Successfully wrote %d bytes of binary data.\n", bytes_written);
    }

//    getchar(); //press Enter
    sleep(1);

    unsigned char read_buf[256];
    int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

    if (num_bytes < 0) {
        perror("Error reading from serial port");
    } else if (num_bytes > 0) {
        printf("Received %d bytes: ", num_bytes);
        for (int i = 0; i < num_bytes; i++) {
            printf("0x%02X ", read_buf[i]);
        }
        printf("\n");
    } else {
        printf("No data received.\n");
    }
    close(serial_port);
    return 0;
}
