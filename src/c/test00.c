#include <stdio.h>
//#include <string.h>
#include <fcntl.h>
//#include <errno.h>
#include <termios.h> // For serial port settings
#include <unistd.h>  // For write()


int setup_tty(unsigned char* ttyUSBx) {
    int serial_port = open(ttyUSBx, O_RDWR | O_NOCTTY | O_NDELAY);

    if (serial_port < 0) {
        perror("Error opening serial port");
        return -1;
    }

    // Configure serial port settings (e.g., baud rate, data bits, parity)
    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0) {
        perror("Error getting serial port attributes");
        close(serial_port);
        return -1;
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
        return -1;
    }
  return serial_port;
}

int write_to_tty(unsigned char *data, int serial_port){
    int data_len = sizeof(&data);
    // Write the binary data
    int bytes_written = write(serial_port, &data, data_len);

    if (bytes_written < 0) {
        perror("Error writing to serial port");
    } else if (bytes_written != data_len) {
        fprintf(stderr, "Warning: Only %d of %d bytes written.\n", bytes_written, data_len);
    } else {
        printf("Successfully wrote %d bytes of binary data.\n", bytes_written);
    }
  return 0;
}

int read_from_tty(int serial_port){
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
    return 0;
}


int main() {
int serial_port;
unsigned char ttyUSBx[12] = "/dev/ttyUSB0";
unsigned char str_handshake[] = {0x0C, 0x03, 0x79, 0x18, 0x00, 0x07, 0x9C, 0x28};
unsigned char str0[] = {0x0A, 0x03, 0x75, 0x30, 0x00, 0x1B, 0x1E, 0xB9}; //responce 59 bytes
unsigned char str1[] = {0x0A, 0x03, 0x79, 0x18, 0x00, 0x0A, 0x5D, 0xED}; //responce 25 bytes


    serial_port = setup_tty(ttyUSBx);
  if (serial_port > 0){

    write_to_tty(str_handshake, serial_port);
    usleep(100*1000); //100ms - need to tune

    write_to_tty(str0, serial_port);
    usleep(500*1000); //500ms - need to tune
    read_from_tty(serial_port);

    usleep(500*1000); //500ms - need to tune
    write_to_tty(str1, serial_port);
    usleep(500*1000); //500ms - need to tune
    read_from_tty(serial_port);

    close(serial_port);
  } else {
    printf("Exiting...");
    return -1;
  }
  return 0;
}
