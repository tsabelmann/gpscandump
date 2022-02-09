#include <iostream>
#include <fstream>
#include <boost/chrono.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <net/if.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
    

int configure_serial_port(char *serial_port_name, int baud_rate, bool parity, bool stop_bits, bool flow_control, int bits_per_byte) {
    int serial_port = open(serial_port_name, O_RDWR);
    if (serial_port < 0) {
        return -1;
    }

    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0) {
        return -1;
    }

    // configure parity
    if (parity) {
        tty.c_cflag |= PARENB;
    } else {
        tty.c_cflag &= ~PARENB;
    }

    // configure stop bits
    if (stop_bits) {
        tty.c_cflag |= CSTOPB;
    } else {
        tty.c_cflag &= ~CSTOPB;
    }

    tty.c_cflag &= ~CSIZE;
    switch (bits_per_byte)
    {
    case 5:
        tty.c_cflag |= CS5;
        break;
    case 6:
        tty.c_cflag |= CS6;
        break;
    case 7:
        tty.c_cflag |= CS7;
        break;
    case 8:
        tty.c_cflag |= CS8;
        break;
    
    default:
        return -1;
    }

    // tty.c_lflag &= ~ICANON;

    // tty.c_lflag &= ~ECHO; // Disable echo
    // tty.c_lflag &= ~ECHOE; // Disable erasure
    // tty.c_lflag &= ~ECHONL; // Disable new-line echo

    // tty.c_lflag &= ~ISIG;

    // tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    // tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    // tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    // tty.c_cc[VMIN] = 0;

    // configure baud rate
    cfsetispeed(&tty, 4800);
    cfsetospeed(&tty, 4800);

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        return -1;
    }

    int nbytes;
    char buffer[100];
    while (true) {
        getline(buffer, sizeof(buffer), serial_port)


        nbytes = read(serial_port, buffer, sizeof(buffer));
        printf("%s\n", buffer);
    }


    if (close(serial_port) < 0) {
        return -1;
    }

    return 0;
}



int main(int argc, char *args[]) {
    if (argc < 4) {
        std::cout << "not enough arguments" << std::endl;
        return 1;
    }

    char* can_interface_name = args[1];
    char* serial_port_name = args[2];
    char* output_file_name = args[3];

    if (configure_serial_port(serial_port_name, 4800, false, false, false, 8) < 0) {
        std::cerr << "could not configure serial port " << serial_port_name << std::endl;
        return -1; 
    }

    std::string mystring;
    std::ifstream serial_port(serial_port_name);
    while (true) {
        std::getline(serial_port, mystring);
        std::cout << mystring << std::endl;
    }


    // std::cout << can_interface_name << std::endl;
    // std::cout << serial_port << std::endl;
    // std::cout << output << std::endl;

    // auto time = boost::chrono::high_resolution_clock::now();
    // std::cout << boost::chrono::time_point_cast<boost::chrono::milliseconds>(time) << std::endl;


    int s;
    int nbytes;
    struct sockaddr_can addr;
    socklen_t len = sizeof(addr);
    struct ifreq ifr;
    struct can_frame frame;

    // create can socket
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        std::cerr << "Socket" << std::endl;
        return 1;
    }

    // configure can address
    if (can_interface_name == "any") {
        addr.can_family = AF_CAN;
        addr.can_ifindex = 0;
    } else {
        strcpy(ifr.ifr_name, can_interface_name);
        ioctl(s, SIOCGIFINDEX, &ifr);

        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
    }

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "Bind" << std::endl;
        return 1;
    }

    while (true) {
        nbytes = recvfrom(s, &frame, sizeof(struct can_frame), 0, (struct sockaddr*)&addr, &len);
        if (!(nbytes < sizeof(struct can_frame))) {
            ifr.ifr_ifindex = addr.can_ifindex;
            ioctl(s, SIOCGIFNAME, &ifr);
            std::cout << ifr.ifr_name << std::endl;
        }
    }


    // close can socket
    if (close(s) < 0) {
        std::cerr << "Close" << std::endl;
        return 1;
    }

    return 0;
}