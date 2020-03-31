
#include <iostream>
#include <api.h>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <cstring>
#include <termios.h>
#include <unistd.h>

static void PiLogCallback(const char* log)
{
    std::cout << log << std::endl;
}

static int USBPort = 0;

static bool PiMoveServoCallback(int servo, float angle, uint32_t time)
{
    char moves[32];
    int angle_int = (int)(1500.f + (angle / 90.0f) * 500.f);
    sprintf(moves, "#%d P%d\r", servo, angle_int);

    std::cout << "Moving: " << moves << std::endl;
    write(USBPort, moves, strlen(moves));
    return false;
}


static int PiInitDriver()
{
    int USB = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);

    if ( USB < 0 )
    {
        std::cout << "Error " << errno << " opening " << "/dev/ttyACM0" << ": " << strerror (errno) << std::endl;
        return -1;
    }

    struct termios tty;
    memset (&tty, 0, sizeof tty);

    if ( tcgetattr ( USB, &tty ) != 0 )
    {
        std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
        return -1;
    }

    cfsetospeed (&tty, B115200);
    cfsetispeed (&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // ignore break signal
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= 0;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tcflush( USB, TCIFLUSH );

    if ( tcsetattr ( USB, TCSANOW, &tty ) != 0)
    {
        std::cout << "Error " << errno << " from tcsetattr" << std::endl;
        return -1;
    }

    return USB;
}


#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
int main()
{
    if (RoboInit("animations", PiLogCallback, PiMoveServoCallback) == 0)
    {
        std::cerr << "Failed to init robo lib!" << std::endl;
        exit(-1);
    }

    USBPort = PiInitDriver();
    if (USBPort == -1)
    {
        exit(-2);
    }

    RoboMove(MOVE_Forward);

    std::chrono::steady_clock::time_point oldTime = std::chrono::steady_clock::now();
    while (true)
    {
        auto now = std::chrono::steady_clock::now();
        auto deltaTime = std::chrono::duration_cast<std::chrono::milliseconds>(now - oldTime).count();
        oldTime = now;
        RoboUpdate(deltaTime);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

}
#pragma clang diagnostic pop