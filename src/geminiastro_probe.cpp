#include "geminiastro_protocol.h"

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>
#include <utility>
#include <string>
#include <vector>

namespace
{

bool configurePort(int fd)
{
    termios tty {};
    if (tcgetattr(fd, &tty) != 0)
        return false;

    cfmakeraw(&tty);
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    return tcsetattr(fd, TCSANOW, &tty) == 0;
}

std::string sendReadCommand(int fd, const std::string &command)
{
    tcflush(fd, TCIOFLUSH);
    if (write(fd, command.data(), command.size()) != static_cast<ssize_t>(command.size()))
        return {};

    std::string response;
    const auto deadlineSeconds = 3;

    for (int i = 0; i < deadlineSeconds * 10; ++i)
    {
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(fd, &readSet);

        timeval timeout {};
        timeout.tv_usec = 100000;

        int ready = select(fd + 1, &readSet, nullptr, nullptr, &timeout);
        if (ready <= 0)
            continue;

        char buffer[64] = {0};
        auto bytes = read(fd, buffer, sizeof(buffer));
        if (bytes <= 0)
            continue;

        response.append(buffer, static_cast<size_t>(bytes));
        if (response.find(geminiastro::Terminator) != std::string::npos)
            break;
    }

    return response;
}

} // namespace

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "usage: geminiastro_probe <serial-port>\n";
        return 2;
    }

    int fd = open(argv[1], O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
        std::cerr << "open failed: " << std::strerror(errno) << "\n";
        return 1;
    }

    if (!configurePort(fd))
    {
        std::cerr << "serial configuration failed: " << std::strerror(errno) << "\n";
        close(fd);
        return 1;
    }

    sleep(2);

    const std::vector<std::pair<std::string, std::string>> commands {
        {"handshake", geminiastro::handshakeCommand()},
        {"firmware", geminiastro::firmwareCommand()},
        {"controller_model", geminiastro::controllerModelCommand()},
        {"max_position", geminiastro::maxPositionCommand()},
        {"max_increment", geminiastro::maxIncrementCommand()},
        {"position", geminiastro::positionCommand()},
        {"moving", geminiastro::movingCommand()},
        {"temperature", geminiastro::temperatureCommand()},
        {"coil_power", geminiastro::coilPowerCommand()},
        {"reverse", geminiastro::reverseCommand()},
        {"temperature_precision", geminiastro::temperaturePrecisionCommand()},
        {"temperature_comp_enabled", geminiastro::temperatureCompEnabledCommand()},
        {"temperature_comp_available", geminiastro::temperatureCompAvailableCommand()},
        {"temperature_comp_value", geminiastro::temperatureCompValueCommand()},
        {"temperature_probe_available", geminiastro::temperatureProbeAvailableCommand()},
        {"temperature_comp_option", geminiastro::temperatureCompOptionCommand()},
        {"temperature_comp_direction", geminiastro::temperatureCompDirectionCommand()},
        {"home_switch", geminiastro::homeSwitchCommand()},
        {"jog_enabled", geminiastro::jogEnabledCommand()},
        {"jog_direction", geminiastro::jogDirectionCommand()},
        {"stepper_power", geminiastro::stepperPowerCommand()},
        {"step_mode", geminiastro::stepModeCommand()},
        {"step_size_enabled", geminiastro::stepSizeEnabledCommand()},
        {"step_size", geminiastro::stepSizeCommand()},
        {"motor_speed", geminiastro::motorSpeedCommand()},
        {"delay_after_move", geminiastro::delayAfterMoveCommand()},
        {"backlash_in_enabled", geminiastro::backlashInEnabledCommand()},
        {"backlash_out_enabled", geminiastro::backlashOutEnabledCommand()},
        {"backlash_in_steps", geminiastro::backlashInStepsCommand()},
        {"backlash_out_steps", geminiastro::backlashOutStepsCommand()},
        {"display_status", geminiastro::displayStatusCommand()},
        {"lcd_page_display_time", geminiastro::lcdPageDisplayTimeCommand()},
        {"lcd_update_while_moving", geminiastro::lcdUpdateWhileMovingCommand()},
    };

    for (const auto &[label, command] : commands)
    {
        auto response = sendReadCommand(fd, command);
        std::cout << label << " " << command << " -> " << (response.empty() ? "<timeout>" : response) << "\n";
        usleep(150000);
    }

    close(fd);
    return 0;
}
