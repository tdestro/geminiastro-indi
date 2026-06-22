#include "geminiastro_protocol.h"

#include <algorithm>
#include <charconv>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <sstream>

namespace geminiastro
{
namespace
{

std::string command(const char *body)
{
    std::string out(body);
    out.push_back(Terminator);
    return out;
}

std::string payload(const std::string &response)
{
    if (response.empty())
        return {};

    auto end = response.find(Terminator);
    if (end == std::string::npos)
        end = response.size();

    return response.substr(0, end);
}

bool parsePrefixedPayload(const std::string &response, char prefix, std::string &value)
{
    auto p = payload(response);
    if (p.size() < 2 || p.front() != prefix)
        return false;

    value = p.substr(1);
    return true;
}

bool parseUint(const std::string &text, uint32_t &value)
{
    if (text.empty())
        return false;

    uint64_t parsed = 0;
    const char *begin = text.data();
    const char *end = begin + text.size();
    auto result = std::from_chars(begin, end, parsed);
    if (result.ec != std::errc() || result.ptr != end || parsed > std::numeric_limits<uint32_t>::max())
        return false;

    value = static_cast<uint32_t>(parsed);
    return true;
}

bool parsePrefixedUint(const std::string &response, char prefix, uint32_t &value)
{
    std::string raw;
    return parsePrefixedPayload(response, prefix, raw) && parseUint(raw, value);
}

bool parsePrefixedBool(const std::string &response, char prefix, bool &value)
{
    uint32_t raw = 0;
    if (!parsePrefixedUint(response, prefix, raw))
        return false;

    value = raw != 0;
    return true;
}

bool parsePrefixedDouble(const std::string &response, char prefix, double &value)
{
    std::string raw;
    if (!parsePrefixedPayload(response, prefix, raw) || raw.empty())
        return false;

    char *end = nullptr;
    value = std::strtod(raw.c_str(), &end);
    return end != nullptr && *end == '\0';
}

std::string fixed2(double value)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << value;
    return out.str();
}

} // namespace

std::string handshakeCommand()
{
    return command(":02");
}

std::string firmwareCommand()
{
    return command(":03");
}

std::string controllerModelCommand()
{
    return command(":04");
}

std::string maxPositionCommand()
{
    return command(":08");
}

std::string maxIncrementCommand()
{
    return command(":10");
}

std::string positionCommand()
{
    return command(":00");
}

std::string movingCommand()
{
    return command(":01");
}

std::string temperatureCommand()
{
    return command(":06");
}

std::string coilPowerCommand()
{
    return command(":11");
}

std::string reverseCommand()
{
    return command(":13");
}

std::string temperaturePrecisionCommand()
{
    return command(":21");
}

std::string temperatureCompEnabledCommand()
{
    return command(":24");
}

std::string temperatureCompAvailableCommand()
{
    return command(":25");
}

std::string temperatureCompValueCommand()
{
    return command(":26");
}

std::string temperatureProbeAvailableCommand()
{
    return command(":38");
}

std::string temperatureCompOptionCommand()
{
    return command(":83");
}

std::string temperatureCompDirectionCommand()
{
    return command(":87");
}

std::string homeSwitchCommand()
{
    return command(":63");
}

std::string jogEnabledCommand()
{
    return command(":66");
}

std::string jogDirectionCommand()
{
    return command(":68");
}

std::string stepperPowerCommand()
{
    return command(":89");
}

std::string stepModeCommand()
{
    return command(":29");
}

std::string stepSizeEnabledCommand()
{
    return command(":32");
}

std::string stepSizeCommand()
{
    return command(":33");
}

std::string motorSpeedCommand()
{
    return command(":43");
}

std::string delayAfterMoveCommand()
{
    return command(":72");
}

std::string backlashInEnabledCommand()
{
    return command(":74");
}

std::string backlashOutEnabledCommand()
{
    return command(":76");
}

std::string backlashInStepsCommand()
{
    return command(":78");
}

std::string backlashOutStepsCommand()
{
    return command(":80");
}

std::string displayStatusCommand()
{
    return command(":37");
}

std::string lcdPageDisplayTimeCommand()
{
    return command(":34");
}

std::string lcdUpdateWhileMovingCommand()
{
    return command(":62");
}

std::string moveAbsoluteCommand(uint32_t position)
{
    return ":05" + std::to_string(position) + Terminator;
}

std::string setMaxPositionCommand(uint32_t maxPosition)
{
    return ":07" + std::to_string(maxPosition) + Terminator;
}

std::string abortCommand()
{
    return command(":27");
}

std::string homeCommand()
{
    return command(":28");
}

std::string syncPositionCommand(uint32_t position)
{
    return ":31" + std::to_string(position) + Terminator;
}

std::string setCoilPowerCommand(bool enabled)
{
    return command(enabled ? ":121" : ":120");
}

std::string setReverseCommand(bool enabled)
{
    return command(enabled ? ":141" : ":140");
}

std::string setTemperaturePrecisionCommand(uint32_t precisionCode)
{
    return ":20" + std::to_string(precisionCode) + Terminator;
}

std::string setTemperatureCompValueCommand(uint32_t stepsPerDegree)
{
    return ":22" + std::to_string(stepsPerDegree) + Terminator;
}

std::string setTemperatureCompEnabledCommand(bool enabled)
{
    return command(enabled ? ":231" : ":230");
}

std::string setTemperatureCompDirectionCommand(bool outward)
{
    return command(outward ? ":881" : ":880");
}

std::string setJogDirectionCommand(bool outward)
{
    return command(outward ? ":671" : ":670");
}

std::string jogStartCommand()
{
    return command(":651");
}

std::string jogStopCommand()
{
    return command(":650");
}

std::string setMotorSpeedSlowCommand()
{
    return command(":150");
}

std::string setMotorSpeedMediumCommand()
{
    return command(":151");
}

std::string setMotorSpeedFastCommand()
{
    return command(":152");
}

std::string setMotorSpeedCommand(uint32_t speed)
{
    return ":15" + std::to_string(speed) + Terminator;
}

std::string setStepSizeEnabledCommand(bool enabled)
{
    return command(enabled ? ":181" : ":180");
}

std::string setStepSizeCommand(double size)
{
    return ":19" + fixed2(size) + Terminator;
}

std::string setStepModeCommand(uint32_t mode)
{
    return ":30" + std::to_string(mode) + Terminator;
}

std::string setDelayAfterMoveCommand(uint32_t delay)
{
    return ":71" + std::to_string(delay) + Terminator;
}

std::string setBacklashInEnabledCommand(bool enabled)
{
    return command(enabled ? ":731" : ":730");
}

std::string setBacklashOutEnabledCommand(bool enabled)
{
    return command(enabled ? ":751" : ":750");
}

std::string setBacklashInStepsCommand(uint32_t steps)
{
    return ":77" + std::to_string(steps) + Terminator;
}

std::string setBacklashOutStepsCommand(uint32_t steps)
{
    return ":79" + std::to_string(steps) + Terminator;
}

std::string setLcdPageDisplayTimeCommand(uint32_t milliseconds)
{
    switch (milliseconds)
    {
        case 2000:
            return command(":352");
        case 3000:
            return command(":353");
        case 4000:
            return command(":354");
        default:
            return {};
    }
}

std::string setLcdUpdateWhileMovingCommand(bool enabled)
{
    return command(enabled ? ":611" : ":610");
}

std::string setLcdEnabledCommand(bool enabled)
{
    return command(enabled ? ":361" : ":360");
}

std::string setDelayedDisplayUpdateCommand(bool enabled)
{
    return command(enabled ? ":901" : ":900");
}

std::string setLcdPageOptionCommand(uint32_t option)
{
    return ":92" + std::to_string(option) + Terminator;
}

std::string writeSettingsToEepromCommand()
{
    return command(":48");
}

std::string writeDefaultSettingsCommand()
{
    return command(":42");
}

std::string resetControllerCommand()
{
    return command(":40");
}

bool parseHandshake(const std::string &response)
{
    std::string raw;
    return parsePrefixedPayload(response, 'E', raw) && raw == "OK";
}

bool parseFirmware(const std::string &response, std::string &firmware)
{
    return parsePrefixedPayload(response, 'F', firmware) && !firmware.empty();
}

bool parseControllerModel(const std::string &response, std::string &model)
{
    if (!parsePrefixedPayload(response, 'F', model) || model.empty())
        return false;

    std::replace(model.begin(), model.end(), '\r', ' ');
    std::replace(model.begin(), model.end(), '\n', ' ');
    return true;
}

bool parseMaxPosition(const std::string &response, uint32_t &position)
{
    return parsePrefixedUint(response, 'M', position);
}

bool parseMaxIncrement(const std::string &response, uint32_t &increment)
{
    return parsePrefixedUint(response, 'Y', increment);
}

bool parsePosition(const std::string &response, uint32_t &position)
{
    return parsePrefixedUint(response, 'P', position);
}

bool parseMoving(const std::string &response, bool &moving)
{
    return parsePrefixedBool(response, 'I', moving);
}

bool parseTemperature(const std::string &response, double &temperatureC)
{
    return parsePrefixedDouble(response, 'Z', temperatureC);
}

bool parseCoilPower(const std::string &response, bool &enabled)
{
    return parsePrefixedBool(response, 'O', enabled);
}

bool parseReverse(const std::string &response, bool &enabled)
{
    return parsePrefixedBool(response, 'R', enabled);
}

bool parseTemperaturePrecision(const std::string &response, uint32_t &precision)
{
    return parsePrefixedUint(response, 'Q', precision);
}

bool parseTemperatureCompEnabled(const std::string &response, bool &enabled)
{
    return parsePrefixedBool(response, '1', enabled);
}

bool parseTemperatureCompAvailable(const std::string &response, bool &available)
{
    return parsePrefixedBool(response, 'A', available);
}

bool parseTemperatureCompValue(const std::string &response, uint32_t &steps)
{
    return parsePrefixedUint(response, 'B', steps);
}

bool parseTemperatureProbeAvailable(const std::string &response, bool &available)
{
    return parsePrefixedBool(response, 'b', available);
}

bool parseTemperatureCompOption(const std::string &response, bool &enabled)
{
    return parsePrefixedBool(response, 'c', enabled);
}

bool parseTemperatureCompDirection(const std::string &response, bool &outward)
{
    return parsePrefixedBool(response, 'k', outward);
}

bool parseHomeSwitch(const std::string &response, bool &closed)
{
    return parsePrefixedBool(response, 'H', closed);
}

bool parseJogEnabled(const std::string &response, bool &enabled)
{
    return parsePrefixedBool(response, 'K', enabled);
}

bool parseJogDirection(const std::string &response, bool &outward)
{
    return parsePrefixedBool(response, 'V', outward);
}

bool parseStepperPower(const std::string &response, bool &powered)
{
    return parsePrefixedBool(response, '9', powered);
}

bool parseStepMode(const std::string &response, uint32_t &mode)
{
    return parsePrefixedUint(response, 'S', mode);
}

bool parseStepSizeEnabled(const std::string &response, bool &enabled)
{
    return parsePrefixedBool(response, 'U', enabled);
}

bool parseStepSize(const std::string &response, double &size)
{
    return parsePrefixedDouble(response, 'T', size);
}

bool parseMotorSpeed(const std::string &response, uint32_t &speed)
{
    return parsePrefixedUint(response, 'C', speed);
}

bool parseDelayAfterMove(const std::string &response, uint32_t &delay)
{
    return parsePrefixedUint(response, '3', delay);
}

bool parseBacklashInEnabled(const std::string &response, bool &enabled)
{
    return parsePrefixedBool(response, '4', enabled);
}

bool parseBacklashOutEnabled(const std::string &response, bool &enabled)
{
    return parsePrefixedBool(response, '5', enabled);
}

bool parseBacklashInSteps(const std::string &response, uint32_t &steps)
{
    return parsePrefixedUint(response, '6', steps);
}

bool parseBacklashOutSteps(const std::string &response, uint32_t &steps)
{
    return parsePrefixedUint(response, '7', steps);
}

bool parseDisplayStatus(const std::string &response, bool &enabled)
{
    return parsePrefixedBool(response, 'D', enabled);
}

bool parseLcdPageDisplayTime(const std::string &response, uint32_t &milliseconds)
{
    return parsePrefixedUint(response, 'X', milliseconds);
}

bool parseLcdUpdateWhileMoving(const std::string &response, bool &enabled)
{
    return parsePrefixedBool(response, 'L', enabled);
}

uint32_t clampPosition(int64_t requested, uint32_t maxPosition)
{
    if (requested < 0)
        return 0;

    const auto maxAsInt = static_cast<int64_t>(maxPosition);
    if (requested > maxAsInt)
        return maxPosition;

    return static_cast<uint32_t>(requested);
}

bool validateSafeLimits(uint32_t configuredMin, uint32_t configuredMax, uint32_t guardSteps,
                        uint32_t controllerMaxPosition)
{
    if (controllerMaxPosition == 0 || configuredMax > controllerMaxPosition || configuredMax <= configuredMin)
        return false;

    const auto range = static_cast<uint64_t>(configuredMax) - configuredMin;
    return static_cast<uint64_t>(guardSteps) * 2 < range;
}

uint32_t effectiveSafeLimitMin(uint32_t configuredMin, uint32_t guardSteps)
{
    return configuredMin + guardSteps;
}

uint32_t effectiveSafeLimitMax(uint32_t configuredMax, uint32_t guardSteps)
{
    return configuredMax - guardSteps;
}

} // namespace geminiastro
