#pragma once

#include <cstdint>
#include <string>

namespace geminiastro
{

constexpr char Terminator = '#';

std::string handshakeCommand();
std::string firmwareCommand();
std::string controllerModelCommand();
std::string maxPositionCommand();
std::string maxIncrementCommand();
std::string positionCommand();
std::string movingCommand();
std::string temperatureCommand();
std::string coilPowerCommand();
std::string reverseCommand();
std::string temperaturePrecisionCommand();
std::string temperatureCompEnabledCommand();
std::string temperatureCompAvailableCommand();
std::string temperatureCompValueCommand();
std::string temperatureProbeAvailableCommand();
std::string temperatureCompOptionCommand();
std::string temperatureCompDirectionCommand();
std::string homeSwitchCommand();
std::string jogEnabledCommand();
std::string jogDirectionCommand();
std::string stepperPowerCommand();
std::string stepModeCommand();
std::string stepSizeEnabledCommand();
std::string stepSizeCommand();
std::string motorSpeedCommand();
std::string delayAfterMoveCommand();
std::string backlashInEnabledCommand();
std::string backlashOutEnabledCommand();
std::string backlashInStepsCommand();
std::string backlashOutStepsCommand();
std::string displayStatusCommand();
std::string lcdPageDisplayTimeCommand();
std::string lcdUpdateWhileMovingCommand();
std::string moveAbsoluteCommand(uint32_t position);
std::string setMaxPositionCommand(uint32_t maxPosition);
std::string abortCommand();
std::string homeCommand();
std::string syncPositionCommand(uint32_t position);
std::string setCoilPowerCommand(bool enabled);
std::string setReverseCommand(bool enabled);
std::string setTemperaturePrecisionCommand(uint32_t precisionCode);
std::string setTemperatureCompValueCommand(uint32_t stepsPerDegree);
std::string setTemperatureCompEnabledCommand(bool enabled);
std::string setTemperatureCompDirectionCommand(bool outward);
std::string setJogDirectionCommand(bool outward);
std::string jogStartCommand();
std::string jogStopCommand();
std::string setMotorSpeedSlowCommand();
std::string setMotorSpeedMediumCommand();
std::string setMotorSpeedFastCommand();
std::string setMotorSpeedCommand(uint32_t speed);
std::string setStepSizeEnabledCommand(bool enabled);
std::string setStepSizeCommand(double size);
std::string setStepModeCommand(uint32_t mode);
std::string setDelayAfterMoveCommand(uint32_t delay);
std::string setBacklashInEnabledCommand(bool enabled);
std::string setBacklashOutEnabledCommand(bool enabled);
std::string setBacklashInStepsCommand(uint32_t steps);
std::string setBacklashOutStepsCommand(uint32_t steps);
std::string setLcdPageDisplayTimeCommand(uint32_t milliseconds);
std::string setLcdUpdateWhileMovingCommand(bool enabled);
std::string setLcdEnabledCommand(bool enabled);
std::string setDelayedDisplayUpdateCommand(bool enabled);
std::string setLcdPageOptionCommand(uint32_t option);
std::string writeSettingsToEepromCommand();
std::string writeDefaultSettingsCommand();
std::string resetControllerCommand();

bool parseHandshake(const std::string &response);
bool parseFirmware(const std::string &response, std::string &firmware);
bool parseControllerModel(const std::string &response, std::string &model);
bool parseMaxPosition(const std::string &response, uint32_t &position);
bool parseMaxIncrement(const std::string &response, uint32_t &increment);
bool parsePosition(const std::string &response, uint32_t &position);
bool parseMoving(const std::string &response, bool &moving);
bool parseTemperature(const std::string &response, double &temperatureC);
bool parseCoilPower(const std::string &response, bool &enabled);
bool parseReverse(const std::string &response, bool &enabled);
bool parseTemperaturePrecision(const std::string &response, uint32_t &precision);
bool parseTemperatureCompEnabled(const std::string &response, bool &enabled);
bool parseTemperatureCompAvailable(const std::string &response, bool &available);
bool parseTemperatureCompValue(const std::string &response, uint32_t &steps);
bool parseTemperatureProbeAvailable(const std::string &response, bool &available);
bool parseTemperatureCompOption(const std::string &response, bool &enabled);
bool parseTemperatureCompDirection(const std::string &response, bool &outward);
bool parseHomeSwitch(const std::string &response, bool &closed);
bool parseJogEnabled(const std::string &response, bool &enabled);
bool parseJogDirection(const std::string &response, bool &outward);
bool parseStepperPower(const std::string &response, bool &powered);
bool parseStepMode(const std::string &response, uint32_t &mode);
bool parseStepSizeEnabled(const std::string &response, bool &enabled);
bool parseStepSize(const std::string &response, double &size);
bool parseMotorSpeed(const std::string &response, uint32_t &speed);
bool parseDelayAfterMove(const std::string &response, uint32_t &delay);
bool parseBacklashInEnabled(const std::string &response, bool &enabled);
bool parseBacklashOutEnabled(const std::string &response, bool &enabled);
bool parseBacklashInSteps(const std::string &response, uint32_t &steps);
bool parseBacklashOutSteps(const std::string &response, uint32_t &steps);
bool parseDisplayStatus(const std::string &response, bool &enabled);
bool parseLcdPageDisplayTime(const std::string &response, uint32_t &milliseconds);
bool parseLcdUpdateWhileMoving(const std::string &response, bool &enabled);

uint32_t clampPosition(int64_t requested, uint32_t maxPosition);
bool validateSafeLimits(uint32_t configuredMin, uint32_t configuredMax, uint32_t guardSteps,
                        uint32_t controllerMaxPosition);
uint32_t effectiveSafeLimitMin(uint32_t configuredMin, uint32_t guardSteps);
uint32_t effectiveSafeLimitMax(uint32_t configuredMax, uint32_t guardSteps);

} // namespace geminiastro
