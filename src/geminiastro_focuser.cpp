#include "geminiastro_focuser.h"

#include "indicom.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace
{

constexpr int DriverVersionMajor = 0;
constexpr int DriverVersionMinor = 0;
constexpr uint8_t ResponseSize = 32;
constexpr uint8_t SerialTimeoutSeconds = 5;
constexpr uint32_t InitialSafeWindowSteps = 50;
constexpr uint32_t DefaultConfiguredGuardSteps = 25;
constexpr uint32_t DefaultCalibrationStep = 50;
constexpr uint32_t MaxControllerSettingValue = 250000;

bool updatedName(char *names[], int n, const char *target)
{
    for (int i = 0; i < n; ++i)
    {
        if (names[i] != nullptr && std::strcmp(names[i], target) == 0)
            return true;
    }

    return false;
}

bool finiteUInt(double raw, uint32_t maxValue, uint32_t &value)
{
    if (!std::isfinite(raw) || raw < 0 || raw > maxValue || raw != std::lround(raw))
        return false;

    value = static_cast<uint32_t>(std::lround(raw));
    return true;
}

bool finiteDouble(double raw, double maxValue, double &value)
{
    if (!std::isfinite(raw) || raw < 0 || raw > maxValue)
        return false;

    value = raw;
    return true;
}

std::unique_ptr<GeminiAstroFocuser> driver(new GeminiAstroFocuser());

} // namespace

GeminiAstroFocuser::GeminiAstroFocuser()
{
    setVersion(DriverVersionMajor, DriverVersionMinor);
    setSupportedConnections(CONNECTION_SERIAL);
    FI::SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_ABORT | FOCUSER_CAN_SYNC);
}

const char *GeminiAstroFocuser::getDefaultName()
{
    return "GeminiAstro Focuser";
}

bool GeminiAstroFocuser::initProperties()
{
    INDI::Focuser::initProperties();

    FocusRelPosN[0].min = 0;
    FocusRelPosN[0].max = 10000;
    FocusRelPosN[0].value = 0;
    FocusRelPosN[0].step = 100;

    FocusAbsPosN[0].min = 0;
    FocusAbsPosN[0].max = 10000;
    FocusAbsPosN[0].value = 0;
    FocusAbsPosN[0].step = 100;

    ControllerInfoTP[0].fill("FIRMWARE", "Firmware", "");
    ControllerInfoTP[1].fill("MODEL", "Controller Model", "");
    ControllerInfoTP.fill(getDeviceName(), "FOCUSER_CONTROLLER_INFO", "Controller Info", MAIN_CONTROL_TAB, IP_RO, 0,
                          IPS_IDLE);

    TemperatureNP[0].fill("TEMPERATURE", "Celsius", "%6.2f", -50, 70, 0, 0);
    TemperatureNP.fill(getDeviceName(), "FOCUS_TEMPERATURE", "Temperature", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    SafeLimitsNP[0].fill("MIN_POSITION", "Minimum", "%.0f", 0, 10000, 1, 0);
    SafeLimitsNP[1].fill("MAX_POSITION", "Maximum", "%.0f", 0, 10000, 1, 0);
    SafeLimitsNP[2].fill("GUARD_STEPS", "Guard", "%.0f", 0, 1000, 1, DefaultConfiguredGuardSteps);
    SafeLimitsNP.fill(getDeviceName(), "FOCUS_SAFE_LIMITS", "Configured Safe Limits", MAIN_CONTROL_TAB, IP_RW, 0,
                      IPS_IDLE);

    LimitStatusNP[0].fill("EFFECTIVE_MIN_POSITION", "Effective Minimum", "%.0f", 0, 10000, 1, 0);
    LimitStatusNP[1].fill("EFFECTIVE_MAX_POSITION", "Effective Maximum", "%.0f", 0, 10000, 1, 10000);
    LimitStatusNP[2].fill("CONFIGURED_LIMITS_ACTIVE", "Configured", "%.0f", 0, 1, 1, 0);
    LimitStatusNP[3].fill("HOME_SWITCH_CLOSED", "Home Switch Closed", "%.0f", 0, 1, 1, 0);
    LimitStatusNP[4].fill("STEPPER_POWERED", "Stepper Powered", "%.0f", 0, 1, 1, 0);
    LimitStatusNP.fill(getDeviceName(), "FOCUS_LIMIT_STATUS", "Limit Status", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    CalibrationStepNP[0].fill("STEP_SIZE", "Step Size", "%.0f", 1, 10000, 1, DefaultCalibrationStep);
    CalibrationStepNP.fill(getDeviceName(), "FOCUS_LIMIT_CALIBRATION_STEP", "Safe-Limit Calibration Step",
                           MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE);

    CalibrationStatusNP[0].fill("CURRENT_POSITION", "Current Position", "%.0f", 0, 10000, 1, 0);
    CalibrationStatusNP[1].fill("CANDIDATE_MIN_POSITION", "Candidate Minimum", "%.0f", -1, 10000, 1, -1);
    CalibrationStatusNP[2].fill("CANDIDATE_MAX_POSITION", "Candidate Maximum", "%.0f", -1, 10000, 1, -1);
    CalibrationStatusNP[3].fill("READY_TO_APPLY", "Ready To Apply", "%.0f", 0, 1, 1, 0);
    CalibrationStatusNP[4].fill("LAST_NUDGE_TARGET", "Last Nudge Target", "%.0f", -1, 10000, 1, -1);
    CalibrationStatusNP.fill(getDeviceName(), "FOCUS_LIMIT_CALIBRATION_STATUS", "Safe-Limit Calibration Status",
                             MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    CalibrationSP[0].fill("NUDGE_INWARD", "Nudge Inward", ISS_OFF);
    CalibrationSP[1].fill("NUDGE_OUTWARD", "Nudge Outward", ISS_OFF);
    CalibrationSP[2].fill("CAPTURE_MIN", "Capture Minimum", ISS_OFF);
    CalibrationSP[3].fill("CAPTURE_MAX", "Capture Maximum", ISS_OFF);
    CalibrationSP[4].fill("APPLY_LIMITS", "Apply Limits", ISS_OFF);
    CalibrationSP[5].fill("RESET_CANDIDATES", "Reset Candidates", ISS_OFF);
    CalibrationSP[6].fill("CENTER_POSITION", "Center Position", ISS_OFF);
    CalibrationSP.fill(getDeviceName(), "FOCUS_LIMIT_CALIBRATION", "Safe-Limit Calibration", MAIN_CONTROL_TAB, IP_RW,
                       ISR_ATMOST1, 0, IPS_IDLE);

    ControllerStatusNP[0].fill("MAX_INCREMENT", "Max Increment", "%.0f", 0, 10000, 1, 0);
    ControllerStatusNP[1].fill("COIL_POWER_ENABLED", "Coil Power Enabled", "%.0f", 0, 1, 1, 0);
    ControllerStatusNP[2].fill("REVERSE_ENABLED", "Reverse Enabled", "%.0f", 0, 1, 1, 0);
    ControllerStatusNP[3].fill("TEMPERATURE_PRECISION", "Temperature Precision", "%.0f", 0, 1000, 1, 0);
    ControllerStatusNP[4].fill("TEMPERATURE_COMP_ENABLED", "Temp Comp Enabled", "%.0f", 0, 1, 1, 0);
    ControllerStatusNP[5].fill("TEMPERATURE_COMP_AVAILABLE", "Temp Comp Available", "%.0f", 0, 1, 1, 0);
    ControllerStatusNP[6].fill("TEMPERATURE_COMP_VALUE", "Temp Comp Value", "%.0f", 0, 100000, 1, 0);
    ControllerStatusNP[7].fill("TEMPERATURE_PROBE_AVAILABLE", "Temp Probe Available", "%.0f", 0, 1, 1, 0);
    ControllerStatusNP[8].fill("TEMPERATURE_COMP_OPTION", "Temp Comp Option", "%.0f", 0, 1, 1, 0);
    ControllerStatusNP[9].fill("TEMPERATURE_COMP_DIRECTION_OUT", "Temp Comp Direction Out", "%.0f", 0, 1, 1, 0);
    ControllerStatusNP[10].fill("JOG_ENABLED", "Jog Enabled", "%.0f", 0, 1, 1, 0);
    ControllerStatusNP[11].fill("JOG_DIRECTION_OUT", "Jog Direction Out", "%.0f", 0, 1, 1, 0);
    ControllerStatusNP[12].fill("STEP_MODE", "Step Mode", "%.0f", 0, 256, 1, 0);
    ControllerStatusNP[13].fill("STEP_SIZE_ENABLED", "Step Size Enabled", "%.0f", 0, 1, 1, 0);
    ControllerStatusNP[14].fill("STEP_SIZE", "Step Size", "%.2f", 0, 100000, 0, 0);
    ControllerStatusNP[15].fill("MOTOR_SPEED", "Motor Speed", "%.0f", 0, 100000, 1, 0);
    ControllerStatusNP[16].fill("DELAY_AFTER_MOVE", "Delay After Move", "%.0f", 0, 100000, 1, 0);
    ControllerStatusNP[17].fill("BACKLASH_IN_ENABLED", "Backlash IN Enabled", "%.0f", 0, 1, 1, 0);
    ControllerStatusNP[18].fill("BACKLASH_OUT_ENABLED", "Backlash OUT Enabled", "%.0f", 0, 1, 1, 0);
    ControllerStatusNP[19].fill("BACKLASH_IN_STEPS", "Backlash IN Steps", "%.0f", 0, 100000, 1, 0);
    ControllerStatusNP[20].fill("BACKLASH_OUT_STEPS", "Backlash OUT Steps", "%.0f", 0, 100000, 1, 0);
    ControllerStatusNP[21].fill("DISPLAY_ENABLED", "Display Enabled", "%.0f", 0, 1, 1, 0);
    ControllerStatusNP[22].fill("LCD_PAGE_DISPLAY_TIME", "LCD Page Time", "%.0f", 0, 100000, 1, 0);
    ControllerStatusNP[23].fill("LCD_UPDATE_WHILE_MOVING", "LCD Update While Moving", "%.0f", 0, 1, 1, 0);
    ControllerStatusNP.fill(getDeviceName(), "FOCUSER_CONTROLLER_STATUS", "Controller Status", MAIN_CONTROL_TAB, IP_RO,
                            0, IPS_IDLE);

    MotorSettingsNP[0].fill("CONTROLLER_MAX_POSITION", "Controller Max Position", "%.0f", 1, MaxControllerSettingValue, 1,
                            10000);
    MotorSettingsNP[1].fill("MOTOR_SPEED", "Motor Speed", "%.0f", 0, MaxControllerSettingValue, 1, 0);
    MotorSettingsNP[2].fill("STEP_MODE", "Step Mode", "%.0f", 0, 256, 1, 0);
    MotorSettingsNP[3].fill("STEP_SIZE", "Step Size", "%.2f", 0, MaxControllerSettingValue, 0, 0);
    MotorSettingsNP[4].fill("DELAY_AFTER_MOVE", "Delay After Move", "%.0f", 0, MaxControllerSettingValue, 1, 0);
    MotorSettingsNP.fill(getDeviceName(), "GEMINIASTRO_MOTOR_SETTINGS", "GeminiAstro Motor Settings",
                         MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE);

    MotorSwitchSP[0].fill("COIL_POWER_ON", "Coil Power On", ISS_OFF);
    MotorSwitchSP[1].fill("COIL_POWER_OFF", "Coil Power Off", ISS_OFF);
    MotorSwitchSP[2].fill("REVERSE_ON", "Reverse On", ISS_OFF);
    MotorSwitchSP[3].fill("REVERSE_OFF", "Reverse Off", ISS_OFF);
    MotorSwitchSP[4].fill("STEP_SIZE_ENABLE", "Step Size Enable", ISS_OFF);
    MotorSwitchSP[5].fill("STEP_SIZE_DISABLE", "Step Size Disable", ISS_OFF);
    MotorSwitchSP[6].fill("SPEED_SLOW", "Speed Slow", ISS_OFF);
    MotorSwitchSP[7].fill("SPEED_MEDIUM", "Speed Medium", ISS_OFF);
    MotorSwitchSP[8].fill("SPEED_FAST", "Speed Fast", ISS_OFF);
    MotorSwitchSP.fill(getDeviceName(), "GEMINIASTRO_MOTOR_SWITCHES", "GeminiAstro Motor Switches",
                       MAIN_CONTROL_TAB, IP_RW, ISR_ATMOST1, 0, IPS_IDLE);

    JogSP[0].fill("HOME", "Home", ISS_OFF);
    JogSP[1].fill("JOG_IN", "Jog In", ISS_OFF);
    JogSP[2].fill("JOG_OUT", "Jog Out", ISS_OFF);
    JogSP[3].fill("JOG_STOP", "Jog Stop", ISS_OFF);
    JogSP.fill(getDeviceName(), "GEMINIASTRO_JOG", "GeminiAstro Jog", MAIN_CONTROL_TAB, IP_RW, ISR_ATMOST1, 0,
               IPS_IDLE);

    TemperatureCompNP[0].fill("TEMPERATURE_PRECISION_CODE", "Temperature Precision Code", "%.0f", 9, 12, 1, 10);
    TemperatureCompNP[1].fill("TEMPERATURE_COMP_STEPS", "Temp Comp Steps", "%.0f", 0, MaxControllerSettingValue, 1, 0);
    TemperatureCompNP.fill(getDeviceName(), "GEMINIASTRO_TEMP_COMP", "GeminiAstro Temperature Compensation",
                           MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE);

    TemperatureCompSP[0].fill("ENABLE", "Enable", ISS_OFF);
    TemperatureCompSP[1].fill("DISABLE", "Disable", ISS_OFF);
    TemperatureCompSP[2].fill("DIRECTION_OUT", "Direction Out", ISS_OFF);
    TemperatureCompSP[3].fill("DIRECTION_IN", "Direction In", ISS_OFF);
    TemperatureCompSP.fill(getDeviceName(), "GEMINIASTRO_TEMP_COMP_SWITCHES",
                           "GeminiAstro Temperature Compensation Switches", MAIN_CONTROL_TAB, IP_RW, ISR_ATMOST1, 0,
                           IPS_IDLE);

    BacklashNP[0].fill("BACKLASH_IN_STEPS", "Backlash IN Steps", "%.0f", 0, MaxControllerSettingValue, 1, 0);
    BacklashNP[1].fill("BACKLASH_OUT_STEPS", "Backlash OUT Steps", "%.0f", 0, MaxControllerSettingValue, 1, 0);
    BacklashNP.fill(getDeviceName(), "GEMINIASTRO_BACKLASH", "GeminiAstro Backlash", MAIN_CONTROL_TAB, IP_RW, 0,
                    IPS_IDLE);

    BacklashSP[0].fill("BACKLASH_IN_ENABLE", "Backlash IN Enable", ISS_OFF);
    BacklashSP[1].fill("BACKLASH_IN_DISABLE", "Backlash IN Disable", ISS_OFF);
    BacklashSP[2].fill("BACKLASH_OUT_ENABLE", "Backlash OUT Enable", ISS_OFF);
    BacklashSP[3].fill("BACKLASH_OUT_DISABLE", "Backlash OUT Disable", ISS_OFF);
    BacklashSP.fill(getDeviceName(), "GEMINIASTRO_BACKLASH_SWITCHES", "GeminiAstro Backlash Switches",
                    MAIN_CONTROL_TAB, IP_RW, ISR_ATMOST1, 0, IPS_IDLE);

    DisplayNP[0].fill("LCD_PAGE_DISPLAY_TIME", "LCD Page Time", "%.0f", 2000, 4000, 1000, 2000);
    DisplayNP[1].fill("LCD_PAGE_OPTION", "LCD Page Option", "%.0f", 0, MaxControllerSettingValue, 1, 0);
    DisplayNP.fill(getDeviceName(), "GEMINIASTRO_DISPLAY", "GeminiAstro Display", MAIN_CONTROL_TAB, IP_RW, 0,
                   IPS_IDLE);

    DisplaySP[0].fill("LCD_ENABLE", "LCD Enable", ISS_OFF);
    DisplaySP[1].fill("LCD_DISABLE", "LCD Disable", ISS_OFF);
    DisplaySP[2].fill("UPDATE_WHILE_MOVING_ENABLE", "Update While Moving Enable", ISS_OFF);
    DisplaySP[3].fill("UPDATE_WHILE_MOVING_DISABLE", "Update While Moving Disable", ISS_OFF);
    DisplaySP[4].fill("DELAYED_UPDATE_ENABLE", "Delayed Update Enable", ISS_OFF);
    DisplaySP[5].fill("DELAYED_UPDATE_DISABLE", "Delayed Update Disable", ISS_OFF);
    DisplaySP.fill(getDeviceName(), "GEMINIASTRO_DISPLAY_SWITCHES", "GeminiAstro Display Switches",
                   MAIN_CONTROL_TAB, IP_RW, ISR_ATMOST1, 0, IPS_IDLE);

    PersistenceSP[0].fill("WRITE_EEPROM", "Write EEPROM", ISS_OFF);
    PersistenceSP[1].fill("WRITE_DEFAULTS", "Write Defaults", ISS_OFF);
    PersistenceSP[2].fill("RESET_CONTROLLER", "Reset Controller", ISS_OFF);
    PersistenceSP.fill(getDeviceName(), "GEMINIASTRO_PERSISTENCE", "GeminiAstro Persistence", MAIN_CONTROL_TAB,
                       IP_RW, ISR_ATMOST1, 0, IPS_IDLE);

    setDefaultPollingPeriod(500);
    addDebugControl();

    return true;
}

bool GeminiAstroFocuser::updateProperties()
{
    INDI::Focuser::updateProperties();

    if (isConnected())
    {
        defineProperty(ControllerInfoTP);
        defineProperty(TemperatureNP);
        defineProperty(SafeLimitsNP);
        loadConfig(SafeLimitsNP);
        defineProperty(LimitStatusNP);
        defineProperty(CalibrationStepNP);
        defineProperty(CalibrationStatusNP);
        defineProperty(CalibrationSP);
        defineProperty(ControllerStatusNP);
        defineProperty(MotorSettingsNP);
        defineProperty(MotorSwitchSP);
        defineProperty(JogSP);
        defineProperty(TemperatureCompNP);
        defineProperty(TemperatureCompSP);
        defineProperty(BacklashNP);
        defineProperty(BacklashSP);
        defineProperty(DisplayNP);
        defineProperty(DisplaySP);
        defineProperty(PersistenceSP);
        readStartupValues();
        LOG_INFO("GeminiAstro focuser is ready.");
    }
    else
    {
        deleteProperty(ControllerInfoTP.getName());
        deleteProperty(TemperatureNP.getName());
        deleteProperty(SafeLimitsNP.getName());
        deleteProperty(LimitStatusNP.getName());
        deleteProperty(CalibrationStepNP.getName());
        deleteProperty(CalibrationStatusNP.getName());
        deleteProperty(CalibrationSP.getName());
        deleteProperty(ControllerStatusNP.getName());
        deleteProperty(MotorSettingsNP.getName());
        deleteProperty(MotorSwitchSP.getName());
        deleteProperty(JogSP.getName());
        deleteProperty(TemperatureCompNP.getName());
        deleteProperty(TemperatureCompSP.getName());
        deleteProperty(BacklashNP.getName());
        deleteProperty(BacklashSP.getName());
        deleteProperty(DisplayNP.getName());
        deleteProperty(DisplaySP.getName());
        deleteProperty(PersistenceSP.getName());
    }

    return true;
}

bool GeminiAstroFocuser::Handshake()
{
    for (int attempt = 0; attempt < 3; ++attempt)
    {
        std::string response;
        if (sendCommand(geminiastro::handshakeCommand(), &response) && geminiastro::parseHandshake(response))
        {
            LOG_INFO("GeminiAstro focuser handshake succeeded.");
            return true;
        }
        pulseSerialControlLines();
        sleep(1);
    }

    LOG_ERROR("GeminiAstro focuser did not answer handshake. Check power, serial "
              "port, and baud rate.");
    return false;
}

IPState GeminiAstroFocuser::MoveAbsFocuser(uint32_t targetTicks)
{
    const auto controllerClamped = geminiastro::clampPosition(targetTicks, controllerMaxPosition_);
    targetPosition_ = clampToEffectiveLimits(controllerClamped);

    if (targetPosition_ != targetTicks)
        LOGF_WARN("Requested absolute target %u adjusted to safe target %u.", targetTicks, targetPosition_);

    if (!moveAbsolute(targetPosition_))
        return IPS_ALERT;

    FocusAbsPosN[0].value = targetPosition_;
    FocusAbsPosNP.s = IPS_BUSY;
    return IPS_BUSY;
}

IPState GeminiAstroFocuser::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    uint32_t current = 0;
    if (!readPosition(current))
        return IPS_ALERT;

    const int64_t offset = (dir == FOCUS_INWARD) ? -static_cast<int64_t>(ticks) : static_cast<int64_t>(ticks);
    const auto controllerClamped =
        geminiastro::clampPosition(static_cast<int64_t>(current) + offset, controllerMaxPosition_);
    targetPosition_ = clampToEffectiveLimits(controllerClamped);

    if (targetPosition_ != controllerClamped)
        LOGF_WARN("Requested relative move adjusted to safe target %u.", targetPosition_);

    if (!moveAbsolute(targetPosition_))
        return IPS_ALERT;

    FocusRelPosN[0].value = ticks;
    FocusRelPosNP.s = IPS_BUSY;
    FocusAbsPosN[0].value = targetPosition_;
    FocusAbsPosNP.s = IPS_BUSY;
    return IPS_BUSY;
}

bool GeminiAstroFocuser::AbortFocuser()
{
    if (!sendCommand(geminiastro::abortCommand(), nullptr))
        return false;

    FocusAbsPosNP.s = IPS_IDLE;
    FocusRelPosNP.s = IPS_IDLE;
    return true;
}

bool GeminiAstroFocuser::SyncFocuser(uint32_t ticks)
{
    const auto clamped = geminiastro::clampPosition(ticks, controllerMaxPosition_);

    if (!sendCommand(geminiastro::syncPositionCommand(clamped), nullptr))
        return false;

    FocusAbsPosN[0].value = clamped;
    IDSetNumber(&FocusAbsPosNP, nullptr);
    lastPosition_ = clamped;
    clearConfiguredSafeLimits();
    resetSafeLimitsToTemporaryWindow(clamped);
    resetCalibration();
    SafeLimitsNP.apply("Logical position synchronized; configured safe limits were cleared and must be recalibrated.");
    updateLimitStatus();
    updateCalibrationStatus(clamped, true);
    return true;
}

bool GeminiAstroFocuser::SetFocuserMaxPosition(uint32_t ticks)
{
    SafeLimitsNP[1].setValue(geminiastro::clampPosition(ticks, controllerMaxPosition_));
    if (!updateConfiguredSafeLimits())
        return false;

    SafeLimitsNP.apply();
    return true;
}

void GeminiAstroFocuser::TimerHit()
{
    if (!isConnected())
        return;

    uint32_t position = 0;
    bool havePosition = false;
    if (readPosition(position))
    {
        havePosition = true;
        FocusAbsPosN[0].value = position;
        if (std::abs(static_cast<int>(position) - static_cast<int>(lastPosition_)) > 2)
        {
            IDSetNumber(&FocusAbsPosNP, nullptr);
            lastPosition_ = position;
        }
        updateCalibrationStatus(position, true);
    }

    double temperature = 0;
    if (readTemperature(temperature))
    {
        if (std::abs(TemperatureNP[0].getValue() - temperature) >= 0.1)
        {
            TemperatureNP[0].setValue(temperature);
            TemperatureNP.apply();
        }
    }

    bool moving = false;
    if (readMoving(moving))
    {
        const bool wasMoving = lastMoving_;
        lastMoving_ = moving;
        if (moving && havePosition && hasUsableEffectiveLimits() &&
            (position <= effectiveSafeMinPosition_ || position >= effectiveSafeMaxPosition_))
        {
            LOGF_WARN("Focuser reached effective safe-limit edge at %u; aborting motion.", position);
            AbortFocuser();
            FocusAbsPosNP.s = IPS_ALERT;
            FocusRelPosNP.s = IPS_ALERT;
            IDSetNumber(&FocusAbsPosNP, nullptr);
            IDSetNumber(&FocusRelPosNP, nullptr);
            lastMoving_ = false;
        }
        if (!moving && (FocusAbsPosNP.s == IPS_BUSY || FocusRelPosNP.s == IPS_BUSY))
        {
            FocusAbsPosNP.s = IPS_OK;
            FocusRelPosNP.s = IPS_OK;
            IDSetNumber(&FocusAbsPosNP, nullptr);
            IDSetNumber(&FocusRelPosNP, nullptr);
            LOG_INFO("GeminiAstro focuser reached the requested position.");
        }

        bool homeSwitchClosed = false;
        if (readHomeSwitch(homeSwitchClosed))
        {
            if (haveHomeSwitchState_ && wasMoving && homeSwitchClosed != lastHomeSwitchClosed_)
            {
                AbortFocuser();
                uint32_t limitPosition = position;
                readPosition(limitPosition);
                if (targetPosition_ <= position)
                    effectiveSafeMinPosition_ = std::max(effectiveSafeMinPosition_, limitPosition);
                else
                    effectiveSafeMaxPosition_ = std::min(effectiveSafeMaxPosition_, limitPosition);
                applyEffectiveLimitsToProperties();
                LOGF_WARN("Home-switch state changed during motion; aborting and "
                          "narrowing effective range at %u.",
                          limitPosition);
            }
            lastHomeSwitchClosed_ = homeSwitchClosed;
            haveHomeSwitchState_ = true;
        }

        bool stepperPowered = false;
        if (readStepperPower(stepperPowered))
        {
            lastStepperPowered_ = stepperPowered;
            haveStepperPowerState_ = true;
        }

        updateLimitStatus();
    }

    SetTimer(getCurrentPollingPeriod());
}

bool GeminiAstroFocuser::saveConfigItems(FILE *fp)
{
    INDI::Focuser::saveConfigItems(fp);
    SafeLimitsNP.save(fp);
    return true;
}

bool GeminiAstroFocuser::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev != nullptr && std::strcmp(dev, getDeviceName()) == 0 && SafeLimitsNP.isNameMatch(name))
    {
        SafeLimitsNP.update(values, names, n);
        if (!updateConfiguredSafeLimits())
        {
            SafeLimitsNP.setState(IPS_ALERT);
            SafeLimitsNP.apply("Invalid safe limits. Use min < max, max <= controller max, and "
                               "guard smaller than half the range.");
            return true;
        }

        SafeLimitsNP.setState(IPS_OK);
        SafeLimitsNP.apply();
        return true;
    }

    if (dev != nullptr && std::strcmp(dev, getDeviceName()) == 0 && CalibrationStepNP.isNameMatch(name))
    {
        CalibrationStepNP.update(values, names, n);
        const auto rawStep = CalibrationStepNP[0].getValue();
        auto step = std::isfinite(rawStep) ? static_cast<uint32_t>(std::lround(rawStep)) : DefaultCalibrationStep;
        step = std::clamp<uint32_t>(step, 1, maxCalibrationStep());
        CalibrationStepNP[0].setValue(step);
        CalibrationStepNP.setState(IPS_OK);
        CalibrationStepNP.apply();
        return true;
    }

    if (dev != nullptr && std::strcmp(dev, getDeviceName()) == 0 && MotorSettingsNP.isNameMatch(name))
    {
        MotorSettingsNP.update(values, names, n);
        const bool ok = updateMotorSettings(names, n);
        MotorSettingsNP.setState(ok ? IPS_OK : IPS_ALERT);
        MotorSettingsNP.apply(ok ? "GeminiAstro motor setting applied." : "GeminiAstro motor setting failed.");
        return true;
    }

    if (dev != nullptr && std::strcmp(dev, getDeviceName()) == 0 && TemperatureCompNP.isNameMatch(name))
    {
        TemperatureCompNP.update(values, names, n);
        const bool ok = updateTemperatureCompSettings(names, n);
        TemperatureCompNP.setState(ok ? IPS_OK : IPS_ALERT);
        TemperatureCompNP.apply(ok ? "GeminiAstro temperature compensation setting applied."
                                   : "GeminiAstro temperature compensation setting failed.");
        return true;
    }

    if (dev != nullptr && std::strcmp(dev, getDeviceName()) == 0 && BacklashNP.isNameMatch(name))
    {
        BacklashNP.update(values, names, n);
        const bool ok = updateBacklashSettings(names, n);
        BacklashNP.setState(ok ? IPS_OK : IPS_ALERT);
        BacklashNP.apply(ok ? "GeminiAstro backlash setting applied." : "GeminiAstro backlash setting failed.");
        return true;
    }

    if (dev != nullptr && std::strcmp(dev, getDeviceName()) == 0 && DisplayNP.isNameMatch(name))
    {
        DisplayNP.update(values, names, n);
        const bool ok = updateDisplaySettings(names, n);
        DisplayNP.setState(ok ? IPS_OK : IPS_ALERT);
        DisplayNP.apply(ok ? "GeminiAstro display setting applied." : "GeminiAstro display setting failed.");
        return true;
    }

    return INDI::Focuser::ISNewNumber(dev, name, values, names, n);
}

bool GeminiAstroFocuser::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && std::strcmp(dev, getDeviceName()) == 0 && CalibrationSP.isNameMatch(name))
    {
        CalibrationSP.update(states, names, n);

        bool ok = true;
        const char *message = nullptr;

        if (CalibrationSP[0].getState() == ISS_ON)
        {
            ok = nudgeCalibration(FOCUS_INWARD);
            message = ok ? "Calibration nudge inward started." : "Calibration nudge inward failed.";
        }
        else if (CalibrationSP[1].getState() == ISS_ON)
        {
            ok = nudgeCalibration(FOCUS_OUTWARD);
            message = ok ? "Calibration nudge outward started." : "Calibration nudge outward failed.";
        }
        else if (CalibrationSP[2].getState() == ISS_ON)
        {
            ok = captureCalibrationPoint(true);
            message = ok ? "Captured candidate minimum safe-limit position." : "Could not capture candidate minimum.";
        }
        else if (CalibrationSP[3].getState() == ISS_ON)
        {
            ok = captureCalibrationPoint(false);
            message = ok ? "Captured candidate maximum safe-limit position." : "Could not capture candidate maximum.";
        }
        else if (CalibrationSP[4].getState() == ISS_ON)
        {
            ok = applyCalibrationLimits();
            message = ok ? "Applied calibrated safe limits." : "Calibrated safe limits are incomplete or invalid.";
        }
        else if (CalibrationSP[5].getState() == ISS_ON)
        {
            resetCalibration();
            ok = true;
            message = "Reset candidate safe-limit positions.";
        }
        else if (CalibrationSP[6].getState() == ISS_ON)
        {
            ok = centerCalibrationPosition();
            message = ok ? "Synchronized current physical position to controller midpoint."
                         : "Could not synchronize current position to controller midpoint.";
        }

        CalibrationSP.reset();
        CalibrationSP.setState(ok ? IPS_OK : IPS_ALERT);
        CalibrationSP.apply("%s", message != nullptr ? message : "No calibration action selected.");
        return true;
    }

    if (dev != nullptr && std::strcmp(dev, getDeviceName()) == 0 && MotorSwitchSP.isNameMatch(name))
    {
        MotorSwitchSP.update(states, names, n);
        const bool ok = handleMotorSwitch();
        MotorSwitchSP.reset();
        MotorSwitchSP.setState(ok ? IPS_OK : IPS_ALERT);
        MotorSwitchSP.apply(ok ? "GeminiAstro motor switch command sent." : "GeminiAstro motor switch command failed.");
        return true;
    }

    if (dev != nullptr && std::strcmp(dev, getDeviceName()) == 0 && JogSP.isNameMatch(name))
    {
        JogSP.update(states, names, n);
        const bool ok = handleJogSwitch();
        JogSP.reset();
        JogSP.setState(ok ? IPS_OK : IPS_ALERT);
        JogSP.apply(ok ? "GeminiAstro jog command sent." : "GeminiAstro jog command failed.");
        return true;
    }

    if (dev != nullptr && std::strcmp(dev, getDeviceName()) == 0 && TemperatureCompSP.isNameMatch(name))
    {
        TemperatureCompSP.update(states, names, n);
        const bool ok = handleTemperatureCompSwitch();
        TemperatureCompSP.reset();
        TemperatureCompSP.setState(ok ? IPS_OK : IPS_ALERT);
        TemperatureCompSP.apply(ok ? "GeminiAstro temperature compensation command sent."
                                   : "GeminiAstro temperature compensation command failed.");
        return true;
    }

    if (dev != nullptr && std::strcmp(dev, getDeviceName()) == 0 && BacklashSP.isNameMatch(name))
    {
        BacklashSP.update(states, names, n);
        const bool ok = handleBacklashSwitch();
        BacklashSP.reset();
        BacklashSP.setState(ok ? IPS_OK : IPS_ALERT);
        BacklashSP.apply(ok ? "GeminiAstro backlash command sent." : "GeminiAstro backlash command failed.");
        return true;
    }

    if (dev != nullptr && std::strcmp(dev, getDeviceName()) == 0 && DisplaySP.isNameMatch(name))
    {
        DisplaySP.update(states, names, n);
        const bool ok = handleDisplaySwitch();
        DisplaySP.reset();
        DisplaySP.setState(ok ? IPS_OK : IPS_ALERT);
        DisplaySP.apply(ok ? "GeminiAstro display command sent." : "GeminiAstro display command failed.");
        return true;
    }

    if (dev != nullptr && std::strcmp(dev, getDeviceName()) == 0 && PersistenceSP.isNameMatch(name))
    {
        PersistenceSP.update(states, names, n);
        const bool ok = handlePersistenceSwitch();
        PersistenceSP.reset();
        PersistenceSP.setState(ok ? IPS_OK : IPS_ALERT);
        PersistenceSP.apply(ok ? "GeminiAstro persistence command sent." : "GeminiAstro persistence command failed.");
        return true;
    }

    return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}

bool GeminiAstroFocuser::sendCommand(const std::string &command, std::string *response)
{
    int bytesWritten = 0;
    int rc = -1;

    tcflush(PortFD, TCIOFLUSH);
    LOGF_DEBUG("CMD <%s>", command.c_str());

    rc = tty_write_string(PortFD, command.c_str(), &bytesWritten);
    if (rc != TTY_OK)
    {
        char error[MAXRBUF] = {0};
        tty_error_msg(rc, error, MAXRBUF);
        LOGF_ERROR("Serial write error: %s.", error);
        handleCommunicationFailure("serial write");
        return false;
    }

    if (response == nullptr)
    {
        tcdrain(PortFD);
        return true;
    }

    char buffer[ResponseSize] = {0};
    int bytesRead = 0;
    rc = tty_nread_section(PortFD, buffer, ResponseSize, geminiastro::Terminator, SerialTimeoutSeconds, &bytesRead);
    if (rc != TTY_OK)
    {
        char error[MAXRBUF] = {0};
        tty_error_msg(rc, error, MAXRBUF);
        LOGF_ERROR("Serial read error: %s.", error);
        handleCommunicationFailure("serial read");
        return false;
    }

    *response = buffer;
    LOGF_DEBUG("RES <%s>", response->c_str());
    tcflush(PortFD, TCIOFLUSH);
    return true;
}

void GeminiAstroFocuser::pulseSerialControlLines()
{
    int both = TIOCM_DTR | TIOCM_RTS;
    if (ioctl(PortFD, TIOCMBIS, &both) == -1)
    {
        LOG_DEBUG("Could not assert GeminiAstro serial DTR/RTS control lines.");
        return;
    }

    usleep(500000);

    int rts = TIOCM_RTS;
    if (ioctl(PortFD, TIOCMBIC, &rts) == -1)
        LOG_DEBUG("Could not clear GeminiAstro serial RTS control line.");

    usleep(1500000);

    int dtr = TIOCM_DTR;
    if (ioctl(PortFD, TIOCMBIC, &dtr) == -1)
        LOG_DEBUG("Could not clear GeminiAstro serial DTR control line.");

    usleep(1500000);
}

void GeminiAstroFocuser::handleCommunicationFailure(const char *context)
{
    if (!isConnected())
        return;

    LOGF_ERROR("GeminiAstro communication failure during %s; disconnecting driver.", context);
    INDI::Focuser::Disconnect();
    setConnected(false, IPS_ALERT, "GeminiAstro serial communication failed; reconnect required.");
}

bool GeminiAstroFocuser::readStartupValues()
{
    std::string firmware;
    if (readFirmware(firmware))
    {
        ControllerInfoTP[0].setText(firmware);
        LOGF_INFO("GeminiAstro firmware revision: %s", firmware.c_str());
    }

    std::string model;
    if (readControllerModel(model))
    {
        ControllerInfoTP[1].setText(model);
        LOGF_INFO("GeminiAstro controller model: %s", model.c_str());
    }

    uint32_t maxPosition = 0;
    if (readMaxPosition(maxPosition))
    {
        applyControllerMaxPosition(maxPosition);
        LOGF_INFO("GeminiAstro max position: %u", maxPosition);
    }

    uint32_t maxIncrement = 0;
    if (readMaxIncrement(maxIncrement))
    {
        controllerMaxIncrement_ = std::max<uint32_t>(1, maxIncrement);
        ControllerStatusNP[0].setValue(maxIncrement);
        LOGF_INFO("GeminiAstro max increment: %u", maxIncrement);
    }
    CalibrationStepNP[0].setMinMax(1, maxCalibrationStep());

    uint32_t position = 0;
    bool havePosition = false;
    if (readPosition(position))
    {
        FocusAbsPosN[0].value = position;
        lastPosition_ = position;
        havePosition = true;
    }

    initializeSafeLimits(controllerMaxPosition_, position);

    double temperature = 0;
    if (readTemperature(temperature))
        TemperatureNP[0].setValue(temperature);

    bool homeSwitchClosed = false;
    if (readHomeSwitch(homeSwitchClosed))
    {
        lastHomeSwitchClosed_ = homeSwitchClosed;
        haveHomeSwitchState_ = true;
        LOGF_INFO("GeminiAstro home-switch state: %s.", homeSwitchClosed ? "closed" : "open");
    }

    bool stepperPowered = false;
    if (readStepperPower(stepperPowered))
    {
        lastStepperPowered_ = stepperPowered;
        haveStepperPowerState_ = true;
        LOGF_INFO("GeminiAstro stepper power state: %s.", stepperPowered ? "powered" : "off");
    }

    readControllerDiagnostics();
    updateLimitStatus();
    updateCalibrationStatus(position, havePosition);

    IDSetNumber(&FocusAbsPosNP, nullptr);
    ControllerInfoTP.apply();
    TemperatureNP.apply();
    SafeLimitsNP.apply();
    LimitStatusNP.apply();
    CalibrationStepNP.apply();
    CalibrationStatusNP.apply();
    ControllerStatusNP.apply();
    MotorSettingsNP.apply();
    TemperatureCompNP.apply();
    BacklashNP.apply();
    DisplayNP.apply();

    return true;
}

bool GeminiAstroFocuser::readFirmware(std::string &firmware)
{
    std::string response;
    return sendCommand(geminiastro::firmwareCommand(), &response) && geminiastro::parseFirmware(response, firmware);
}

bool GeminiAstroFocuser::readControllerModel(std::string &model)
{
    std::string response;
    return sendCommand(geminiastro::controllerModelCommand(), &response) &&
           geminiastro::parseControllerModel(response, model);
}

bool GeminiAstroFocuser::readMaxPosition(uint32_t &maxPosition)
{
    std::string response;
    return sendCommand(geminiastro::maxPositionCommand(), &response) &&
           geminiastro::parseMaxPosition(response, maxPosition);
}

bool GeminiAstroFocuser::readMaxIncrement(uint32_t &increment)
{
    std::string response;
    return sendCommand(geminiastro::maxIncrementCommand(), &response) &&
           geminiastro::parseMaxIncrement(response, increment);
}

bool GeminiAstroFocuser::readPosition(uint32_t &position)
{
    std::string response;
    return sendCommand(geminiastro::positionCommand(), &response) && geminiastro::parsePosition(response, position);
}

bool GeminiAstroFocuser::readMoving(bool &moving)
{
    std::string response;
    return sendCommand(geminiastro::movingCommand(), &response) && geminiastro::parseMoving(response, moving);
}

bool GeminiAstroFocuser::readTemperature(double &temperatureC)
{
    std::string response;
    return sendCommand(geminiastro::temperatureCommand(), &response) &&
           geminiastro::parseTemperature(response, temperatureC);
}

bool GeminiAstroFocuser::readCoilPower(bool &enabled)
{
    std::string response;
    return sendCommand(geminiastro::coilPowerCommand(), &response) && geminiastro::parseCoilPower(response, enabled);
}

bool GeminiAstroFocuser::readReverse(bool &enabled)
{
    std::string response;
    return sendCommand(geminiastro::reverseCommand(), &response) && geminiastro::parseReverse(response, enabled);
}

bool GeminiAstroFocuser::readTemperaturePrecision(uint32_t &precision)
{
    std::string response;
    return sendCommand(geminiastro::temperaturePrecisionCommand(), &response) &&
           geminiastro::parseTemperaturePrecision(response, precision);
}

bool GeminiAstroFocuser::readTemperatureCompEnabled(bool &enabled)
{
    std::string response;
    return sendCommand(geminiastro::temperatureCompEnabledCommand(), &response) &&
           geminiastro::parseTemperatureCompEnabled(response, enabled);
}

bool GeminiAstroFocuser::readTemperatureCompAvailable(bool &available)
{
    std::string response;
    return sendCommand(geminiastro::temperatureCompAvailableCommand(), &response) &&
           geminiastro::parseTemperatureCompAvailable(response, available);
}

bool GeminiAstroFocuser::readTemperatureCompValue(uint32_t &steps)
{
    std::string response;
    return sendCommand(geminiastro::temperatureCompValueCommand(), &response) &&
           geminiastro::parseTemperatureCompValue(response, steps);
}

bool GeminiAstroFocuser::readTemperatureProbeAvailable(bool &available)
{
    std::string response;
    return sendCommand(geminiastro::temperatureProbeAvailableCommand(), &response) &&
           geminiastro::parseTemperatureProbeAvailable(response, available);
}

bool GeminiAstroFocuser::readTemperatureCompOption(bool &enabled)
{
    std::string response;
    return sendCommand(geminiastro::temperatureCompOptionCommand(), &response) &&
           geminiastro::parseTemperatureCompOption(response, enabled);
}

bool GeminiAstroFocuser::readTemperatureCompDirection(bool &outward)
{
    std::string response;
    return sendCommand(geminiastro::temperatureCompDirectionCommand(), &response) &&
           geminiastro::parseTemperatureCompDirection(response, outward);
}

bool GeminiAstroFocuser::readHomeSwitch(bool &closed)
{
    std::string response;
    return sendCommand(geminiastro::homeSwitchCommand(), &response) && geminiastro::parseHomeSwitch(response, closed);
}

bool GeminiAstroFocuser::readJogEnabled(bool &enabled)
{
    std::string response;
    return sendCommand(geminiastro::jogEnabledCommand(), &response) && geminiastro::parseJogEnabled(response, enabled);
}

bool GeminiAstroFocuser::readJogDirection(bool &outward)
{
    std::string response;
    return sendCommand(geminiastro::jogDirectionCommand(), &response) &&
           geminiastro::parseJogDirection(response, outward);
}

bool GeminiAstroFocuser::readStepperPower(bool &powered)
{
    std::string response;
    return sendCommand(geminiastro::stepperPowerCommand(), &response) &&
           geminiastro::parseStepperPower(response, powered);
}

bool GeminiAstroFocuser::readStepMode(uint32_t &mode)
{
    std::string response;
    return sendCommand(geminiastro::stepModeCommand(), &response) && geminiastro::parseStepMode(response, mode);
}

bool GeminiAstroFocuser::readStepSizeEnabled(bool &enabled)
{
    std::string response;
    return sendCommand(geminiastro::stepSizeEnabledCommand(), &response) &&
           geminiastro::parseStepSizeEnabled(response, enabled);
}

bool GeminiAstroFocuser::readStepSize(double &size)
{
    std::string response;
    return sendCommand(geminiastro::stepSizeCommand(), &response) && geminiastro::parseStepSize(response, size);
}

bool GeminiAstroFocuser::readMotorSpeed(uint32_t &speed)
{
    std::string response;
    return sendCommand(geminiastro::motorSpeedCommand(), &response) && geminiastro::parseMotorSpeed(response, speed);
}

bool GeminiAstroFocuser::readDelayAfterMove(uint32_t &delay)
{
    std::string response;
    return sendCommand(geminiastro::delayAfterMoveCommand(), &response) &&
           geminiastro::parseDelayAfterMove(response, delay);
}

bool GeminiAstroFocuser::readBacklashInEnabled(bool &enabled)
{
    std::string response;
    return sendCommand(geminiastro::backlashInEnabledCommand(), &response) &&
           geminiastro::parseBacklashInEnabled(response, enabled);
}

bool GeminiAstroFocuser::readBacklashOutEnabled(bool &enabled)
{
    std::string response;
    return sendCommand(geminiastro::backlashOutEnabledCommand(), &response) &&
           geminiastro::parseBacklashOutEnabled(response, enabled);
}

bool GeminiAstroFocuser::readBacklashInSteps(uint32_t &steps)
{
    std::string response;
    return sendCommand(geminiastro::backlashInStepsCommand(), &response) &&
           geminiastro::parseBacklashInSteps(response, steps);
}

bool GeminiAstroFocuser::readBacklashOutSteps(uint32_t &steps)
{
    std::string response;
    return sendCommand(geminiastro::backlashOutStepsCommand(), &response) &&
           geminiastro::parseBacklashOutSteps(response, steps);
}

bool GeminiAstroFocuser::readDisplayStatus(bool &enabled)
{
    std::string response;
    return sendCommand(geminiastro::displayStatusCommand(), &response) &&
           geminiastro::parseDisplayStatus(response, enabled);
}

bool GeminiAstroFocuser::readLcdPageDisplayTime(uint32_t &milliseconds)
{
    std::string response;
    return sendCommand(geminiastro::lcdPageDisplayTimeCommand(), &response) &&
           geminiastro::parseLcdPageDisplayTime(response, milliseconds);
}

bool GeminiAstroFocuser::readLcdUpdateWhileMoving(bool &enabled)
{
    std::string response;
    return sendCommand(geminiastro::lcdUpdateWhileMovingCommand(), &response) &&
           geminiastro::parseLcdUpdateWhileMoving(response, enabled);
}

bool GeminiAstroFocuser::moveAbsolute(uint32_t position)
{
    return sendCommand(geminiastro::moveAbsoluteCommand(position), nullptr);
}

bool GeminiAstroFocuser::sendControllerWrite(const std::string &command)
{
    if (command.empty())
    {
        LOG_ERROR("GeminiAstro controller command rejected: empty command.");
        return false;
    }

    if (!sendCommand(command, nullptr))
        return false;

    readControllerDiagnostics();
    ControllerStatusNP.apply();
    MotorSettingsNP.apply();
    TemperatureCompNP.apply();
    BacklashNP.apply();
    DisplayNP.apply();
    return true;
}

bool GeminiAstroFocuser::sendControllerWriteSequence(const std::string &first, const std::string &second)
{
    if (first.empty() || second.empty())
    {
        LOG_ERROR("GeminiAstro controller command sequence rejected: empty command.");
        return false;
    }

    if (!sendCommand(first, nullptr) || !sendCommand(second, nullptr))
        return false;

    readControllerDiagnostics();
    ControllerStatusNP.apply();
    MotorSettingsNP.apply();
    TemperatureCompNP.apply();
    BacklashNP.apply();
    DisplayNP.apply();
    return true;
}

void GeminiAstroFocuser::readControllerDiagnostics()
{
    uint32_t number = 0;
    double decimal = 0;
    bool flag = false;

    if (readMaxIncrement(number))
    {
        controllerMaxIncrement_ = std::max<uint32_t>(1, number);
        CalibrationStepNP[0].setMinMax(1, maxCalibrationStep());
        ControllerStatusNP[0].setValue(number);
    }
    if (readCoilPower(flag))
        ControllerStatusNP[1].setValue(flag ? 1 : 0);
    if (readReverse(flag))
        ControllerStatusNP[2].setValue(flag ? 1 : 0);
    if (readTemperaturePrecision(number))
    {
        ControllerStatusNP[3].setValue(number);
        TemperatureCompNP[0].setValue(number);
    }
    if (readTemperatureCompEnabled(flag))
        ControllerStatusNP[4].setValue(flag ? 1 : 0);
    if (readTemperatureCompAvailable(flag))
        ControllerStatusNP[5].setValue(flag ? 1 : 0);
    if (readTemperatureCompValue(number))
    {
        ControllerStatusNP[6].setValue(number);
        TemperatureCompNP[1].setValue(number);
    }
    if (readTemperatureProbeAvailable(flag))
        ControllerStatusNP[7].setValue(flag ? 1 : 0);
    if (readTemperatureCompOption(flag))
        ControllerStatusNP[8].setValue(flag ? 1 : 0);
    if (readTemperatureCompDirection(flag))
        ControllerStatusNP[9].setValue(flag ? 1 : 0);
    if (readJogEnabled(flag))
        ControllerStatusNP[10].setValue(flag ? 1 : 0);
    if (readJogDirection(flag))
        ControllerStatusNP[11].setValue(flag ? 1 : 0);
    if (readStepMode(number))
    {
        ControllerStatusNP[12].setValue(number);
        MotorSettingsNP[2].setValue(number);
    }
    if (readStepSizeEnabled(flag))
        ControllerStatusNP[13].setValue(flag ? 1 : 0);
    if (readStepSize(decimal))
    {
        ControllerStatusNP[14].setValue(decimal);
        MotorSettingsNP[3].setValue(decimal);
    }
    if (readMotorSpeed(number))
    {
        ControllerStatusNP[15].setValue(number);
        MotorSettingsNP[1].setValue(number);
    }
    if (readDelayAfterMove(number))
    {
        ControllerStatusNP[16].setValue(number);
        MotorSettingsNP[4].setValue(number);
    }
    if (readBacklashInEnabled(flag))
        ControllerStatusNP[17].setValue(flag ? 1 : 0);
    if (readBacklashOutEnabled(flag))
        ControllerStatusNP[18].setValue(flag ? 1 : 0);
    if (readBacklashInSteps(number))
    {
        ControllerStatusNP[19].setValue(number);
        BacklashNP[0].setValue(number);
    }
    if (readBacklashOutSteps(number))
    {
        ControllerStatusNP[20].setValue(number);
        BacklashNP[1].setValue(number);
    }
    if (readDisplayStatus(flag))
        ControllerStatusNP[21].setValue(flag ? 1 : 0);
    if (readLcdPageDisplayTime(number))
    {
        ControllerStatusNP[22].setValue(number);
        DisplayNP[0].setValue(number);
    }
    if (readLcdUpdateWhileMoving(flag))
        ControllerStatusNP[23].setValue(flag ? 1 : 0);

    ControllerStatusNP.setState(IPS_OK);
}

void GeminiAstroFocuser::applyControllerMaxPosition(uint32_t maxPosition)
{
    controllerMaxPosition_ = std::max<uint32_t>(1, maxPosition);
    MotorSettingsNP[0].setValue(controllerMaxPosition_);
    MotorSettingsNP[0].setMinMax(1, MaxControllerSettingValue);
    SafeLimitsNP[0].setMinMax(0, controllerMaxPosition_);
    SafeLimitsNP[1].setMinMax(0, controllerMaxPosition_);
    LimitStatusNP[0].setMinMax(0, controllerMaxPosition_);
    LimitStatusNP[1].setMinMax(0, controllerMaxPosition_);
    CalibrationStatusNP[0].setMinMax(0, controllerMaxPosition_);
    CalibrationStatusNP[1].setMinMax(-1, controllerMaxPosition_);
    CalibrationStatusNP[2].setMinMax(-1, controllerMaxPosition_);
    CalibrationStatusNP[4].setMinMax(-1, controllerMaxPosition_);
    CalibrationStepNP[0].setMinMax(1, maxCalibrationStep());
}

void GeminiAstroFocuser::initializeSafeLimits(uint32_t controllerMaxPosition, uint32_t currentPosition)
{
    controllerMaxPosition_ = std::max<uint32_t>(1, controllerMaxPosition);
    if (!updateConfiguredSafeLimits())
    {
        resetSafeLimitsToTemporaryWindow(currentPosition);
    }
}

void GeminiAstroFocuser::resetSafeLimitsToTemporaryWindow(uint32_t currentPosition)
{
    safeLimitsConfigured_ = false;
    const auto low = (currentPosition > InitialSafeWindowSteps) ? currentPosition - InitialSafeWindowSteps : 0;
    const auto high = std::min(controllerMaxPosition_, currentPosition + InitialSafeWindowSteps);
    effectiveSafeMinPosition_ = std::min(low, high);
    effectiveSafeMaxPosition_ = std::max(low, high);
    if (effectiveSafeMinPosition_ == effectiveSafeMaxPosition_)
        effectiveSafeMaxPosition_ = std::min(controllerMaxPosition_, effectiveSafeMinPosition_ + 1);

    LOGF_WARN("No configured safe limits found; using temporary startup window %u..%u around current position %u.",
              effectiveSafeMinPosition_, effectiveSafeMaxPosition_, currentPosition);
    applyEffectiveLimitsToProperties();
}

void GeminiAstroFocuser::clearConfiguredSafeLimits()
{
    SafeLimitsNP[0].setValue(0);
    SafeLimitsNP[1].setValue(0);
    safeLimitsConfigured_ = false;
}

bool GeminiAstroFocuser::updateConfiguredSafeLimits()
{
    const auto configuredMinRaw = SafeLimitsNP[0].getValue();
    const auto configuredMaxRaw = SafeLimitsNP[1].getValue();
    const auto guardStepsRaw = SafeLimitsNP[2].getValue();

    if (!std::isfinite(configuredMinRaw) || !std::isfinite(configuredMaxRaw) || !std::isfinite(guardStepsRaw) ||
        configuredMinRaw < 0 || configuredMaxRaw < 0 || guardStepsRaw < 0 ||
        configuredMinRaw > controllerMaxPosition_ || configuredMaxRaw > controllerMaxPosition_ ||
        guardStepsRaw > controllerMaxPosition_)
    {
        safeLimitsConfigured_ = false;
        return false;
    }

    const auto configuredMin = static_cast<uint32_t>(std::lround(configuredMinRaw));
    const auto configuredMax = static_cast<uint32_t>(std::lround(configuredMaxRaw));
    const auto guardSteps = static_cast<uint32_t>(std::lround(guardStepsRaw));
    if (configuredMinRaw != configuredMin || configuredMaxRaw != configuredMax || guardStepsRaw != guardSteps)
    {
        safeLimitsConfigured_ = false;
        return false;
    }

    return applyConfiguredSafeLimits(configuredMin, configuredMax, guardSteps);
}

bool GeminiAstroFocuser::applyConfiguredSafeLimits(uint32_t configuredMin, uint32_t configuredMax, uint32_t guardSteps)
{
    if (!geminiastro::validateSafeLimits(configuredMin, configuredMax, guardSteps, controllerMaxPosition_))
    {
        safeLimitsConfigured_ = false;
        return false;
    }

    safeLimitsConfigured_ = true;
    effectiveSafeMinPosition_ = geminiastro::effectiveSafeLimitMin(configuredMin, guardSteps);
    effectiveSafeMaxPosition_ = geminiastro::effectiveSafeLimitMax(configuredMax, guardSteps);
    applyEffectiveLimitsToProperties();
    LOGF_INFO("Configured safe limits active: raw %u..%u, guard %u, effective %u..%u.", configuredMin, configuredMax,
              guardSteps, effectiveSafeMinPosition_, effectiveSafeMaxPosition_);
    return true;
}

void GeminiAstroFocuser::applyEffectiveLimitsToProperties()
{
    FocusAbsPosN[0].min = effectiveSafeMinPosition_;
    FocusAbsPosN[0].max = effectiveSafeMaxPosition_;
    FocusRelPosN[0].min = 0;
    FocusRelPosN[0].max = (effectiveSafeMaxPosition_ > effectiveSafeMinPosition_)
                              ? effectiveSafeMaxPosition_ - effectiveSafeMinPosition_
                              : 0;
    FocusMaxPosN[0].value = effectiveSafeMaxPosition_;
    LimitStatusNP[0].setValue(effectiveSafeMinPosition_);
    LimitStatusNP[1].setValue(effectiveSafeMaxPosition_);
    LimitStatusNP[2].setValue(safeLimitsConfigured_ ? 1 : 0);
}

void GeminiAstroFocuser::updateLimitStatus()
{
    LimitStatusNP[0].setValue(effectiveSafeMinPosition_);
    LimitStatusNP[1].setValue(effectiveSafeMaxPosition_);
    LimitStatusNP[2].setValue(safeLimitsConfigured_ ? 1 : 0);
    LimitStatusNP[3].setValue((haveHomeSwitchState_ && lastHomeSwitchClosed_) ? 1 : 0);
    LimitStatusNP[4].setValue((haveStepperPowerState_ && lastStepperPowered_) ? 1 : 0);
    LimitStatusNP.setState(hasUsableEffectiveLimits() ? IPS_OK : IPS_ALERT);
    LimitStatusNP.apply();
}

void GeminiAstroFocuser::updateCalibrationStatus(uint32_t currentPosition, bool havePosition)
{
    if (havePosition)
        CalibrationStatusNP[0].setValue(currentPosition);

    CalibrationStatusNP[1].setValue(calibrationMinCaptured_ ? static_cast<double>(calibrationMinPosition_) : -1.0);
    CalibrationStatusNP[2].setValue(calibrationMaxCaptured_ ? static_cast<double>(calibrationMaxPosition_) : -1.0);

    uint32_t guardSteps = DefaultConfiguredGuardSteps;
    const auto guardStepsRaw = SafeLimitsNP[2].getValue();
    if (std::isfinite(guardStepsRaw) && guardStepsRaw >= 0 && guardStepsRaw <= controllerMaxPosition_ &&
        guardStepsRaw == std::lround(guardStepsRaw))
    {
        guardSteps = static_cast<uint32_t>(std::lround(guardStepsRaw));
    }
    const bool ready = calibrationMinCaptured_ && calibrationMaxCaptured_ &&
                       geminiastro::validateSafeLimits(calibrationMinPosition_, calibrationMaxPosition_, guardSteps,
                                                       controllerMaxPosition_);
    CalibrationStatusNP[3].setValue(ready ? 1 : 0);
    CalibrationStatusNP[4].setValue(haveLastCalibrationTarget_ ? static_cast<double>(lastCalibrationTarget_) : -1.0);

    if (calibrationMinCaptured_ && calibrationMaxCaptured_)
        CalibrationStatusNP.setState(ready ? IPS_OK : IPS_ALERT);
    else
        CalibrationStatusNP.setState(IPS_IDLE);

    CalibrationStatusNP.apply();
}

void GeminiAstroFocuser::updateCalibrationStatus()
{
    uint32_t position = 0;
    updateCalibrationStatus(position, readPosition(position));
}

bool GeminiAstroFocuser::readIdlePosition(uint32_t &position)
{
    bool moving = false;
    if (!readMoving(moving))
        return false;

    if (moving)
    {
        LOG_WARN("Calibration action rejected while focuser is moving.");
        return false;
    }

    return readPosition(position);
}

uint32_t GeminiAstroFocuser::calibrationStep() const
{
    const auto raw = CalibrationStepNP[0].getValue();
    if (!std::isfinite(raw))
        return DefaultCalibrationStep;

    return std::clamp<uint32_t>(static_cast<uint32_t>(std::lround(raw)), 1, maxCalibrationStep());
}

uint32_t GeminiAstroFocuser::maxCalibrationStep() const
{
    return std::max<uint32_t>(1, std::min(controllerMaxPosition_, controllerMaxIncrement_));
}

bool GeminiAstroFocuser::nudgeCalibration(FocusDirection dir)
{
    uint32_t current = 0;
    if (!readIdlePosition(current))
        return false;

    const int64_t offset =
        (dir == FOCUS_INWARD) ? -static_cast<int64_t>(calibrationStep()) : static_cast<int64_t>(calibrationStep());
    const auto target = geminiastro::clampPosition(static_cast<int64_t>(current) + offset, controllerMaxPosition_);
    if (target == current)
    {
        LOG_WARN("Calibration nudge rejected because the target equals the current "
                 "position after controller-range clamp.");
        return false;
    }

    if (!moveAbsolute(target))
        return false;

    targetPosition_ = target;
    haveLastCalibrationTarget_ = true;
    lastCalibrationTarget_ = target;
    FocusAbsPosN[0].value = target;
    FocusAbsPosNP.s = IPS_BUSY;
    IDSetNumber(&FocusAbsPosNP, nullptr);
    updateCalibrationStatus(current, true);
    LOGF_INFO("Calibration nudge started: %u -> %u.", current, target);
    return true;
}

bool GeminiAstroFocuser::captureCalibrationPoint(bool minimum)
{
    uint32_t position = 0;
    if (!readIdlePosition(position))
        return false;

    if (minimum)
    {
        calibrationMinPosition_ = position;
        calibrationMinCaptured_ = true;
        LOGF_INFO("Captured candidate safe-limit minimum at %u.", position);
    }
    else
    {
        calibrationMaxPosition_ = position;
        calibrationMaxCaptured_ = true;
        LOGF_INFO("Captured candidate safe-limit maximum at %u.", position);
    }

    updateCalibrationStatus(position, true);
    return true;
}

bool GeminiAstroFocuser::applyCalibrationLimits()
{
    uint32_t position = 0;
    if (!readIdlePosition(position))
        return false;

    if (!calibrationMinCaptured_ || !calibrationMaxCaptured_)
        return false;

    const auto guardStepsRaw = SafeLimitsNP[2].getValue();
    if (!std::isfinite(guardStepsRaw) || guardStepsRaw < 0 || guardStepsRaw > controllerMaxPosition_ ||
        guardStepsRaw != std::lround(guardStepsRaw))
    {
        updateCalibrationStatus(position, true);
        return false;
    }

    const auto guardSteps = static_cast<uint32_t>(std::lround(guardStepsRaw));
    if (!geminiastro::validateSafeLimits(calibrationMinPosition_, calibrationMaxPosition_, guardSteps,
                                         controllerMaxPosition_))
    {
        updateCalibrationStatus(position, true);
        return false;
    }

    SafeLimitsNP[0].setValue(calibrationMinPosition_);
    SafeLimitsNP[1].setValue(calibrationMaxPosition_);
    SafeLimitsNP[2].setValue(guardSteps);

    if (!applyConfiguredSafeLimits(calibrationMinPosition_, calibrationMaxPosition_, guardSteps))
        return false;

    SafeLimitsNP.setState(IPS_OK);
    SafeLimitsNP.apply();
    updateLimitStatus();
    updateCalibrationStatus(position, true);
    return true;
}

bool GeminiAstroFocuser::centerCalibrationPosition()
{
    uint32_t position = 0;
    if (!readIdlePosition(position))
        return false;

    const auto midpoint = std::max<uint32_t>(1, controllerMaxPosition_) / 2;
    return SyncFocuser(midpoint);
}

void GeminiAstroFocuser::resetCalibration()
{
    calibrationMinCaptured_ = false;
    calibrationMaxCaptured_ = false;
    haveLastCalibrationTarget_ = false;
    calibrationMinPosition_ = 0;
    calibrationMaxPosition_ = 0;
    lastCalibrationTarget_ = 0;
    updateCalibrationStatus();
}

bool GeminiAstroFocuser::updateMotorSettings(char *names[], int n)
{
    bool ok = true;
    uint32_t value = 0;
    double decimal = 0;

    if (updatedName(names, n, "CONTROLLER_MAX_POSITION"))
    {
        ok = finiteUInt(MotorSettingsNP[0].getValue(), MaxControllerSettingValue, value) &&
             sendCommand(geminiastro::setMaxPositionCommand(value), nullptr);
        if (ok && readMaxPosition(value))
        {
            applyControllerMaxPosition(value);
            initializeSafeLimits(controllerMaxPosition_, lastPosition_);
        }
        else
        {
            ok = false;
        }
    }

    if (updatedName(names, n, "MOTOR_SPEED"))
    {
        ok = finiteUInt(MotorSettingsNP[1].getValue(), MaxControllerSettingValue, value) &&
             sendControllerWrite(geminiastro::setMotorSpeedCommand(value)) && ok;
    }

    if (updatedName(names, n, "STEP_MODE"))
    {
        ok = finiteUInt(MotorSettingsNP[2].getValue(), 256, value) &&
             sendControllerWrite(geminiastro::setStepModeCommand(value)) && ok;
    }

    if (updatedName(names, n, "STEP_SIZE"))
    {
        ok = finiteDouble(MotorSettingsNP[3].getValue(), MaxControllerSettingValue, decimal) &&
             sendControllerWrite(geminiastro::setStepSizeCommand(decimal)) && ok;
    }

    if (updatedName(names, n, "DELAY_AFTER_MOVE"))
    {
        ok = finiteUInt(MotorSettingsNP[4].getValue(), MaxControllerSettingValue, value) &&
             sendControllerWrite(geminiastro::setDelayAfterMoveCommand(value)) && ok;
    }

    readControllerDiagnostics();
    ControllerStatusNP.apply();
    updateLimitStatus();
    return ok;
}

bool GeminiAstroFocuser::updateTemperatureCompSettings(char *names[], int n)
{
    bool ok = true;
    uint32_t value = 0;

    if (updatedName(names, n, "TEMPERATURE_PRECISION_CODE"))
    {
        ok = finiteUInt(TemperatureCompNP[0].getValue(), 12, value) && value >= 9 &&
             sendControllerWrite(geminiastro::setTemperaturePrecisionCommand(value)) && ok;
    }

    if (updatedName(names, n, "TEMPERATURE_COMP_STEPS"))
    {
        ok = finiteUInt(TemperatureCompNP[1].getValue(), MaxControllerSettingValue, value) &&
             sendControllerWrite(geminiastro::setTemperatureCompValueCommand(value)) && ok;
    }

    return ok;
}

bool GeminiAstroFocuser::updateBacklashSettings(char *names[], int n)
{
    bool ok = true;
    uint32_t value = 0;

    if (updatedName(names, n, "BACKLASH_IN_STEPS"))
    {
        ok = finiteUInt(BacklashNP[0].getValue(), MaxControllerSettingValue, value) &&
             sendControllerWrite(geminiastro::setBacklashInStepsCommand(value)) && ok;
    }

    if (updatedName(names, n, "BACKLASH_OUT_STEPS"))
    {
        ok = finiteUInt(BacklashNP[1].getValue(), MaxControllerSettingValue, value) &&
             sendControllerWrite(geminiastro::setBacklashOutStepsCommand(value)) && ok;
    }

    return ok;
}

bool GeminiAstroFocuser::updateDisplaySettings(char *names[], int n)
{
    bool ok = true;
    uint32_t value = 0;

    if (updatedName(names, n, "LCD_PAGE_DISPLAY_TIME"))
    {
        ok = finiteUInt(DisplayNP[0].getValue(), 4000, value) &&
             sendControllerWrite(geminiastro::setLcdPageDisplayTimeCommand(value)) && ok;
    }

    if (updatedName(names, n, "LCD_PAGE_OPTION"))
    {
        ok = finiteUInt(DisplayNP[1].getValue(), MaxControllerSettingValue, value) &&
             sendControllerWrite(geminiastro::setLcdPageOptionCommand(value)) && ok;
    }

    return ok;
}

bool GeminiAstroFocuser::handleMotorSwitch()
{
    if (MotorSwitchSP[0].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::setCoilPowerCommand(true));
    if (MotorSwitchSP[1].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::setCoilPowerCommand(false));
    if (MotorSwitchSP[2].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::setReverseCommand(true));
    if (MotorSwitchSP[3].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::setReverseCommand(false));
    if (MotorSwitchSP[4].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::setStepSizeEnabledCommand(true));
    if (MotorSwitchSP[5].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::setStepSizeEnabledCommand(false));
    if (MotorSwitchSP[6].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::setMotorSpeedSlowCommand());
    if (MotorSwitchSP[7].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::setMotorSpeedMediumCommand());
    if (MotorSwitchSP[8].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::setMotorSpeedFastCommand());

    return true;
}

bool GeminiAstroFocuser::handleJogSwitch()
{
    if (JogSP[0].getState() == ISS_ON)
    {
        targetPosition_ = effectiveSafeMinPosition_;
        const bool ok = sendControllerWrite(geminiastro::homeCommand());
        if (ok)
            FocusAbsPosNP.s = IPS_BUSY;
        return ok;
    }
    if (JogSP[1].getState() == ISS_ON)
    {
        targetPosition_ = effectiveSafeMinPosition_;
        const bool ok = sendControllerWriteSequence(geminiastro::setJogDirectionCommand(false), geminiastro::jogStartCommand());
        if (ok)
            FocusAbsPosNP.s = IPS_BUSY;
        return ok;
    }
    if (JogSP[2].getState() == ISS_ON)
    {
        targetPosition_ = effectiveSafeMaxPosition_;
        const bool ok = sendControllerWriteSequence(geminiastro::setJogDirectionCommand(true), geminiastro::jogStartCommand());
        if (ok)
            FocusAbsPosNP.s = IPS_BUSY;
        return ok;
    }
    if (JogSP[3].getState() == ISS_ON)
    {
        const bool ok = sendControllerWriteSequence(geminiastro::jogStopCommand(), geminiastro::abortCommand());
        if (ok)
        {
            FocusAbsPosNP.s = IPS_IDLE;
            FocusRelPosNP.s = IPS_IDLE;
        }
        return ok;
    }

    return true;
}

bool GeminiAstroFocuser::handleTemperatureCompSwitch()
{
    if (TemperatureCompSP[0].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::setTemperatureCompEnabledCommand(true));
    if (TemperatureCompSP[1].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::setTemperatureCompEnabledCommand(false));
    if (TemperatureCompSP[2].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::setTemperatureCompDirectionCommand(true));
    if (TemperatureCompSP[3].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::setTemperatureCompDirectionCommand(false));

    return true;
}

bool GeminiAstroFocuser::handleBacklashSwitch()
{
    if (BacklashSP[0].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::setBacklashInEnabledCommand(true));
    if (BacklashSP[1].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::setBacklashInEnabledCommand(false));
    if (BacklashSP[2].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::setBacklashOutEnabledCommand(true));
    if (BacklashSP[3].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::setBacklashOutEnabledCommand(false));

    return true;
}

bool GeminiAstroFocuser::handleDisplaySwitch()
{
    if (DisplaySP[0].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::setLcdEnabledCommand(true));
    if (DisplaySP[1].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::setLcdEnabledCommand(false));
    if (DisplaySP[2].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::setLcdUpdateWhileMovingCommand(true));
    if (DisplaySP[3].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::setLcdUpdateWhileMovingCommand(false));
    if (DisplaySP[4].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::setDelayedDisplayUpdateCommand(true));
    if (DisplaySP[5].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::setDelayedDisplayUpdateCommand(false));

    return true;
}

bool GeminiAstroFocuser::handlePersistenceSwitch()
{
    bool moving = false;
    if (!readMoving(moving))
        return false;

    if (moving)
    {
        LOG_WARN("GeminiAstro persistence command rejected because the focuser is moving.");
        return false;
    }

    if (PersistenceSP[0].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::writeSettingsToEepromCommand());
    if (PersistenceSP[1].getState() == ISS_ON)
        return sendControllerWrite(geminiastro::writeDefaultSettingsCommand());
    if (PersistenceSP[2].getState() == ISS_ON)
    {
        if (!sendCommand(geminiastro::resetControllerCommand(), nullptr))
            return false;

        tcdrain(PortFD);
        usleep(2500000);
        pulseSerialControlLines();

        std::string response;
        if (!sendCommand(geminiastro::handshakeCommand(), &response) || !geminiastro::parseHandshake(response))
        {
            LOG_ERROR("GeminiAstro controller did not answer after reset.");
            return false;
        }

        return readStartupValues();
    }

    return true;
}

uint32_t GeminiAstroFocuser::clampToEffectiveLimits(uint32_t requested) const
{
    if (!hasUsableEffectiveLimits())
        return geminiastro::clampPosition(requested, controllerMaxPosition_);

    return std::clamp(requested, effectiveSafeMinPosition_, effectiveSafeMaxPosition_);
}

bool GeminiAstroFocuser::hasUsableEffectiveLimits() const
{
    return effectiveSafeMaxPosition_ > effectiveSafeMinPosition_ && effectiveSafeMaxPosition_ <= controllerMaxPosition_;
}
