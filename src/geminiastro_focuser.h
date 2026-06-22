#pragma once

#include "geminiastro_protocol.h"

#include <indifocuser.h>
#include <indipropertyswitch.h>
#include <indipropertytext.h>

#include <cstdint>
#include <cstdio>
#include <string>

class GeminiAstroFocuser : public INDI::Focuser
{
  public:
    GeminiAstroFocuser();
    ~GeminiAstroFocuser() override = default;

    const char *getDefaultName() override;
    bool initProperties() override;
    bool updateProperties() override;

  protected:
    bool Handshake() override;
    IPState MoveAbsFocuser(uint32_t targetTicks) override;
    IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks) override;
    bool AbortFocuser() override;
    bool SyncFocuser(uint32_t ticks) override;
    bool SetFocuserMaxPosition(uint32_t ticks) override;
    void TimerHit() override;
    bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
    bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
    bool saveConfigItems(FILE *fp) override;

  private:
    bool sendCommand(const std::string &command, std::string *response);
    void pulseSerialControlLines();
    void handleCommunicationFailure(const char *context);
    bool readStartupValues();
    bool readFirmware(std::string &firmware);
    bool readControllerModel(std::string &model);
    bool readMaxPosition(uint32_t &maxPosition);
    bool readMaxIncrement(uint32_t &increment);
    bool readPosition(uint32_t &position);
    bool readMoving(bool &moving);
    bool readTemperature(double &temperatureC);
    bool readCoilPower(bool &enabled);
    bool readReverse(bool &enabled);
    bool readTemperaturePrecision(uint32_t &precision);
    bool readTemperatureCompEnabled(bool &enabled);
    bool readTemperatureCompAvailable(bool &available);
    bool readTemperatureCompValue(uint32_t &steps);
    bool readTemperatureProbeAvailable(bool &available);
    bool readTemperatureCompOption(bool &enabled);
    bool readTemperatureCompDirection(bool &outward);
    bool readHomeSwitch(bool &closed);
    bool readJogEnabled(bool &enabled);
    bool readJogDirection(bool &outward);
    bool readStepperPower(bool &powered);
    bool readStepMode(uint32_t &mode);
    bool readStepSizeEnabled(bool &enabled);
    bool readStepSize(double &size);
    bool readMotorSpeed(uint32_t &speed);
    bool readDelayAfterMove(uint32_t &delay);
    bool readBacklashInEnabled(bool &enabled);
    bool readBacklashOutEnabled(bool &enabled);
    bool readBacklashInSteps(uint32_t &steps);
    bool readBacklashOutSteps(uint32_t &steps);
    bool readDisplayStatus(bool &enabled);
    bool readLcdPageDisplayTime(uint32_t &milliseconds);
    bool readLcdUpdateWhileMoving(bool &enabled);
    bool moveAbsolute(uint32_t position);
    bool sendControllerWrite(const std::string &command);
    bool sendControllerWriteSequence(const std::string &first, const std::string &second);
    void readControllerDiagnostics();
    void applyControllerMaxPosition(uint32_t maxPosition);
    void initializeSafeLimits(uint32_t controllerMaxPosition, uint32_t currentPosition);
    void resetSafeLimitsToTemporaryWindow(uint32_t currentPosition);
    void clearConfiguredSafeLimits();
    bool updateConfiguredSafeLimits();
    bool applyConfiguredSafeLimits(uint32_t configuredMin, uint32_t configuredMax, uint32_t guardSteps);
    void applyEffectiveLimitsToProperties();
    void updateLimitStatus();
    void updateCalibrationStatus(uint32_t currentPosition, bool havePosition);
    void updateCalibrationStatus();
    bool nudgeCalibration(FocusDirection dir);
    bool captureCalibrationPoint(bool minimum);
    bool applyCalibrationLimits();
    bool centerCalibrationPosition();
    void resetCalibration();
    bool updateMotorSettings(char *names[], int n);
    bool updateTemperatureCompSettings(char *names[], int n);
    bool updateBacklashSettings(char *names[], int n);
    bool updateDisplaySettings(char *names[], int n);
    bool handleMotorSwitch();
    bool handleJogSwitch();
    bool handleTemperatureCompSwitch();
    bool handleBacklashSwitch();
    bool handleDisplaySwitch();
    bool handlePersistenceSwitch();
    uint32_t calibrationStep() const;
    uint32_t maxCalibrationStep() const;
    bool readIdlePosition(uint32_t &position);
    uint32_t clampToEffectiveLimits(uint32_t requested) const;
    bool hasUsableEffectiveLimits() const;

    uint32_t targetPosition_{0};
    uint32_t lastPosition_{0};
    uint32_t controllerMaxPosition_{10000};
    uint32_t controllerMaxIncrement_{10000};
    uint32_t effectiveSafeMinPosition_{0};
    uint32_t effectiveSafeMaxPosition_{10000};
    bool lastMoving_{false};
    bool safeLimitsConfigured_{false};
    bool calibrationMinCaptured_{false};
    bool calibrationMaxCaptured_{false};
    bool haveLastCalibrationTarget_{false};
    uint32_t calibrationMinPosition_{0};
    uint32_t calibrationMaxPosition_{0};
    uint32_t lastCalibrationTarget_{0};
    bool haveHomeSwitchState_{false};
    bool lastHomeSwitchClosed_{false};
    bool haveStepperPowerState_{false};
    bool lastStepperPowered_{false};

    INDI::PropertyText ControllerInfoTP{2};
    INDI::PropertyNumber TemperatureNP{1};
    INDI::PropertyNumber SafeLimitsNP{3};
    INDI::PropertyNumber LimitStatusNP{5};
    INDI::PropertyNumber CalibrationStepNP{1};
    INDI::PropertyNumber CalibrationStatusNP{5};
    INDI::PropertySwitch CalibrationSP{7};
    INDI::PropertyNumber ControllerStatusNP{24};
    INDI::PropertyNumber MotorSettingsNP{5};
    INDI::PropertySwitch MotorSwitchSP{9};
    INDI::PropertySwitch JogSP{4};
    INDI::PropertyNumber TemperatureCompNP{2};
    INDI::PropertySwitch TemperatureCompSP{4};
    INDI::PropertyNumber BacklashNP{2};
    INDI::PropertySwitch BacklashSP{4};
    INDI::PropertyNumber DisplayNP{2};
    INDI::PropertySwitch DisplaySP{6};
    INDI::PropertySwitch PersistenceSP{3};
};
