//   This program is template code for programming small esp32 powered wifi controlled robots.
//   https://github.com/rcmgames/RCMv2
//   for information about the electronics, see the link at the top of this page: https://github.com/RCMgames
#include "rcm.h" //defines pins
#include <ESP32_easy_wifi_data.h> //https://github.com/joshua-8/ESP32_easy_wifi_data >=v1.0.0
#include <JMotor.h> //https://github.com/joshua-8/JMotor

const int dacUnitsPerVolt = 380; // increasing this number decreases the calculated voltage
const float wheelCir = PI * 0.065; // wheel diam
JVoltageCompMeasure<10> voltageComp = JVoltageCompMeasure<10>(batMonitorPin, dacUnitsPerVolt);
JMotorDriverEsp32L293 lMotorDriver = JMotorDriverEsp32L293(portC, true, false, false, 8000, 12);
JMotorDriverEsp32L293 rMotorDriver = JMotorDriverEsp32L293(portA, true, false, false, 8000, 12);
JEncoderSingleAttachInterrupt lEncoder = JEncoderSingleAttachInterrupt(inport2, wheelCir / 25 /*enc slits*/, false, 300000, 10000, RISING);
JEncoderSingleAttachInterrupt rEncoder = JEncoderSingleAttachInterrupt(inport1, wheelCir / 25 /*enc slits*/, false, 300000, 10000, RISING);
JMotorCompStandardConfig lMotorConfig = JMotorCompStandardConfig(3, .55, 5, 1.60, 6.75, 2.75, 80);
JMotorCompStandardConfig rMotorConfig = JMotorCompStandardConfig(3, .55, 5, 1.60, 6.75, 2.75, 80);
JMotorCompStandard lMotorCompensator = JMotorCompStandard(voltageComp, lMotorConfig, 1.0 / (wheelCir)); // factor converts from ground speed to rotations per second
JMotorCompStandard rMotorCompensator = JMotorCompStandard(voltageComp, rMotorConfig, 1.0 / (wheelCir));
JControlLoopBasic lCtrlLoop = JControlLoopBasic(20, 0, true);
JControlLoopBasic rCtrlLoop = JControlLoopBasic(20, 0, true);
JMotorControllerClosed lMotor = JMotorControllerClosed(lMotorDriver, lMotorCompensator, lEncoder, lCtrlLoop, INFINITY, INFINITY, 50000);
JMotorControllerClosed rMotor = JMotorControllerClosed(rMotorDriver, rMotorCompensator, rEncoder, rCtrlLoop, INFINITY, INFINITY, 50000);
JDrivetrainTwoSide drivetrain = JDrivetrainTwoSide(lMotor, rMotor, 0.16);
JDrivetrainControllerBasic driveController = JDrivetrainControllerBasic(drivetrain, { INFINITY, 0, INFINITY }, { 1, 0, 10 }, { 0.05, 0, 0.5 }, false);

JTwoDTransform driveInput = JTwoDTransform();
JTwoDTransform driveOutput = JTwoDTransform();

float A = 0;
float Aout = 0;
float Bout = 0;

void Enabled()
{
    // code to run while enabled, put your main code here
    driveOutput = JDeadzoneRemover::calculate(driveInput, { 0, 0, 0 }, driveController.getMaxVel() * .9, { 0, 0, 0 });
    driveController.moveVel(driveOutput);
}

void Enable()
{
    // turn on outputs
    driveController.resetDist();
    driveController.enable();
}

void Disable()
{
    // shut off all outputs
    driveController.disable();
}

jENCODER_MAKE_ISR_MACRO(lEncoder);
jENCODER_MAKE_ISR_MACRO(rEncoder);

void PowerOn()
{
    // runs once on robot startup, set pin modes and use begin() if applicable here
    lEncoder.setUpInterrupts(lEncoder_jENCODER_ISR);
    rEncoder.setUpInterrupts(rEncoder_jENCODER_ISR);
}

void Always()
{
    // always runs if void loop is running, JMotor run() functions should be put here
    // (but only the "top level", for example if you call drivetrainController.run() you shouldn't also call motorController.run())

    Aout = lEncoder.getPos();
    Bout = rEncoder.getPos();

    driveController.run();
    delay(1);
}

void configWifi()
{
    WiFi.setHostname("robot-j-2a"); // define hostname
    EWD::mode = EWD::Mode::connectToNetwork;
    EWD::routerName = "chicken";
    EWD::routerPassword = "bawkbawk";
    EWD::routerPort = 25210;

    // EWD::mode = EWD::Mode::createAP;
    // EWD::APName = "rcm0";
    // EWD::APPassword = "rcmPassword";
    // EWD::APPort = 25210;
}

void WifiDataToParse()
{
    enabled = EWD::recvBl();
    // add data to read here: (EWD::recvBl, EWD::recvBy, EWD::recvIn, EWD::recvFl)(boolean, byte, int, float)
    driveInput.theta = -EWD::recvFl();
    driveInput.x = EWD::recvFl();
}
void WifiDataToSend()
{
    EWD::sendFl(voltageComp.getSupplyVoltage());
    // add data to send here: (EWD::sendBl(), EWD::sendBy(), EWD::sendIn(), EWD::sendFl())(boolean, byte, int, float)
    EWD::sendFl(Aout);
    EWD::sendFl(Bout);
}

////////////////////////////// you don't need to edit below this line ////////////////////

void setup()
{
    Serial.begin(115200);
    pinMode(ONBOARD_LED, OUTPUT);
    PowerOn();
    Disable();
    configWifi();
    EWD::setupWifi(WifiDataToParse, WifiDataToSend);
}

void loop()
{
    EWD::runWifiCommunication();
    if (!EWD::wifiConnected || EWD::timedOut()) {
        enabled = false;
    }
    Always();
    if (enabled && !wasEnabled) {
        Enable();
    }
    if (!enabled && wasEnabled) {
        Disable();
    }
    if (enabled) {
        Enabled();
        digitalWrite(ONBOARD_LED, millis() % 500 < 250); // flash, enabled
    } else {
        if (!EWD::wifiConnected)
            digitalWrite(ONBOARD_LED, millis() % 1000 <= 100); // short flash, wifi connection fail
        else if (EWD::timedOut())
            digitalWrite(ONBOARD_LED, millis() % 1000 >= 100); // long flash, no driver station connected
        else
            digitalWrite(ONBOARD_LED, HIGH); // on, disabled
    }
    wasEnabled = enabled;
}
