package frc.robot.subsystems.configs;

import SOTAlib.Config.EncoderConfig;
import SOTAlib.Config.MotorControllerConfig;

public class ShooterConfig {
    private double p;
    private double i;
    private double d;
    private double maxLinearValue;
    private double minLinearValue;

    private double angleConvA;
    private double angleConvB;
    private double angleConvC;

    private double shuttleAngleCalcA;
    private double shuttleAngleCalcB;
    private double shuttleAngleCalcC;

    private double angleCalcA;
    private double angleCalcB;
    private double angleCalcC;

    private double rpmCalcA;
    private double rpmCalcB;
    private double rpmCalcC;

    private double speakerTagHeight;
    private double speakerTagToHood;
    private double limeLightHeight;
    private double limeLightAngle;
    private double limeLightToShooterPivot;
    private double pivotHeight;

    private double leftKS;
    private double leftKV;
    private double rightKS;
    private double rightKV;

    private double targetRPM;

    private MotorControllerConfig linearActuatorConfig;
    private EncoderConfig linearEncoderConfig;

    private MotorControllerConfig leftShooterConfig;
    private MotorControllerConfig rightShooterConfig;
    private double restLinearValue;

    public double getShuttleAngleCalcA() {
        return shuttleAngleCalcA;
    }

    public double getShuttleAngleCalcB() {
        return shuttleAngleCalcB;
    }

    public double getShuttleAngleCalcC() {
        return shuttleAngleCalcC;
    }

    public double getRpmCalcA() {
        return rpmCalcA;
    }

    public double getRpmCalcB() {
        return rpmCalcB;
    }

    public double getRpmCalcC() {
        return rpmCalcC;
    }

    public double getAngleCalcA() {
        return angleCalcA;
    }

    public double getAngleCalcB() {
        return angleCalcB;
    }

    public double getAngleCalcC() {
        return angleCalcC;
    }

    public double getMinLinearValue() {
        return minLinearValue;
    }

    public double getLeftKS() {
        return leftKS;
    }

    public double getLeftKV() {
        return leftKV;
    }

    public double getRightKS() {
        return rightKS;
    }

    public double getRightKV() {
        return rightKV;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getSpeakerTagToHood() {
        return speakerTagToHood;
    }

    public double getLimeLightToShooterPivot() {
        return limeLightToShooterPivot;
    }

    public double getSpeakerTagHeight() {
        return speakerTagHeight;
    }

    public double getLimeLightHeight() {
        return limeLightHeight;
    }

    public double getLimeLightAngle() {
        return limeLightAngle;
    }

    public double getAngleConvA() {
        return angleConvA;
    }

    public double getAngleConvB() {
        return angleConvB;
    }

    public double getMaxLinearValue() {
        return maxLinearValue;
    }

    public MotorControllerConfig getLinearActuatorConfig() {
        return linearActuatorConfig;
    }

    public EncoderConfig getLinearEncoderConfig() {
        return linearEncoderConfig;
    }

    public MotorControllerConfig getLeftShooterConfig() {
        return leftShooterConfig;
    }

    public MotorControllerConfig getRightShooterConfig() {
        return rightShooterConfig;
    }

    public double getP() {
        return p;
    }

    public double getI() {
        return i;
    }

    public double getD() {
        return d;
    }

    public double getPivotHeight() {
        return pivotHeight;
    }

    public double getRestLinearValue() {
        return restLinearValue;
    }

    public double getAngleConvC() {
        return angleConvC;
    }
}
