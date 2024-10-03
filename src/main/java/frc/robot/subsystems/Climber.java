package frc.robot.subsystems;

import SOTAlib.MotorController.SOTA_MotorController;
import frc.robot.subsystems.configs.ClimberConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private SOTA_MotorController mMotor;
    private DigitalInput mLimitSwitch;
    private double kClimbSpeed;
    private String name;

    public Climber(ClimberConfig config, SOTA_MotorController motor, DigitalInput limitSwitch) {
        this.mMotor = motor;
        this.mLimitSwitch = limitSwitch;
        this.kClimbSpeed = config.getClimbSpeed();
        this.name = config.getName();
        Shuffleboard.getTab("Climber").addBoolean(name + " isFullyRetracted:", this::isFullyRetracted);
        Shuffleboard.getTab("Climber").addNumber(name + " encoder position", mMotor::getEncoderPosition);
        Shuffleboard.getTab("Climber").addDouble(name + "Current", mMotor::getMotorCurrent);
    }

    public void climb() {
        mMotor.set(kClimbSpeed);
    }

    public void downClimb() {
        if (mMotor.getEncoderPosition() >= -140) {
            mMotor.set(-kClimbSpeed);
        } else {
            mMotor.stopMotor();
        }
    }

    public boolean isFullyRetracted() {
        return !mLimitSwitch.get();
    }

    public double getCurrentExtension() {
        return mMotor.getEncoderPosition();
    }

    @Override
    public void periodic() {
        if (isFullyRetracted()) {
            mMotor.resetEncoder();
        }
    }

    public void stopMotor() {
        mMotor.stopMotor();
    }

}
