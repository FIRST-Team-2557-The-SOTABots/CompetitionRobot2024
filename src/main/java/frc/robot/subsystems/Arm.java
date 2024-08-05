package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.configs.ArmConfig;

public class Arm extends SubsystemBase {
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private AbsoluteEncoder mEncoder;
    private SparkPIDController mPID;
    private ArmPosition currentPosition;

    public enum ArmPosition {
        REST(0.02),
        VERTICAL(0.264),
        AMP(0.285);

        public double position;

        private ArmPosition(double position) {
            this.position = position;
        }
    }

    public Arm(ArmConfig config, SparkPIDController armPID, AbsoluteEncoder armLeftEncoder, CANSparkMax armLeftMotor,
            CANSparkMax armRightMotor) {

        this.leftMotor = armLeftMotor;
        this.rightMotor = armRightMotor;
        this.mEncoder = armLeftEncoder;
        this.mPID = armPID;

        leftMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        rightMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        rightMotor.setInverted(config.getRightMotorInverted());
        leftMotor.setInverted(config.getLeftMotorInverted());
        rightMotor.follow(leftMotor, true);

        mEncoder.setInverted(true);

        mPID.setP(config.getP());
        mPID.setI(config.getI());
        mPID.setD(config.getD());
        mPID.setFeedbackDevice(mEncoder);
        mPID.setOutputRange(config.getMinOutputRange(), config.getMaxOutputRange());
        this.currentPosition = ArmPosition.REST;

        Shuffleboard.getTab("Arm").addDouble("Left Encoder Positon", mEncoder::getPosition);
        Shuffleboard.getTab("Arm").addNumber("Setpoint", this::getCurrentSetpoint);
        Shuffleboard.getTab("Arm").addBoolean("At setpoint", this::isAtSetpoint);
    }

    public double getCorrectedLeftPosition() {
        if (mEncoder.getPosition() >= 0.9) {
            return 0.0;
        } else {
            return mEncoder.getPosition();
        }
    }

    public boolean isAtSetpoint() {
        if (getCorrectedLeftPosition() - currentPosition.position < .05) {
            return true;
        } else {
            return false;
        }
    }

    public void setDesiredPosition(ArmPosition position) {
        SmartDashboard.putBoolean("SETPOINT CHANGED", true);
        this.currentPosition = position;
    }

    private double getCurrentSetpoint() {
        return currentPosition.position;
    }

    @Override
    public void periodic() {
        mPID.setReference(currentPosition.position, ControlType.kPosition);
        SmartDashboard.putNumber("encoder position", getCorrectedLeftPosition());
    }
}
