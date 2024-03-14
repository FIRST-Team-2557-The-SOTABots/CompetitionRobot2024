package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.configs.WristConfig;

public class Wrist extends SubsystemBase {

    public enum WristPosition {
        FLOOR(0.38),
        REST(0.048),
        AMP(0.34);

        public double position;

        private WristPosition(double position) {
            this.position = position;
        }
    }

    private AbsoluteEncoder mEncoder;
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private SparkPIDController mPID;
    private WristPosition currentPosition;

    public Wrist(WristConfig config, SparkPIDController wristPID, AbsoluteEncoder wristEncoder, CANSparkMax wristLeftMotor,
            CANSparkMax wristRightMotor) {
        this.mEncoder = wristEncoder;
        this.leftMotor = wristLeftMotor;
        this.rightMotor = wristRightMotor;
        this.mPID = wristPID;

        leftMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        rightMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        rightMotor.setInverted(config.getRightMotorInverted());
        leftMotor.setInverted(config.getLeftMotorInverted());
        rightMotor.follow(leftMotor, true);

        mPID.setP(config.getP());
        mPID.setI(config.getI());
        mPID.setD(config.getD());
        mPID.setFeedbackDevice(wristEncoder);
        mPID.setPositionPIDWrappingEnabled(true);
        mPID.setOutputRange(config.getMinOutputRange(), config.getMaxOutputRange());
        currentPosition = WristPosition.REST;
        
        

        Shuffleboard.getTab("Wrist").addDouble("Encoder Pos", mEncoder::getPosition);


        // this.currentPosition = WristPosition.REST;
    }

    public void setDesiredPosition(WristPosition position) {
        this.currentPosition = position;
    }

    public double getAdjustedEncoder(){
        if (mEncoder.getPosition() > 0.9){
            return 0.0;
        } else{
            return mEncoder.getPosition();
        }
    }

    public boolean atSetpoint(){
        if (getAdjustedEncoder() - currentPosition.position < .05){
            return true;
        } else{
            return false;
        }
    }


    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }


    @Override
    public void periodic() {
         mPID.setReference(currentPosition.position, ControlType.kPosition);
         SmartDashboard.putNumber("encoder position", getAdjustedEncoder());
        //Shuffleboard.getTab("Wrist").addBoolean("At setpoint", Wrist.atSetpoint);
        //Shuffleboard.getTab("Wrist").addDouble("Current Encoder Postion", mEncoder::getPosition);
    }
}
