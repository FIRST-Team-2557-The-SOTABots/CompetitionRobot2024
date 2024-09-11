package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import SOTAlib.Encoder.Absolute.SOTA_AbsoulteEncoder;
import SOTAlib.MotorController.SOTA_MotorController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.configs.ShooterConfig;

public class Shooter extends SubsystemBase {
    // private CANSparkMax linearActuator;
    // private AbsoluteEncoder linearEncoder;
    // private SparkPIDController linearPID;

    SOTA_MotorController linearActuator;
    SOTA_AbsoulteEncoder linearEncoder;
    PIDController linearPID;
    private ArmFeedforward linearFF;
    private double maxLinearValue;
    private double minLinearValue;
    private double restLinearValue;
    private double angleConvA;
    private double angleConvB;
    private double angleConvC;
    private double angleCalcA;
    private double angleCalcB;
    private double angleCalcC;
    private double ShuttleAngleCalcA;
    private double ShuttleAngleCalcB;
    private double ShuttleAngleCalcC;

    private double rpmCalcA;
    private double rpmCalcB;
    private double rpmCalcC;

    private double speakerTagHeight;
    private double speakerTagToHood;
    private double limeLightHeight;
    private double limeLightAngle;
    private double limeLightToShooterPivot;
    private double pivotHeight;

    private SOTA_MotorController leftShooter;
    private SOTA_MotorController rightShooter;
    private double targetRPM;
    private double kMaxShooterAngle;

    private double currentAngleSetpoint;
    private double leftKv;
    private double leftKs;
    private double rightKv;
    private double rightKs;
    private double targetAngle;

    private boolean blueSide = true;

    public Shooter(ShooterConfig config, SOTA_MotorController linearActuator, SOTA_AbsoulteEncoder linearEncoder,
            SOTA_MotorController leftShooter, SOTA_MotorController rightShooter) {

        this.linearActuator = linearActuator;
        this.linearEncoder = linearEncoder;
        this.currentAngleSetpoint = 0.0;
        this.targetAngle = 0.0;

        // this.linearPID = linearActuator.getPIDController();
        this.linearPID = new PIDController(config.getP(), config.getI(), config.getD());
        this.linearPID.setTolerance(0.15);
        this.maxLinearValue = config.getMaxLinearValue();
        this.minLinearValue = config.getMinLinearValue();
        this.restLinearValue = config.getRestLinearValue();
        this.angleConvA = config.getAngleConvA();
        this.angleConvB = config.getAngleConvB();
        this.angleConvC = config.getAngleConvC();

        this.speakerTagHeight = config.getSpeakerTagHeight();
        this.speakerTagToHood = config.getSpeakerTagToHood();
        this.limeLightHeight = config.getLimeLightHeight();
        this.limeLightAngle = config.getLimeLightAngle();
        this.limeLightToShooterPivot = config.getLimeLightToShooterPivot();
        this.pivotHeight = config.getPivotHeight();

        this.leftShooter = leftShooter;
        this.rightShooter = rightShooter;
        this.leftKs = config.getLeftKS();
        this.leftKv = config.getLeftKV();
        this.rightKs = config.getRightKS();
        this.rightKv = config.getRightKV();
        this.targetRPM = config.getTargetRPM();
        this.kMaxShooterAngle = encoderToAngle(maxLinearValue);

        this.ShuttleAngleCalcA = config.getShuttleAngleCalcA();
        this.ShuttleAngleCalcB = config.getShuttleAngleCalcB();
        this.ShuttleAngleCalcC = config.getShuttleAngleCalcC();

        this.angleCalcA = config.getAngleCalcA();
        this.angleCalcB = config.getAngleCalcB();
        this.angleCalcC = config.getAngleCalcC();

        this.rpmCalcA = config.getRpmCalcA();
        this.rpmCalcB = config.getRpmCalcB();
        this.rpmCalcC = config.getRpmCalcC();

        Shuffleboard.getTab("Shooter").addDouble("Encoder Position", this.linearEncoder::getPosition);
        Shuffleboard.getTab("Shooter").addDouble("Angle Target", this::calcTargetAngle);
        Shuffleboard.getTab("Shooter").addDouble("RPM Target", this::calcTargetRpm);
        Shuffleboard.getTab("Shooter").addDouble("Shooter Angle", this::getShooterAngle);
        Shuffleboard.getTab("Shooter").addDouble("Left rpm", leftShooter::getEncoderVelocity);
        Shuffleboard.getTab("Shooter").addDouble("Right rpm", rightShooter::getEncoderVelocity);
        Shuffleboard.getTab("Shooter").addDouble("Right Current", rightShooter::getMotorCurrent);
        Shuffleboard.getTab("Shooter").addDouble("Left Current", leftShooter::getMotorCurrent);
        // Shuffleboard.getTab("Competition").addDouble("Left rpm",
        // leftShooter::getEncoderVelocity);
        // Shuffleboard.getTab("Competition").addDouble("Right rpm",
        // rightShooter::getEncoderVelocity);
        Shuffleboard.getTab("Competition").addBoolean("Close Enough!", this::isTooFar);
        Shuffleboard.getTab("Competition").addBoolean("Ready To Shoot", this::isReadyToShoot);
        Shuffleboard.getTab("Competition").addDouble("Angle Target", this::calcTargetAngle);
        Shuffleboard.getTab("Competition").addBoolean("On Blue Side", this::onBlueSide);
        Shuffleboard.getTab("Shooter").addDouble("Corrected Position", this::getCorrectedEncoderPosition);
        Shuffleboard.getTab("Shooter").addBoolean("isAtShootingSpeed", this::isAtShootingSpeed);
        Shuffleboard.getTab("Shooter").addDouble("Distance to Limelight", this::calcDistanceLimeLightToTag);
        Shuffleboard.getTab("Shooter").addDouble("Linear Actuator Voltage", linearActuator::getMotorCurrent);
        Shuffleboard.getTab("Shooter").addBoolean("At Angle", this::isAtAngle);
        Shuffleboard.getTab("Shooter").addBoolean("Intakeable", this::isIntakeAble);

    }

    public void changeSides() {
        if (blueSide == false){
            blueSide = true;
        }
        else if (blueSide == true){
            blueSide = false;
        }
    }

    public boolean onBlueSide() {
        return blueSide;
    }

    public double getCorrectedEncoderPosition() {
        double output;
        if (linearEncoder.getPosition() > 0.95) {
            output = 0.0;
        } else {
            output = linearEncoder.getPosition();
        }

        return output;
    }

    public double calcShuttleAngle() {
        double x = calcDistanceLimeLightToTag();
        return (ShuttleAngleCalcA * x * x) + (ShuttleAngleCalcB * x) + ShuttleAngleCalcC;
    }

    public double calcTargetAngle() {
        double x = calcDistanceLimeLightToTag();
        return (angleCalcA * x * x) + (angleCalcB * x) + angleCalcC;
    }

    public double calcTargetRpm() {
        double x = calcDistanceLimeLightToTag();
        return (rpmCalcA * x * x) + (rpmCalcB * x) + rpmCalcC;
    }

    public void linearActuatorSetVoltage(double volts) {
        if (getCorrectedEncoderPosition() > maxLinearValue && volts > 0) {
            linearActuator.stopMotor();
        } else if (getCorrectedEncoderPosition() < minLinearValue && volts < 0) {
            linearActuator.stopMotor();
        } else if (linearPID.atSetpoint()) {
            linearActuator.stopMotor();
        } else {
            linearActuator.setVoltage(volts);
        }
        // linearActuator.setVoltage(volts);
    }

    public void spinUpFlyWheel() {
        targetRPM = Math.min(calcTargetRpm(), 5000);
        leftShooter.setVoltage(leftRPMToVolts(targetRPM));
        rightShooter.setVoltage(rightRPMToVolts(targetRPM));
    }

    public void spinToRpm(int rpm) {
        leftShooter.setVoltage(leftRPMToVolts(rpm));
        rightShooter.setVoltage(rightRPMToVolts(rpm));
    }

    public void stopFlyWheel() {
        leftShooter.stopMotor();
        rightShooter.stopMotor();
    }

    public boolean isAtShootingSpeed() {
        return leftShooter.getEncoderVelocity() >= targetRPM - 100
                || rightShooter.getEncoderVelocity() >= targetRPM - 100 || leftShooter.getEncoderVelocity() >= 4800;
    }

    public boolean isNotAtShootingSpeed() {
        return !isAtShootingSpeed();
    }

    public double encoderToAngle(double encoderPos) {
        return (angleConvA * encoderPos * encoderPos) + (angleConvB * encoderPos) + angleConvC;
    }

    public double getShooterAngle() {
        return encoderToAngle(getCorrectedEncoderPosition());
    }

    public double calcDistanceLimeLightToTag() {
        return (speakerTagHeight - limeLightHeight) / Math.tan(Math.toRadians(limeLightAngle
                + NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0)));
    }

    public double calcAngleToHood() {
        return Math.toDegrees(Math.atan((speakerTagToHood + speakerTagHeight - pivotHeight)
                / (calcDistanceLimeLightToTag() + limeLightToShooterPivot)));
    }

    public void goToAngle() {
        currentAngleSetpoint = calcTargetAngle();
        double volts = linearPID.calculate(encoderToAngle(getCorrectedEncoderPosition()),
                calcTargetAngle());
        linearActuatorSetVoltage(volts);
    }

    public void goToShuttleAngle() {
        currentAngleSetpoint = calcShuttleAngle();
        double volts = linearPID.calculate(encoderToAngle(getCorrectedEncoderPosition()),
                calcShuttleAngle());
        linearActuatorSetVoltage(volts);
    }

    public void goToSpecifiedAngle(double angle) {
        currentAngleSetpoint = angle;
        double volts = linearPID.calculate(encoderToAngle(getCorrectedEncoderPosition()), angle);
        linearActuatorSetVoltage(volts);
    }

    public void goToRestAngle() {
        currentAngleSetpoint = encoderToAngle(restLinearValue);
        // currentAngleSetpoint = 53;
        double volts = linearPID.calculate(encoderToAngle(getCorrectedEncoderPosition()),
                encoderToAngle(restLinearValue));
        linearActuatorSetVoltage(volts);
    }

    public boolean isAtAngle() {
        return getShooterAngle() <= currentAngleSetpoint + 0.15 && getShooterAngle() >= currentAngleSetpoint - 0.15;
    }

    public boolean isIntakeAble() {
        return getShooterAngle() < 26.5;
    }

    public boolean isReadyToShoot() {
        return isAtAngle() && isAtShootingSpeed();
    }

    private boolean isTooClose() {
        return calcAngleToHood() > kMaxShooterAngle;
    }

    private boolean isTooFar() {
        return calcTargetAngle() > 26.5; // TODO: find actual angle using encoder units
    }

    public void setSpeed(double speed) {
        leftShooter.set(speed);
        rightShooter.set(speed);
    }

    public double leftRPMToVolts(double RPM) {
        return leftKv * RPM + leftKs * Math.signum(RPM);
    }

    public double rightRPMToVolts(double RPM) {
        return rightKv * RPM + rightKs * Math.signum(RPM);
    }

    @Override
    public void periodic() {
        // linearPID.setReference(Math.toRadians(currentAngleSetpoint),
        // ControlType.kPosition);
        targetAngle = calcTargetAngle();
        if (linearPID.atSetpoint()) {
            linearPID.reset();
        }
    }
}
