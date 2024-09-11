package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SOTA_SwerveDrive;

public class RotateToAprilTag extends Command {
    private SOTA_SwerveDrive mDrive;
    private PIDController mRotationController;
    private double xOffset;
    private double setPoint;

    public  RotateToAprilTag(SOTA_SwerveDrive drive, double setPoint) {
        this.setPoint = setPoint;
        this.mDrive = drive;
        mRotationController = new PIDController(0.1, 0, 0);
        mRotationController.setTolerance(1.6);

        addRequirements(drive);
    }

    public double calculateRotationSpeed() {
        xOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
        SmartDashboard.putNumber("xOffset", xOffset);
        SmartDashboard.putBoolean("At setpoint", mRotationController.atSetpoint());
        if (mRotationController.atSetpoint()) {
            return 0;
        } else {
            return mRotationController.calculate(xOffset, setPoint);
        }
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, calculateRotationSpeed());
        mDrive.drive(speeds);
    }

    @Override
    public boolean isFinished() {
        return mRotationController.atSetpoint() && xOffset != 0.0;

    }

    @Override
    public void initialize() {
        mRotationController.reset();
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.drive(new ChassisSpeeds(0, 0, 0));
    }
}
