package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer.LimeLightPipelines;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.swerve.RotateToAprilTag;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SOTA_SwerveDrive;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristPosition;

public class ShuttleSequence extends SequentialCommandGroup {
    private Shooter shooter;

    public ShuttleSequence(Shooter mShooter, Delivery mDelivery, Intake mIntake, Wrist mWrist,
            SOTA_SwerveDrive mSwerve , double setPoint, boolean onBlueSide) {
        this.shooter = mShooter;

        if (onBlueSide == false){
            setPoint = -setPoint;
        } 
        
        LimelightHelpers.takeSnapshot("", "Sequence " + LimelightHelpers.getTY(""));
        addCommands(
                Commands.runOnce(() -> {
                    mWrist.setDesiredPosition(WristPosition.REST);
                    LimelightHelpers.setPipelineIndex("", LimeLightPipelines.STAGE.id);                    
                }, mWrist),
                Commands.waitUntil(mWrist::atSetpoint),
                // Commands.run(() -> {mIntake.outtake(); mDelivery.toIntake();}, mIntake,
                // mWrist).until(mIntake::hasNote),
                new RotateToAprilTag(mSwerve, setPoint),
                Commands.parallel(
                        Commands.run(() -> {
                            mShooter.spinToRpm(2700);
                            mShooter.goToShuttleAngle();
                        }, mShooter).until(this::isReadyToShoot),
                        Commands.waitUntil(this::isReadyToShoot).andThen(Commands.run(() -> {
                            mIntake.intake();
                            mDelivery.toShooter();
                        }, mIntake, mDelivery))).withTimeout(1.5),
                Commands.waitSeconds(0.5).andThen(Commands.runOnce(() -> {
                    mShooter.stopFlyWheel();
                    mIntake.stop();
                    mDelivery.stop();
                }, mShooter)));

        addRequirements(mShooter, mDelivery, mIntake, mWrist, mSwerve);
    }

    public boolean isReadyToShoot() {
        return shooter.isAtShootingSpeed() && shooter.isAtAngle();
    }
}
