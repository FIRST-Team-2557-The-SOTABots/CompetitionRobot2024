package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer.LimeLightPipelines;
import frc.robot.commands.swerve.RotateToAprilTag;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SOTA_SwerveDrive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Wrist.WristPosition;

public class AutoCommands {

    private Shooter mShooter;
    private Intake mIntake;
    private Wrist mWrist;
    private Delivery mDelivery;
    private Arm mArm;
    private SOTA_SwerveDrive mSwerve;

    public AutoCommands(Shooter shooter, Intake intake, Wrist wrist, Delivery delivery, Arm arm,
            SOTA_SwerveDrive swerve) {
        if (shooter == null) {
            throw new NullPointerException("Shooter Null in AutoCommands.");
        }

        if (delivery == null) {
            throw new NullPointerException("Delivery is null in AutoCommands");
        }

        if (intake == null) {
            throw new NullPointerException("intake is null in AutoCommands");
        }
        this.mShooter = shooter;
        this.mIntake = intake;
        this.mWrist = wrist;
        this.mDelivery = delivery;
        this.mArm = arm;
        this.mSwerve = swerve;

    }

    public Command intakeAutoStop () {
        return Commands.sequence(
            Commands.run(() -> {
                mIntake.intake();
                mWrist.setDesiredPosition(WristPosition.FLOOR);
            }, mIntake, mWrist).until(mIntake::hasNote).raceWith(Commands.waitSeconds(2.5)),
            Commands.runOnce(() -> {
                mIntake.stop();
                mWrist.setDesiredPosition(WristPosition.REST);
            }, mWrist)
        );
    }

    // public Command spinUpShoot(boolean isPreloaded) {
    //     return Commands.sequence(
    //             Commands.runOnce(() -> LimelightHelpers.setPipelineIndex("", LimeLightPipelines.SPEAKER.id), mShooter),
    //             new RotateToAprilTag(mSwerve),
    //             Commands.parallel(
    //                     // Commands.race(new RotateToAprilTag(mSwerve), Commands.waitSeconds(1.5)),
    //                     Commands.run(() -> {
    //                         mShooter.spinUpFlyWheel();
    //                         mShooter.goToAngle();
    //                     }, mShooter).until(this::isReadyToShoot),
    //                     Commands.waitUntil(this::isReadyToShoot).andThen(
    //                             Commands.runOnce(() -> {
    //                                 mIntake.intake();
    //                                 mDelivery.toShooter();
    //                             }))),
    //             Commands.waitSeconds(isPreloaded ? 0.75 : 1.5),
    //             Commands.runOnce(() -> {
    //                 // mShooter.stopFlyWheel();
    //                 mDelivery.stop();
    //                 mIntake.stop();
    //                 Commands.runOnce(() -> LimelightHelpers.setPipelineIndex("", LimeLightPipelines.MEGATAG.id));
    //             }, mShooter, mDelivery, mIntake));
    // }

    public Command spinUpShoot(boolean isPreloaded, boolean shouldRotate) {
        
        return Commands.sequence(
            // Commands.run(() -> new RotateToAprilTag(mSwerve)).raceWith(Commands.waitSeconds(1)),
            // new RotateToAprilTag(mSwerve),
            Commands.race(
                Commands.parallel(
                Commands.run(() -> {
                    LimelightHelpers.takeSnapshot("", "Auto Command " + LimelightHelpers.getTY(""));
                    mShooter.spinUpFlyWheel();
                    mShooter.goToAngle();
                }, mShooter),
                    shouldRotate ? new RotateToAprilTag(mSwerve) : Commands.waitSeconds(0) 
                ),
                Commands.waitUntil(this::isReadyToShoot).andThen(
                    Commands.waitSeconds(0.25).andThen(
                    Commands.run(() -> {mIntake.intake(); mDelivery.toShooter();}, mIntake, mDelivery).withTimeout(isPreloaded ? 0.75 : 1.2)
                ))
            ),
            Commands.runOnce(() -> {
                // mShooter.stopFlyWheel();
                mDelivery.stop();
                mIntake.stop();
            })
        );
    }


    public Command alignShooter () {
        return Commands.run(() -> mShooter.goToAngle());
    }

    public Command setFlyWheels() {
        return Commands.runOnce(() -> {
            mShooter.setSpeed(0.9);

        });
    }

    public Command runFlywheelsToRpm() {
        return Commands.run(() -> mShooter.spinUpFlyWheel(), mShooter).until(mShooter::isAtShootingSpeed);
    }

    public Command stopFlyWheels() {
        return Commands.runOnce(() -> {
            mShooter.stopFlyWheel();
        });
    }

    public Command spinFlywheels() {
        return Commands.run(() -> mShooter.spinUpFlyWheel());
    }

    public Command shootNote() {
        return Commands.sequence(
            // new RotateToAprilTag(mSwerve),
            Commands.runOnce(() -> {
                LimelightHelpers.setPipelineIndex("", LimeLightPipelines.SPEAKER.id);
                mIntake.intake();
                mDelivery.toShooter();
            }, mIntake, mDelivery),
            Commands.waitSeconds(0.75),
            Commands.runOnce(() -> {
                mIntake.stop();
                mDelivery.stop();
                LimelightHelpers.setPipelineIndex("", LimeLightPipelines.MEGATAG.id);
            }, mIntake, mDelivery)
        );
    }

    public Command checkIntake() {
        return Commands.runOnce(() -> {
            if (!mIntake.hasNote()) {
                mWrist.setDesiredPosition(WristPosition.REST);
                mIntake.stop();
            }
        }, mIntake, mWrist);
    }

    public Command alignAndShoot() {
        return Commands.sequence(
            Commands.runOnce(() -> LimelightHelpers.setPipelineIndex("", LimeLightPipelines.SPEAKER.id)),
            new RotateToAprilTag(mSwerve),
            Commands.parallel(
                Commands.run(() -> {
                    mShooter.goToAngle();
                }, mShooter).until(this::isReadyToShoot),
                Commands.waitUntil(this::isReadyToShoot).andThen(
                    Commands.runOnce(() -> {
                        mIntake.intake();
                        mDelivery.toShooter();
            }))),
            Commands.waitSeconds(1),
            Commands.runOnce(() -> {
                mIntake.stop();
                mDelivery.stop();
                LimelightHelpers.setPipelineIndex("", LimeLightPipelines.MEGATAG.id);
            }));
    }

    public Command setArmToAmp() {
        return Commands.runOnce(() -> {
            mArm.setDesiredPosition(ArmPosition.AMP);
            mWrist.setDesiredPosition(WristPosition.AMP);
        }).andThen(Commands.waitUntil(this::armIsAtSetpoint));
    }

    public Command scoreInAmp () {
        return Commands.sequence(
            Commands.runOnce(() -> {
                mArm.setDesiredPosition(ArmPosition.AMP);
                mWrist.setDesiredPosition(WristPosition.AMP);
            }),
            Commands.waitUntil(this::armIsAtSetpoint),
            Commands.run(() -> mIntake.intake()).withTimeout(1),
            // Commands.waitSeconds(0.5),
            Commands.runOnce(() -> mIntake.stop()),

            Commands.runOnce(() -> {
                mArm.setDesiredPosition(ArmPosition.REST);
                mWrist.setDesiredPosition(WristPosition.REST);
            })

        );
    }

    public Command setArmToRest() {
        return Commands.runOnce(() -> {
            mArm.setDesiredPosition(ArmPosition.REST);
            mWrist.setDesiredPosition(WristPosition.REST);
        }).andThen(Commands.waitUntil(mArm::isAtSetpoint));
    }

    public Command intakeAmp () {
        return Commands.run(() -> {
            mIntake.intake();
        }).withTimeout(0.3).andThen(Commands.runOnce(() -> {
            mIntake.stop();
        }));
    }

    public boolean isReadyToShoot() {
        return mShooter.isAtShootingSpeed() && mShooter.isAtAngle() && mWrist.atSetpoint();
    }

    public boolean armIsAtSetpoint() {
        return mArm.isAtSetpoint() && mWrist.atSetpoint();
    }

    public Command spitNote() {
        return Commands.run(
            () -> mShooter.setSpeed(1), mShooter).withTimeout(0.25).andThen(Commands.runOnce(
            () -> mShooter.stopFlyWheel(), mShooter));
    }

    public Command feedNote() {
        return Commands.run(() -> {
            mIntake.intake(); 
            mDelivery.toShooter();
        }, mIntake, mDelivery).until(mDelivery::shooterHasNote).withTimeout(2).andThen(Commands.runOnce(() -> {
            mIntake.stop(); 
            mDelivery.stop();
        }, mIntake, mDelivery));
    }
}
