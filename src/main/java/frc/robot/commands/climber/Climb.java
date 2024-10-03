package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Climber;

public class Climb extends ParallelCommandGroup {

    public Climb(Climber leftClimber, Climber rightClimber) {
        addCommands(
                Commands.run(() -> leftClimber.climb(), leftClimber).until(leftClimber::isFullyRetracted)
                        .andThen(Commands.runOnce(() -> leftClimber.stopMotor(), leftClimber)),

                Commands.run(() -> rightClimber.climb(), rightClimber).until(rightClimber::isFullyRetracted)
                        .andThen(Commands.runOnce(() -> rightClimber.stopMotor(), rightClimber)));

        addRequirements(leftClimber, rightClimber);
    }
}
