package frc.robot.commands.FullAutoSystemCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Release;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetPivotAngle;
import frc.robot.commands.SetTurretAngle;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelOne;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelThree;

public class ScoreL3InAuto extends SequentialCommandGroup {
    public ScoreL3InAuto() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new ScoreLevelThree().withTimeout(1),
            new SetPivotAngle(0.100).withTimeout(1)

        );
    }
}
