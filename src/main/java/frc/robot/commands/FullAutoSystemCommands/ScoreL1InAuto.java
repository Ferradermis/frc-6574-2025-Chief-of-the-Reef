package frc.robot.commands.FullAutoSystemCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelOne;

public class ScoreL1InAuto extends SequentialCommandGroup {
    public ScoreL1InAuto() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new ScoreLevelOne().withTimeout(1)

        );
    }
}
