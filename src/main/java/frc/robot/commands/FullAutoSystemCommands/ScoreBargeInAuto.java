package frc.robot.commands.FullAutoSystemCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FullTeleopSystemCommands.ScoreAlgaeInBarge;

public class ScoreBargeInAuto extends SequentialCommandGroup {
    public ScoreBargeInAuto() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new ScoreAlgaeInBarge().withTimeout(3)
        );
    }
}
