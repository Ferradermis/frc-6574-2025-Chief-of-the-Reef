package frc.robot.commands.FullAutoSystemCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelFour;

public class ScoreL4InAutoNoAA extends SequentialCommandGroup {
    public ScoreL4InAutoNoAA() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new ScoreLevelFour().withTimeout(3)
        );
    }
}
