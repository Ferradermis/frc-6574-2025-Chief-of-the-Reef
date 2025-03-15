package frc.robot.commands.FullAutoSystemCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FullTeleopSystemCommands.Tomfoolery;

public class TomfooleryInAuto extends SequentialCommandGroup {
    public TomfooleryInAuto() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new Tomfoolery().withTimeout(3)
        );
    }
}
