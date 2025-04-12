package frc.robot.commands.FullAutoSystemCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Release;
import frc.robot.commands.FullTeleopSystemCommands.GrabAlgaeOne;

public class GrabAlgaeInAuto extends SequentialCommandGroup {
    public GrabAlgaeInAuto() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new GrabAlgaeOne().withTimeout(1)
        );
    }
}
