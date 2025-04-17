package frc.robot.commands.FullAutoSystemCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake;

public class ReleaseAlgaeInAuto extends SequentialCommandGroup {
    public ReleaseAlgaeInAuto() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new Intake(16).withTimeout(1.5)
        );
    }
}