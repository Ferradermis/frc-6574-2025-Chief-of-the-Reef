package frc.robot.commands.FullAutoSystemCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake;

public class IntakeInAuto extends SequentialCommandGroup {
    public IntakeInAuto() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new Intake(13).withTimeout(3)
        );
        System.out.println("IntakeInAuto");
    }
    
}
