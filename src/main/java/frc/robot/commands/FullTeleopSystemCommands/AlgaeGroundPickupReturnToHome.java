package frc.robot.commands.FullTeleopSystemCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake;
import frc.robot.commands.Release;

public class AlgaeGroundPickupReturnToHome extends SequentialCommandGroup {
    public AlgaeGroundPickupReturnToHome() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new ParallelCommandGroup(
                new Release(10).withTimeout(1.5),
                new AlgaeReturnToHome().withTimeout(1)
            ),
            new Intake(0).withTimeout(0.1)
        );
    }
}
