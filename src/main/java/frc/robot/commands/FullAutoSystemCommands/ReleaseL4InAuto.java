package frc.robot.commands.FullAutoSystemCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SetPivotAngle;
import frc.robot.commands.FullTeleopSystemCommands.PickupCoralFromChute;
import frc.robot.commands.FullTeleopSystemCommands.ScoreCoral;

public class ReleaseL4InAuto extends SequentialCommandGroup {
    public ReleaseL4InAuto() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new SetPivotAngle(Constants.PositionConstants.PIVOT_LOWER_ANGLE).withTimeout(0.5)
        );
        System.out.println("ReleaseInAuto");
    }
}