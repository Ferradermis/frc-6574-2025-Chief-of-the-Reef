package frc.robot.commands.FullTeleopSystemCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Release;
import frc.robot.commands.SetPivotAngle;

public class LowerPivotAndScore extends SequentialCommandGroup {
    public LowerPivotAndScore() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new SetPivotAngle(Constants.PositionConstants.PIVOT_LOWER_ANGLE),
            new Release(13)
        );
    }
}
