package frc.robot.commands.FullTeleopSystemCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Release;
import frc.robot.commands.SetPivotAngle;

public class ScoreCoral extends SequentialCommandGroup {
  public ScoreCoral() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetPivotAngle(Constants.PositionConstants.PIVOT_LOWER_ANGLE),
        new Release(10).withTimeout(0.25),
        new ReturnToHome()
    );
  }
    
}
