package frc.robot.commands.FullTeleopSystemCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Release;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetPivotAngle;

public class ScoreCoralL3 extends SequentialCommandGroup {
  public ScoreCoralL3() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetPivotAngle(Constants.PositionConstants.LEVEL_THREE_LOWER_ANGLE).withTimeout(0.5),
        new ParallelCommandGroup(
          new Release(10).withTimeout(1.5),
          new SetElevatorPosition(10.8 * 39.37).withTimeout(1)
        ).withTimeout(1),
        new PickupCoralFromChute()
    );
  }
    
}
