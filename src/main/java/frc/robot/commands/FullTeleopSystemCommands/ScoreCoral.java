package frc.robot.commands.FullTeleopSystemCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Release;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetPivotAngle;
import frc.robot.commands.SetTurretAngle;
import frc.robot.util.ReefPositions.ReefLevel;

public class ScoreCoral extends SequentialCommandGroup {
  public ScoreCoral(ReefLevel level) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      switch (level) {
        case LEVEL_ONE:
        yield new SequentialCommandGroup(
          new SetPivotAngle(Constants.PositionConstants.PIVOT_LOWER_ANGLE).withTimeout(0.5),
          new Release(10).withTimeout(0.5),
          new PickupCoralFromChute()
        );
        case LEVEL_TWO:
        yield new SequentialCommandGroup(
          new SetPivotAngle(Constants.PositionConstants.PIVOT_LOWER_ANGLE).withTimeout(0.5),
          new Release(10).withTimeout(0.5),
          new PickupCoralFromChute()
        );
        case LEVEL_THREE:
        yield new SequentialCommandGroup(
          new SetPivotAngle(Constants.PositionConstants.PIVOT_LOWER_ANGLE).withTimeout(0.5),
          new Release(10).withTimeout(0.5),
          new PickupCoralFromChute()
        );
        case LEVEL_FOUR:
          yield new SequentialCommandGroup(
            new Release(10).withTimeout(0.5)
          );
      }
    );
  }
    
}
