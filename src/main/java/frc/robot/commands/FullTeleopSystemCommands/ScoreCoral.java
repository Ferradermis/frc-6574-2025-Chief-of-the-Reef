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
  public ScoreCoral() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetPivotAngle(Constants.PositionConstants.PIVOT_LOWER_ANGLE).withTimeout(0.5),
        new Release(10).withTimeout(0.5),
        new PickupCoralFromChute()
      );
    // Unused code for scoring at different levels - remake this after MROC
    // System.out.println("Scoring at level: " + level);
    // if (level == ReefLevel.LEVEL_ONE) {
    //   addCommands(
    //     new Release(10).withTimeout(0.5));
    // }
    // if (level == ReefLevel.LEVEL_TWO) {
    //   addCommands(
    //     new SetPivotAngle(Constants.PositionConstants.PIVOT_LOWER_ANGLE).withTimeout(0.5),
    //     new Release(10).withTimeout(0.5),
    //     new PickupCoralFromChute()
    //   );
    // }
    // if (level == ReefLevel.LEVEL_THREE) {
    //   addCommands(
    //     new SetPivotAngle(Constants.PositionConstants.PIVOT_LOWER_ANGLE).withTimeout(0.5),
    //     new Release(10).withTimeout(0.5),
    //     new PickupCoralFromChute()
    //   );
    // }
    // if (level == ReefLevel.LEVEL_FOUR) {
    //     addCommands(
    //       new SetPivotAngle(Constants.PositionConstants.PIVOT_LOWER_ANGLE).withTimeout(0.5),
    //       new Release(10).withTimeout(0.5),
    //       new PickupCoralFromChute()
    //     );
    // }
  }
}
