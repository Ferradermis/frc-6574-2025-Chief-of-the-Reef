package frc.robot.commands.FullTeleopSystemCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Intake;
import frc.robot.commands.SetPivotAngle;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetTurretAngle;
import frc.robot.util.ReefPositions.ReefLevel;

public class ScoreLevel extends SequentialCommandGroup {
  public ScoreLevel(ReefLevel level) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
      switch (level) {
        case LEVEL_ONE:
        addCommands(
        new ParallelCommandGroup(
            new SetElevatorPosition(Constants.PositionConstants.LEVEL_ONE_ELEVATOR_HEIGHT).withTimeout(0.3),
            new SetPivotAngle(Constants.PositionConstants.LEVEL_ONE_PIVOT_ANGLE).withTimeout(0.3),
            new SetTurretAngle(Constants.PositionConstants.HORIZONTAL_TURRET_ANGLE).withTimeout(0.3)
        ));
        break;
        case LEVEL_TWO:
        addCommands(new ParallelCommandGroup(
            new SetElevatorPosition(Constants.PositionConstants.LEVEL_TWO_ELEVATOR_HEIGHT).withTimeout(0.3),
            new SetPivotAngle(Constants.PositionConstants.LEVEL_TWO_PIVOT_ANGLE).withTimeout(0.3),
            new SetTurretAngle(Constants.PositionConstants.VERTICAL_TURRET_ANGLE).withTimeout(0.3)
        ));
        break;
        case LEVEL_THREE:
        addCommands(new ParallelCommandGroup(
            new SetElevatorPosition(Constants.PositionConstants.LEVEL_THREE_ELEVATOR_HEIGHT).withTimeout(0.3),
            new SetPivotAngle(Constants.PositionConstants.LEVEL_THREE_PIVOT_ANGLE).withTimeout(0.3),
            new SetTurretAngle(Constants.PositionConstants.VERTICAL_TURRET_ANGLE).withTimeout(0.3)
        ));
        break;
        case LEVEL_FOUR:
          addCommands(new ParallelCommandGroup(
            new SetElevatorPosition(Constants.PositionConstants.LEVEL_FOUR_ELEVATOR_HEIGHT).withTimeout(0.3),
            new SetPivotAngle(Constants.PositionConstants.LEVEL_FOUR_PIVOT_ANGLE).withTimeout(0.3),
            new SetTurretAngle(Constants.PositionConstants.VERTICAL_TURRET_ANGLE).withTimeout(0.3)
          ));
        break;
      }
  } 
}
