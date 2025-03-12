package frc.robot.commands.FullTeleopSystemCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Intake;
import frc.robot.commands.SetPivotAngle;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetTurretAngle;

public class ScoreLevelOne extends SequentialCommandGroup {
  public ScoreLevelOne() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
            new SetTurretAngle(Constants.PositionConstants.HORIZONTAL_TURRET_ANGLE).withTimeout(0.3),
            new SetElevatorPosition(Constants.PositionConstants.LEVEL_ONE_ELEVATOR_HEIGHT).withTimeout(0.3),
            new WaitCommand(0.2),
            new SetPivotAngle(Constants.PositionConstants.LEVEL_ONE_PIVOT_ANGLE).withTimeout(0.3)
    );
  }
}
