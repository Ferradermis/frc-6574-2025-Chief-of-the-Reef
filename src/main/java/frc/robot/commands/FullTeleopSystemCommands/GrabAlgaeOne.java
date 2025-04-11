package frc.robot.commands.FullTeleopSystemCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake;
import frc.robot.commands.SetPivotAngle;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetTurretAngle;

public class GrabAlgaeOne extends SequentialCommandGroup {
  public GrabAlgaeOne() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            new SetElevatorPosition(Constants.PositionConstants.ALGAE_ONE_ELEVATOR_HEIGHT).withTimeout(0.3),
            new SetTurretAngle(Constants.PositionConstants.VERTICAL_TURRET_ANGLE).withTimeout(0.3) // Coral EE: ALGAE_HORIZONTAL_TURRET_ANGLE
        ).withTimeout(0.3),
        new SetPivotAngle(Constants.PositionConstants.ALGAE_ONE_PIVOT_ANGLE)
    );
  } 
}
