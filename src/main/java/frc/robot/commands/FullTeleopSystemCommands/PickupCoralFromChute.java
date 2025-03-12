package frc.robot.commands.FullTeleopSystemCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake;
import frc.robot.commands.SetPivotAngle;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetTurretAngle;

public class PickupCoralFromChute extends SequentialCommandGroup {
  public PickupCoralFromChute() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            new SetElevatorPosition(Constants.PositionConstants.CHUTE_ELEVATOR_HEIGHT),
            new SetPivotAngle(Constants.PositionConstants.CHUTE_PIVOT_ANGLE),
            new SetTurretAngle(Constants.PositionConstants.HORIZONTAL_TURRET_ANGLE)
        )
    );
  }
}
