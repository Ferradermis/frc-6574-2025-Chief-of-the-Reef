package frc.robot.commands.FullTeleopSystemCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SetPivotAngle;
import frc.robot.commands.Intake;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetTurretAngle;

public class ReturnToHome extends SequentialCommandGroup {
  public ReturnToHome() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
            new SetTurretAngle(Constants.PositionConstants.HORIZONTAL_TURRET_ANGLE).withTimeout(0.3),
            new SetPivotAngle(Constants.PositionConstants.HOME_PIVOT_ANGLE).withTimeout(0.3),
            new ParallelCommandGroup(
              new SetElevatorPosition(Constants.PositionConstants.HOME_ELEVATOR_HEIGHT)
        )
    );
  }   
}
