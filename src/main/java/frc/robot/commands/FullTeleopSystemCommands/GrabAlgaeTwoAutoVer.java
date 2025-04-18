package frc.robot.commands.FullTeleopSystemCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake;
import frc.robot.commands.SetPivotAngle;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetTurretAngle;

public class GrabAlgaeTwoAutoVer extends SequentialCommandGroup {
  public GrabAlgaeTwoAutoVer() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            new SetPivotAngle(Constants.PositionConstants.ALGAE_TWO_AUTO_PIVOT_ANGLE),
            new SetElevatorPosition(Constants.PositionConstants.ALGAE_TWO_AUTO_ELEVATOR_HEIGHT),
            new SetTurretAngle(Constants.PositionConstants.VERTICAL_TURRET_ANGLE) // Coral EE: ALGAE_HORIZONTAL_TURRET_ANGLE
        )
    );
  } 
}
