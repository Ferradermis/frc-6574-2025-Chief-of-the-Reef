package frc.robot.commands.FullTeleopSystemCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake;
import frc.robot.commands.SetArmAngle;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetRotateAngle;

public class PickupCoralFromGround extends SequentialCommandGroup {
  public PickupCoralFromGround() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            new SetElevatorPosition(Constants.PositionConstants.GROUND_ELEVATOR_HEIGHT),
            new SetArmAngle(Constants.PositionConstants.GROUND_ARM_ANGLE),
            new SetRotateAngle(Constants.PositionConstants.VERTICAL_ROTATE_ANGLE)
        ),
        new Intake()
    );
  }
}
