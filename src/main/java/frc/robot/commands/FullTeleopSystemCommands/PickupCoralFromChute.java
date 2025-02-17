package frc.robot.commands.FullTeleopSystemCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake;
import frc.robot.commands.SetArmAngle;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetRotateAngle;

public class PickupCoralFromChute extends SequentialCommandGroup {
  public PickupCoralFromChute() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            new SetElevatorPosition(Constants.PositionConstants.CHUTE_ELEVATOR_HEIGHT),
            new SetArmAngle(Constants.PositionConstants.CHUTE_ARM_ANGLE),
            new SetRotateAngle(Constants.PositionConstants.HORIZONTAL_ROTATE_ANGLE)
        ),
        new Intake()
    );
  }
}
