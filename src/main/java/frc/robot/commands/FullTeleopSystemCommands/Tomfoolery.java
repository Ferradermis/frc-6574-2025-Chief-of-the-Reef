package frc.robot.commands.FullTeleopSystemCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Intake;
import frc.robot.commands.SetPivotAngle;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetTurretAngle;

public class Tomfoolery extends SequentialCommandGroup {
  public Tomfoolery() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
            new SetElevatorPosition(Constants.PositionConstants.ALGAE_NET_ELEVATOR_HEIGHT).withTimeout(0.3),
            new WaitCommand(1),
            new SetPivotAngle(Constants.PositionConstants.ALGAE_NET_PIVOT_ANGLE).withTimeout(0.3),
            new SetTurretAngle(Constants.PositionConstants.VERTICAL_TURRET_ANGLE).withTimeout(0.3)
    );
  } 
}
