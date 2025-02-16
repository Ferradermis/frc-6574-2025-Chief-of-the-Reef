package frc.robot.commands.FullTeleopSystemCommands;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake;
import frc.robot.commands.SetArmAngle;
import frc.robot.commands.SetClimberAngle;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetRotateAngle;

public class ReturnToHome extends SequentialCommandGroup {
  public ReturnToHome() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            new SetElevatorPosition(0),
            new SetArmAngle(0),
            new SetRotateAngle(0),
            new SetClimberAngle(Degrees.of(0), Degrees.of(0))
        ),
        new Intake()
    );
  }   
}
