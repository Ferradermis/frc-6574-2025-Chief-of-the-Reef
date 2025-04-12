package frc.robot.commands.FullTeleopSystemCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetTurretAngle;

public class ScoreCoral extends SequentialCommandGroup {
    public ScoreCoral() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new SetElevatorPosition(Constants.PositionConstants.LOWER_L4_ELEVATOR_HEIGHT).withTimeout(0.3)
        );
    }
    
}
