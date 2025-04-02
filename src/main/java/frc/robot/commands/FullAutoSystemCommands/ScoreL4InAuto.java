package frc.robot.commands.FullAutoSystemCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetPivotAngle;
import frc.robot.commands.SetTurretAngle;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelFour;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelOne;

public class ScoreL4InAuto extends SequentialCommandGroup {
    public ScoreL4InAuto() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new SetPivotAngle(Constants.PositionConstants.HOME_PIVOT_ANGLE).withTimeout(0.3),
            new SetElevatorPosition(Constants.PositionConstants.LEVEL_FOUR_ELEVATOR_HEIGHT).withTimeout(0.3),
            new SetTurretAngle(Constants.PositionConstants.AUTO_ALIGN_VERTICAL_TURRET_ANGLE).withTimeout(0.3)
        );
    }
}
