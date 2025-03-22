package frc.robot.commands.FullTeleopSystemCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.RaiseClimber;
import frc.robot.commands.SetPivotAngle;

public class Climb extends SequentialCommandGroup{
    public Climb() {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        addCommands(
            new ParallelCommandGroup(
                new RaiseClimber(),
                new SetPivotAngle(Constants.PositionConstants.CLIMB_PIVOT_ANGLE)
            )
        );
    }   
}
