package frc.robot.commands.FullTeleopSystemCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetPivotAngle;
import frc.robot.commands.SetTurretAngle;

public class VerticalCoralIntake extends SequentialCommandGroup {

    public VerticalCoralIntake() {
     
        addCommands(
            new SetElevatorPosition(Constants.PositionConstants.VERTICAL_GROUND_CORAL_INTAKE_ELEVATOR_HEIGHT).withTimeout(.3),
            new SetTurretAngle(Constants.PositionConstants.VERTICAL_TURRET_ANGLE).withTimeout(.3),
            new SetPivotAngle(Constants.PositionConstants.VERTICAL_GROUND_CORAL_INTAKE_PIVOT_ANGLE).withTimeout(.3)
        );
    }
    
}
