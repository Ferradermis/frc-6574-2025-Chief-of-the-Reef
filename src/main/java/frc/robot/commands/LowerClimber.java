package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LowerClimber extends Command {

    public LowerClimber() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.climber);
    }

    @Override
    public void initialize() {
        // Called when the command is initially scheduled.
        if (RobotContainer.climber.isClimberDeployed() == false) {
            RobotContainer.climber.setAngle(Constants.PositionConstants.CLIMBER_DOWN_ANGLE);
            RobotContainer.climber.setIsClimberDeployed(true);
        }
    }

    @Override
    public void execute() {
        // Called every time the scheduler runs while the command is scheduled.
    }

    @Override
    public void end(boolean interrupted) {
        // Called once the command ends or is interrupted.
    }

    @Override
    public boolean isFinished() {
        // Make this return true when this Command no longer needs to run execute()
        return false;
    }
}
