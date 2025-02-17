package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SetElevatorPosition extends Command {
    public double distance;
    public SetElevatorPosition(double d) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.elevator);
        distance = d;
    }

    @Override
    public void initialize() {
        // Called when the command is initially scheduled.
        RobotContainer.elevator.getNewSetDistanceCommand(distance); // TODO: test command, defaulted to 0
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
