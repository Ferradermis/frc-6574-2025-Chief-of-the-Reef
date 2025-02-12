package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class Intake extends Command {
    public Intake() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.endEffector);
    }

    @Override
    public void initialize() {
        // Called when the command is initially scheduled.
        RobotContainer.endEffector.getNewSetVoltsCommand(1); // TODO: Test command, default value is 1
    }

    @Override
    public void execute() {
        // Called every time the scheduler runs while the command is scheduled.
    }

    @Override
    public void end(boolean interrupted) {
        // Called once the command ends or is interrupted.
        RobotContainer.endEffector.getNewSetVoltsCommand(0);
    }

    @Override
    public boolean isFinished() {
        // Make this return true when this Command no longer needs to run execute()
        return false;
    }
}
