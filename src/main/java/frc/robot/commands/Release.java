package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class Release extends Command {
    private double speed;
    public Release(double s) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.endEffector);
        speed = s;
    }

    @Override
    public void initialize() {
        // Called when the command is initially scheduled.
        
    }

    @Override
    public void execute() {
        // Called every time the scheduler runs while the command is scheduled.
        RobotContainer.endEffector.setTarget(Volts.of(-speed)); // TODO: Test command, default value is -1
    }

    @Override
    public void end(boolean interrupted) {
        // Called once the command ends or is interrupted.
        RobotContainer.endEffector.setTarget(Volts.of(0));
    }

    @Override
    public boolean isFinished() {
        // Make this return true when this Command no longer needs to run execute()
        return false;
    }
}
