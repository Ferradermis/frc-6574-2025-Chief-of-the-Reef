package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SetTurretAngle extends Command {
    double angle;

    public SetTurretAngle(double a) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.turret);
        angle = a;
    }

    @Override
    public void initialize() {
        // Called when the command is initially scheduled.
        RobotContainer.turret.setAngle(angle); // TODO: test command, defaulted to 0
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
