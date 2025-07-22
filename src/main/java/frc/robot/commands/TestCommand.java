package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestCommand extends Command {
    public TestCommand() {
        
    }

    @Override
    public void initialize() {
        // Initialization logic here
        System.out.println("TestCommand initialized");
    }

    @Override
    public void execute() {
        // Execution logic here
    }

    @Override
    public boolean isFinished() {
        // Return true when the command should end
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Cleanup logic here if needed
    }
    
}
