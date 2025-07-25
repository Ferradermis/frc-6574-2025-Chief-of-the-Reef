package frc.robot.commands.AutoAlignCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.ReefPositions;
import frc.robot.util.ReefPositions.ReefLevel;

public class SelectReefLevel extends Command {

    ReefPositions reefPositions = ReefPositions.getInstance();
    ReefLevel reefLevel;

    public SelectReefLevel(ReefLevel level) {
        reefLevel = level;
    }

    @Override
    public void initialize() {
        System.out.println("Setting reef level to: " + reefLevel);
        reefPositions.setReefLevel(reefLevel);
        System.out.println("Current reef level is now: " + reefPositions.getSelectedLevel());
    }

    @Override
    public boolean isFinished() {
        return true; // This command finishes immediately after setting the level
    }
    
}
