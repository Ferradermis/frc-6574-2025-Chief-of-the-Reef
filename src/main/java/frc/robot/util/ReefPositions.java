package frc.robot.util;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class ReefPositions {
    
    public static enum ReefLevel {
        LEVEL_ONE, 
        LEVEL_TWO, 
        LEVEL_THREE, 
        LEVEL_FOUR
    }

    // Default 
    private ReefLevel selectedLevel = ReefLevel.LEVEL_ONE;

    private static ReefPositions instance;

    public static ReefPositions getInstance() {
        if (instance == null) {
            instance = new ReefPositions();
        }
        return instance;
    }

    private ReefPositions() {
        selectedLevel = ReefLevel.LEVEL_ONE;
    }

    /**
     * Gets the reef level that is currently selected.
     * 
     * @return the currently selected reef level
     */
    public ReefLevel getSelectedLevel() {
        return selectedLevel;
    }

    /**
     * Sets the selected reef level.
     * 
     * @param level the desired level to select
     */
    public void setReefLevel(ReefLevel level) {
        selectedLevel = level;
    }

    /**
     * Creates a new command that sets the selected reef level.
     * 
     * @param level the reef level to select
     * @return an instant command that runs the set method
     */
    public InstantCommand getNewSetReefLevelCommand(ReefLevel level) {
        System.out.println("Setting reef level to: " + level);
        return new InstantCommand(() -> setReefLevel(level));
    }
}
