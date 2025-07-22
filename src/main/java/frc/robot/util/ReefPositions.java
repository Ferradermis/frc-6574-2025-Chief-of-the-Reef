package frc.robot.util;

public class ReefPositions {
    
    public static enum ReefLevel {
        LEVEL_ONE, 
        LEVEL_TWO, 
        LEVEL_THREE, 
        LEVEL_FOUR
    }

    // Default 
    private ReefLevel selectedLevel = ReefLevel.LEVEL_ONE;

    private ReefPositions() {
        selectedLevel = ReefLevel.LEVEL_ONE;
    }

    public ReefLevel getSelectedLevel() {
        return selectedLevel;
    }

    public void setReefLevel(ReefLevel level) {
        selectedLevel = level;
    }
}
