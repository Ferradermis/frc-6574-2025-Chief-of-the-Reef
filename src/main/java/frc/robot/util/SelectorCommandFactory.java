package frc.robot.util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.FullAutoSystemCommands.ReleaseInAuto;
import frc.robot.commands.FullTeleopSystemCommands.ScoreCoral;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelFourNoAA;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelOne;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelThree;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelTwo;
import frc.robot.util.ReefPositions.ReefLevel;

public class SelectorCommandFactory {
    public static Map<ReefLevel, Command> getReefLevelCommandSelector() {
        Map<ReefLevel, Command> commandMap = new HashMap<>();
        commandMap.put(ReefLevel.LEVEL_ONE, new ScoreLevelOne());
        commandMap.put(ReefLevel.LEVEL_TWO, new ScoreLevelTwo());
        commandMap.put(ReefLevel.LEVEL_THREE, new ScoreLevelThree());
        commandMap.put(ReefLevel.LEVEL_FOUR, new ScoreLevelFourNoAA());
        return commandMap;
    }

    public static Map<ReefLevel, Command> getReefLevelScoringCommandSelector() {
        Map<ReefLevel, Command> commandMap = new HashMap<>();
        commandMap.put(ReefLevel.LEVEL_ONE, new ReleaseInAuto());
        commandMap.put(ReefLevel.LEVEL_TWO, new ScoreCoral());
        commandMap.put(ReefLevel.LEVEL_THREE, new ScoreCoral());
        commandMap.put(ReefLevel.LEVEL_FOUR, new ScoreCoral());
        return commandMap;
    }
}
