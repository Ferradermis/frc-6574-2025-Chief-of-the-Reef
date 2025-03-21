package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.FullTeleopSystemCommands.LowerPivotAndScore;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelFour;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelOne;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelThree;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelTwo;
import frc.robot.util.ReefPositionsUtil.ScoreLevel;

public class SelectorCommandFactory {
    public static Map<ScoreLevel,Command> getCoralLevelPrepCommandSelector() {
        Map<ScoreLevel,Command> commandMap = new HashMap<>();
        commandMap.put(ScoreLevel.L1, new ScoreLevelOne());
        commandMap.put(ScoreLevel.L2, new ScoreLevelTwo());
        commandMap.put(ScoreLevel.L3, new ScoreLevelThree());
        commandMap.put(ScoreLevel.L4, new ScoreLevelFour());
        return commandMap;
    }

    public static Map<ScoreLevel,Command> getCoralLevelScoreCommandSelector() {
        Map<ScoreLevel,Command> commandMap = new HashMap<>();
        commandMap.put(ScoreLevel.L1, new LowerPivotAndScore());
        commandMap.put(ScoreLevel.L2, new LowerPivotAndScore());
        commandMap.put(ScoreLevel.L3, new LowerPivotAndScore());
        commandMap.put(ScoreLevel.L4, new LowerPivotAndScore());
        return commandMap;
    }

    public static Map<ScoreLevel,Command> getCoralLevelStopScoreCommandSelector() {
        Map<ScoreLevel,Command> commandMap = new HashMap<>();
        commandMap.put(ScoreLevel.L1, new Intake(0));
        commandMap.put(ScoreLevel.L2, new Intake(0));
        commandMap.put(ScoreLevel.L3, new Intake(0));
        commandMap.put(ScoreLevel.L4, new Intake(0));
        return commandMap;
    }
}