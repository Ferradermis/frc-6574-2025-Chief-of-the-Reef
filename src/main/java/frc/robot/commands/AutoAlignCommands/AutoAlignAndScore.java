package frc.robot.commands.AutoAlignCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Release;
import frc.robot.commands.FullAutoSystemCommands.ScoreL4InAuto;
import frc.robot.commands.FullTeleopSystemCommands.AlgaeReturnToHome;
import frc.robot.commands.FullTeleopSystemCommands.ReturnToHome;
import frc.robot.commands.FullTeleopSystemCommands.ScoreCoral;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevel;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelFour;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelFourNoAA;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelOne;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelThree;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelTwo;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ReefPositions;
import frc.robot.util.ReefPositions.ReefLevel;

public class AutoAlignAndScore extends SequentialCommandGroup {
  ReefPositions reefPositions = ReefPositions.getInstance();
  /** Creates a new AutoAlignAndScore. */
  public AutoAlignAndScore(Drive driveSubsystem, boolean isRtScore, int tag) {
        addCommands(
          new ReefAutoAlign(driveSubsystem, isRtScore, tag),
          new ScoreLevel(reefPositions.getSelectedLevel()),
          new WaitCommand(0.25),
          new ScoringAutoAlign(driveSubsystem, isRtScore, tag),
          new ParallelCommandGroup(
            //new ScoreCoral(reefPositions.getSelectedLevel()),
            new RunCommand(() -> driveSubsystem.setControl(-0.2, 0, 0), driveSubsystem)
                .withTimeout(0.5)).withTimeout(1.0),
          new RunCommand(() -> driveSubsystem.setControl(0, 0, 0), driveSubsystem).withTimeout(0.1),
          new AlgaeReturnToHome()
        );
    }
  }
