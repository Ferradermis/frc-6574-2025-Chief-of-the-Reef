package frc.robot.commands.AutoAlignCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.FullAutoSystemCommands.ScoreL4InAuto;
import frc.robot.commands.FullTeleopSystemCommands.AlgaeReturnToHome;
import frc.robot.commands.FullTeleopSystemCommands.ReturnToHome;
import frc.robot.commands.FullTeleopSystemCommands.ScoreCoral;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelFour;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelFourNoAA;
import frc.robot.subsystems.drive.Drive;

public class AutoAlignAndScore extends SequentialCommandGroup {
  /** Creates a new AutoAlignAndScore. */
  public AutoAlignAndScore(Drive driveSubsystem, boolean isRtScore, int tag) {
    addCommands(
        new ReefAutoAlign(driveSubsystem, isRtScore, tag),
        new ScoreLevelFourNoAA(),
        new WaitCommand(0.25),
        new ScoringAutoAlign(driveSubsystem, isRtScore, tag),
        new ParallelCommandGroup(
          new ScoreCoral(),
          new RunCommand(() -> driveSubsystem.setControl(-0.2, 0, 0), driveSubsystem)
              .withTimeout(0.5)).withTimeout(1.0),
        new RunCommand(() -> driveSubsystem.setControl(0, 0, 0), driveSubsystem).withTimeout(0.1),
        new AlgaeReturnToHome()
      );
  }
}
