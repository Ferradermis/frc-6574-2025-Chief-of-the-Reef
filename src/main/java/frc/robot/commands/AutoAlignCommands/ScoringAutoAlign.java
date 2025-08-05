package frc.robot.commands.AutoAlignCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;

public class ScoringAutoAlign extends Command {
  public double xP = 2.0; // Working value: 2.0
  public double yP = 1.5; // Working value: 1.2
  public double rotationP = 0.02; // Working value: 0.08

  private PIDController xController, yController, rotController;
  public boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer, timeoutTimer;
  int tagID = -1;
  private Drive drive;

  double ySetpoint = 0.0;
  double xSetpoint = 0.0;
  double rotSetpoint = 0.0;
  double tolerance = 0.01;
  double rotTolerance = 0.5;

  /**
   * Creates a new AutoAlignTest, auto aligns to a position on the reef according to the tag given
   * and which pole on the reef to align to (left or right)
   *
   * @param driveSubsystem The drive subsystem to use
   * @param isRtScore Whether the robot is aligning to the right side pole or not
   * @param tag The Apriltag ID to align to
   */
  public ScoringAutoAlign(Drive driveSubsystem, boolean isRtScore, int tag) {
    // TODO: Remake this class to use pose instead of tx, tz, and ry (please dear god I hate this
    // version xd)
    // Okay I hate this less now it might be okay
    xController = new PIDController(xP, 0, 0);
    yController = new PIDController(yP, 0, 0);
    rotController = new PIDController(rotationP, 0, 0);

    isRightScore = isRtScore;
    drive = driveSubsystem;
    tagID = tag;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    System.out.println("AutoAlignTest initialized with tag ID: " + tagID); // Debugging output
    stopTimer = new Timer();
    stopTimer.start();
    dontSeeTagTimer = new Timer();
    dontSeeTagTimer.start();
    timeoutTimer = new Timer();
    timeoutTimer.start();

    if (isRightScore) {
      xSetpoint = Constants.AutoAlignConstants.xRightScoringSetpoint;
      ySetpoint = Constants.AutoAlignConstants.yRightScoringSetpoint;
      rotSetpoint = Constants.AutoAlignConstants.rotRightScoringSetpoint;
    } else {
      xSetpoint = Constants.AutoAlignConstants.xLeftScoringSetpoint;
      ySetpoint = Constants.AutoAlignConstants.yLeftScoringSetpoint;
      rotSetpoint = Constants.AutoAlignConstants.rotLeftScoringSetpoint;
    }

    xController.setSetpoint(xSetpoint);
    xController.setTolerance(tolerance);

    yController.setSetpoint(ySetpoint);
    yController.setTolerance(tolerance);

    rotController.setSetpoint(rotSetpoint);
    rotController.setTolerance(rotTolerance);
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight-left")
        && LimelightHelpers.getFiducialID("limelight-left") == tagID
        && !isRightScore) {
      dontSeeTagTimer.reset();

      double[] positions = LimelightHelpers.getBotPose_TargetSpace("limelight-left");

      double xSpeed = xController.calculate(positions[2]);
      double ySpeed = -yController.calculate(positions[0]);
      double rotValue = -rotController.calculate(positions[4]);

      drive.setControl(xSpeed, ySpeed, rotValue);

      if (!rotController.atSetpoint() || !xController.atSetpoint() || !yController.atSetpoint()) {
        stopTimer.reset();
      } else {
        drive.setControl(0, 0, 0);
      }
    } else if (LimelightHelpers.getTV("limelight-right")
        && LimelightHelpers.getFiducialID("limelight-right") == tagID
        && isRightScore) {
      dontSeeTagTimer.reset();

      double[] positions = LimelightHelpers.getBotPose_TargetSpace("limelight-right");

      double xSpeed = xController.calculate(positions[2]);
      double ySpeed = -yController.calculate(positions[0]);
      double rotValue = -rotController.calculate(positions[4]);

      drive.setControl(xSpeed, ySpeed, rotValue);

      if (!rotController.atSetpoint() || !xController.atSetpoint() || !yController.atSetpoint()) {
        stopTimer.reset();
      } else {
        drive.setControl(0, 0, 0);
      }
    }
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag
    // in the camera
    if (this.dontSeeTagTimer.hasElapsed(Constants.AutoAlignConstants.dontSeeTagTime)
        || stopTimer.hasElapsed(Constants.AutoAlignConstants.validationTime)
        || timeoutTimer.hasElapsed(Constants.AutoAlignConstants.timeoutTime)) {
            System.out.println("ScoringAutoAlign finished"); // Debugging output
            return true;
        }
    else {
      return false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("ScoringAutoAlign ended, interrupted: " + interrupted); // Debugging output
  }
}
