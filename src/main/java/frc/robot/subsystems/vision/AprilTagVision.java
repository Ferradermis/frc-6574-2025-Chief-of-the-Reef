package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;
import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;

public class AprilTagVision extends Vision {

    private boolean updateWithTags = true;
    private Consumer<Pose2d> resetPose;
    private boolean hasRunAuto = false;
    private boolean hasSeenTags = false;
    private Trigger checkAllianceChangeTrigger = null;
    private boolean isFirstTime = true;

    public AprilTagVision(Consumer<Pose2d> reset, VisionConsumer consumer, VisionConsumer consumerAA, VisionIO... io) {
        super(consumer, consumerAA, io);
        resetPose = reset;
    }

    @Override
    public boolean rejectPose(PoseObservation observation) {
        if (!updateWithTags) {
            return true;
        }

        // if (observation.type() == PoseObservationType.MEGATAG_1) {
        //     if (DriverStation.isAutonomousEnabled() || DriverStation.isTeleopEnabled()) {
        //         return true;
        //     }
        // }
        return super.rejectPose(observation);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Vision/UpdateBasedOnTags", updateWithTags);
        super.periodic();
    }

    @Override
    public void addVisionMeasurement(Pose2d pose, double timestamp, Vector<N3> fill) {
        // tell system apriltag was used
        updateTags();
        super.addVisionMeasurement(pose, timestamp, fill);
    }

    public void updateAuto() {
        hasRunAuto = true;
    }

    public void updateTags() {
        hasSeenTags = true;
    }

    /** Sets the robot position based on the alliance if there is one. */
    public void setRobotPositionBasedOnAlliance() {
      Pose2d temp;
      Pose2d pose;
      Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
      if (optionalAlliance.isPresent()) {
        Alliance alliance = optionalAlliance.get();
        if (alliance == Alliance.Red) {
          temp = aprilTagLayout.getTagPose(7).get().toPose2d();
          pose =
              new Pose2d(
                  temp.getX() + 2.0,
                  temp.getY(),
                  temp.getRotation().plus(Rotation2d.fromDegrees(180.0)));
        } else {
          temp = aprilTagLayout.getTagPose(18).get().toPose2d();
          pose =
              new Pose2d(
                  temp.getX() - 2.0,
                  temp.getY(),
                  temp.getRotation().plus(Rotation2d.fromDegrees(180.0)));
        }

        resetPose.accept(pose);
      }
  }

  public void setPoseUsingTags() {
    Pose2d pose;
    if (LimelightHelpers.getTV("limelight")) {
      pose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
      resetPose.accept(pose);
    }
  }

  /** Create a trigger and waits to update the alliance position when it is available */
  public void createTriggerForSimulation() {
    if (checkAllianceChangeTrigger == null) {
      checkAllianceChangeTrigger =
          new Trigger(() -> checkIsAlliancePresent())
              .onTrue(
                  (new InstantCommand(() -> setRobotPositionBasedOnAlliance()))
                      .ignoringDisable(true));
    }
  }

  /**
   * Returns the alliance
   *
   * @return
   */
  private boolean checkIsAlliancePresent() {
    Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
    if (isFirstTime) {
      isFirstTime = false;
      return false;
    }
    return optionalAlliance.isPresent();
  }

  /**
   * If auto hasn't been run and the robot hasn't seen april tags, it will then update the odoemtry
   * when it enters teleop
   */
  public void updateStartingPosition() {
    if (hasRunAuto == false && hasSeenTags == false) {
      if (Robot.isReal()) {
        setRobotPositionBasedOnAlliance();
      } else {
        createTriggerForSimulation();
      }
    }
  }

  /**
   * Determine if odometry will be updated based on April tag
   *
   * @return ifAprilTagUpdatesOdomotry
   */
  public boolean isUpdateOdometryBaseOnApriltags() {
    return updateWithTags;
  }

  /**
   * set whether next april tag will be used for updating odometry
   *
   * @param updateOdometryBasedOnApriltags
   */
  private void setUpdateOdometryBasedOnApriltags(boolean enable) {
    updateWithTags = enable;
  }

  /**
   * Disable next if april tag will be used for updating odometry
   *
   * @param disableUpdateOdometryBasedOnApriltags
   */
  public void disableUpdateOdometryBasedOnApriltags() {
    setUpdateOdometryBasedOnApriltags(false);
  }

  /**
   * Disable next if april tag will be used for updating odometry
   *
   * @param enableUpdateOdometryBasedOnApriltags
   */
  public void enableUpdateOdometryBasedOnApriltags() {
    setUpdateOdometryBasedOnApriltags(true);
  }
}
