package frc.robot.subsystems.vision;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;

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

        if (observation.type() == PoseObservationType.MEGATAG_1) {
            if (DriverStation.isAutonomousEnabled() || DriverStation.isTeleopEnabled()) {
                return true;
            }
        }
        return super.rejectPose(observation);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Vision/UpdateBasedOnTags", updateWithTags);
        super.periodic();
    }

    
}
