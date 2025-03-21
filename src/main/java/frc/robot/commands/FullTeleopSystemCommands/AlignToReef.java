package frc.robot.commands.FullTeleopSystemCommands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import java.util.Map;
import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ReefPositionsUtil;
import frc.robot.util.ReefPositionsUtil.ScoreLevel;

public class AlignToReef {

    private static AlignToReef instance;

    public static enum ReefPosition {
        Left,
        Center,
        Right,
    }
    private static Alliance alliance;
    private final static int[] targetIdsRed = {
        6,7,8,9,10,11
    };

    private final static int[] targetIdsBlue = {
        17,18,19,20,21,22
    };

    private static int[] targetIds;

    //#region TODO Find Accurate Values
    private static LoggedTunableNumber offsetBBackingUp = new LoggedTunableNumber("AutoAlign/offsetBBackingUp", 0.75);
    private static LoggedTunableNumber rightOffsetBFinal = new LoggedTunableNumber("AutoAlign/rightOffsetBFinal", 0.68);
    private static LoggedTunableNumber leftOffsetBFinal = new LoggedTunableNumber("AutoAlign/leftOffsetBFinal", 0.68);
    private static LoggedTunableNumber offsetL = new LoggedTunableNumber("AutoAlign/offsetL", 0.155);
    private static LoggedTunableNumber offsetR = new LoggedTunableNumber("AutoAlign/offsetR", 0.18);
    //#endregion

    /**
     * Finds the closest april tag to a position.
     * 
     * @param pos The Pose2d to find the closest relative tag.
     * @param targets The list of AprilTag IDs to check for.
     * @return The pose of the closest april tag in "targets" to "pos"
     */
    private static Pose2d findClosestPose(Pose2d pos) {
        int[] targets = targetIds;
        double minDistance = Double.MAX_VALUE;
        Pose2d target = Pose2d.kZero;
        
        for (int i = 0; i < targets.length; i++) {
            double distance = pos.getTranslation().getDistance(aprilTagLayout.getTagPose(targets[i]).orElse(Pose3d.kZero).getTranslation().toTranslation2d());
            if (distance < minDistance) {
                target = aprilTagLayout.getTagPose(targets[i]).orElse(Pose3d.kZero).toPose2d();
                minDistance = distance;
            }
        }
        
        return target;
    }

    /**
     * Returns A function which takes the current position on the robot and returns where we want to score.
     * @param pos
     * @param isBackingUp
     * @return
     */
    public static Function<Pose2d, Pose2d> getGetTargetPositionFunction(ReefPosition pos, boolean isBackingUp) {
        refreshAlliance();
        return (Pose2d pose) -> {
            double backOffset = leftOffsetBFinal.get();
            double appliedOffset = 0;
            switch (pos) {
                case Right:
                    appliedOffset = offsetR.getAsDouble();
                    backOffset = rightOffsetBFinal.get();
                    break;
                case Left:
                    appliedOffset = -offsetL.getAsDouble();
                    backOffset = leftOffsetBFinal.get();
                    break;
                case Center:
                default:
                    appliedOffset = 0;
                    break;
            }
            Transform2d offset = new Transform2d(isBackingUp ? offsetBBackingUp.getAsDouble() : backOffset, appliedOffset, Rotation2d.kZero);
            Pose2d closestTarget = findClosestPose(pose);

            Pose2d target = closestTarget.transformBy(offset);
            Logger.recordOutput("TargetPose",target);
            return target;
        };
    }

    public static void initialize() {
        refreshAlliance();
    }

    public static void refreshAlliance() {
        targetIds = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue ? targetIdsBlue : targetIdsRed;
    }

    /**
     * Gets a command which aligns the robot to the nearest reef position
     * @param position Left or right
     * @param isBackingUp Whether or not we should back up if we're too close to the reef
     */
    public static Command getNewAlignToReefCommand(ReefPosition position, boolean isBackingUp, Drive drive) {
        Function<Pose2d, Pose2d> positionFunction = getGetTargetPositionFunction(position, isBackingUp);
        System.out.println("Position Function: " + positionFunction);
        //Base command
        Command returnedCommand = new AutoAlign(getGetTargetPositionFunction(position, isBackingUp), drive);
        //If we're backing up, add kill conditions
        if(isBackingUp) {
            returnedCommand = returnedCommand
                // Kill when we are out of the distance (not necessary since we kill)
                // .until(() -> (drive.getDistanceTo(positionFunction.apply(drive.getPose())).in(Meters) > offsetBBackingUp.getAsDouble()))
                // Don't run the backup if we are out of the distance
                .unless(() -> (drive.getDistanceTo(positionFunction.apply(drive.getPose())).in(Meters) > offsetBBackingUp.getAsDouble()));
        }
        return returnedCommand;
    }

    /**
     * Gets a command which aligns the robot to the nearest reef position and prepares to score the coral, backing up if it's too close
     * @param position
     * @param coralLevelCommands
     * @param scoreCoralLevelCommands
     * @param drive
     * @return
     */
    public static Command getNewReefCoralScoreSequence(ReefPosition position, boolean isBackingUp, Map<ReefPositionsUtil.ScoreLevel,Command> coralLevelCommands, Map<ReefPositionsUtil.ScoreLevel,Command> scoreCoralLevelCommands, Map<ReefPositionsUtil.ScoreLevel,Command> stopCoralLevelCommands, Drive drive) {
        return 
            getNewAlignToReefCommand(position, true, drive).onlyIf(()->isBackingUp)
                .andThen(DriveCommands.brakeDrive(drive))
                .alongWith(ReefPositionsUtil.getInstance().getCoralLevelSelector(coralLevelCommands))
            .andThen(getNewAlignToReefCommand(position, false, drive))
            // .andThen(new WaitCommand(0.2))
            .andThen(ReefPositionsUtil.getInstance().getCoralLevelSelector(scoreCoralLevelCommands))
            // .andThen(new WaitCommand(0.2))
            .andThen(
                getNewAlignToReefCommand(position, true, drive).onlyIf(()->ReefPositionsUtil.getInstance().isSelected(ScoreLevel.L4))
                    .andThen(ReefPositionsUtil.getInstance().getCoralLevelSelector(stopCoralLevelCommands))
            );
    }

    /**
     * Gets a command that takes an algae in auto
     * @param drive
     * @return
     */
    public static Command getNewAutoReefAlgaeScoreSequenceCommand(Drive drive) {
        return getNewAlignToReefCommand(ReefPosition.Center, false, drive)
            .alongWith(new GrabAlgaeOne());
    }
}
