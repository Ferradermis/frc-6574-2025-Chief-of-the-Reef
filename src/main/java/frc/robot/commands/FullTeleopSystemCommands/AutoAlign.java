package frc.robot.commands.FullTeleopSystemCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.function.Function;

import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableGainsBuilder;
import frc.robot.util.LoggedTunableNumber;

public class AutoAlign extends Command {
    
    private Drive drivetrain;

    private Pose2d targetPose;

    private final Function<Pose2d, Pose2d> getTargetPoseFn;

    //#region TODO get accurate values
    private LoggedTunableGainsBuilder throttleGains = new LoggedTunableGainsBuilder("AutoAlign/strafeGains/", 6.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    private LoggedTunableGainsBuilder strafeGains = new LoggedTunableGainsBuilder("AutoAlign/throttleGains/", 4.0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0);
    private LoggedTunableNumber maxStrafeTune = new LoggedTunableNumber("AutoAlign/strafeGains/maxVelMetersPerSecond",1.0);
    private LoggedTunableNumber maxThrottleTune = new LoggedTunableNumber("AutoAlign/throttleGains/maxVelMetersPerSecond",1.0);
    private LoggedTunableNumber maxAccelStrafeTune = new LoggedTunableNumber("AutoAlign/strafeGains/maxAccMetersPerSecond",10.0);
    private LoggedTunableNumber maxAccelDistanceTune = new LoggedTunableNumber("AutoAlign/throttleGains/maxAccMetersPerSecond",10.0);
    private LoggedTunableNumber toleranceB = new LoggedTunableNumber("AutoAlign/toleranceB", 0.01);
    private LoggedTunableNumber toleranceR = new LoggedTunableNumber("AutoAlign/toleranceR", 0.02);
    //#endregion

    private LinearVelocity maxStrafe = MetersPerSecond.of(maxStrafeTune.getAsDouble()); 
    private LinearVelocity maxThrottle = MetersPerSecond.of(maxThrottleTune.getAsDouble());
    private LinearAcceleration maxAccelStrafe = MetersPerSecondPerSecond.of(maxAccelStrafeTune.getAsDouble());
    private LinearAcceleration maxAccelThrottle = MetersPerSecondPerSecond.of(maxAccelStrafeTune.getAsDouble());
    private static final double MAX_SPIN = Math.toRadians(180.0);

    private double strafe;
    private double throttle;
    private double spin;
    private double tx;
    private double ty;
    private double vx;
    private double vy;
    private double tr;
    private ProfiledPIDController strafePID = new ProfiledPIDController(strafeGains.build().kP, strafeGains.build().kI ,strafeGains.build().kD, new Constraints(maxStrafe.in(MetersPerSecond), maxAccelStrafe.in(MetersPerSecondPerSecond)));
    private ProfiledPIDController throttlePID = new ProfiledPIDController(throttleGains.build().kP, throttleGains.build().kI ,throttleGains.build().kD, new Constraints(maxThrottle.in(MetersPerSecond), maxAccelThrottle.in(MetersPerSecondPerSecond)));
    private PIDController spinPID = new PIDController(5.0, 0.0, 0.0);

    /**
     * This command utilitzes the swerve drive while it isn't field relative.
     * The swerve drive returns back to field relative after the command is used.
     * @param getDrivePoseFunction A function that takes a current drivetrain pose and returns a target position.
     * @param drivetrain The Drive class to get the current pose from.
     * @param name The LoggedTunableNumber's (should be) exclusive name
     */
    public AutoAlign(Function<Pose2d, Pose2d> getTargetPoseFunction, Drive drivetrain, String name) {
        this.getTargetPoseFn = getTargetPoseFunction;
        this.drivetrain = drivetrain;
    }

    /**
     * This command utilitzes the swerve drive while it isn't field relative.
     * The swerve drive returns back to field relative after the command is used.
     * @param getDrivePoseFunction A function that takes a current drivetrain pose and returns a target position.
     * @param drivetrain The Drive class to get the current pose from.
     */
    public AutoAlign(Function<Pose2d, Pose2d> getTargetPoseFunction, Drive drivetrain) {
        this(getTargetPoseFunction, drivetrain, "AutoAlign");
    }

    /**
     * Sets the gains to the current values in the LoggedTunableNumbers of this class
     */
    private void resetGains() {
        maxStrafe = MetersPerSecond.of(maxStrafeTune.getAsDouble()); 
        maxThrottle = MetersPerSecond.of(maxThrottleTune.getAsDouble());
        maxAccelStrafe = MetersPerSecondPerSecond.of(maxAccelStrafeTune.getAsDouble());
        maxAccelThrottle = MetersPerSecondPerSecond.of(maxAccelDistanceTune.getAsDouble());

        strafePID = new ProfiledPIDController(strafeGains.build().kP, strafeGains.build().kI ,strafeGains.build().kD, new Constraints(maxStrafe.in(MetersPerSecond), maxAccelStrafe.in(MetersPerSecondPerSecond)));
        throttlePID = new ProfiledPIDController(throttleGains.build().kP, throttleGains.build().kI ,throttleGains.build().kD, new Constraints(maxThrottle.in(MetersPerSecond), maxAccelThrottle.in(MetersPerSecondPerSecond)));
    }

    /**
     * Resets the target pose based on {@link getTargetPoseFn}
     * @return The new target pose
     */
    private Pose2d getNewTargetPose() {
        targetPose = getTargetPoseFn.apply(getCurrentPose());
        return targetPose;
    }

    /**
     * @return The target pose relative to the robot pose.
     */
    private Pose2d getRelativeTarget() {
        return targetPose.relativeTo(getCurrentPose());
    }

    private Pose2d getCurrentPose() {
        return drivetrain.getAutoAlignPose();
    }

    @Override
    public void initialize() {
        resetGains();

        this.targetPose = getNewTargetPose();
        Pose2d targetPose_R = getRelativeTarget();

        tx = -targetPose_R.getY();
        ty = -targetPose_R.getX();
        tr = targetPose_R.getRotation().unaryMinus().getRadians();
        vx = drivetrain.getChassisSpeeds().vxMetersPerSecond;
        vy = drivetrain.getChassisSpeeds().vyMetersPerSecond;

        strafePID.reset(tx,vy);
        throttlePID.reset(ty,vx);
        spinPID.reset();
    }

    /* 
    * This command utilitzes the swerve drive while it isn't field relative. 
    * The swerve drive returns back to field relative after the command is used.
    */
    @Override
    public void execute() {
        Pose2d targetPose_r = getRelativeTarget();

        double distance = getCurrentPose().getTranslation().getDistance(targetPose.getTranslation());

        tx = 0.0 - targetPose_r.getY();
        ty = 0.0 - targetPose_r.getX();
        tr = targetPose_r.getRotation().unaryMinus().getRadians();

        strafe = strafePID.calculate(tx, 0.0); 
        throttle = throttlePID.calculate(ty, 0.0);
        spin = MathUtil.clamp(spinPID.calculate(tr, 0.0),-MAX_SPIN,MAX_SPIN);

        ChassisSpeeds speeds =
        new ChassisSpeeds(
            throttle,
            strafe,
            spin);
        drivetrain.runVelocity(speeds);

        Logger.recordOutput("AutoAlign/TX", tx);
        Logger.recordOutput("AutoAlign/TZ", ty);
        Logger.recordOutput("AutoAlign/TR", tr);
        Logger.recordOutput("AutoAlign/strafe", strafe);
        Logger.recordOutput("AutoAlign/throttle", throttle);
        Logger.recordOutput("AutoAlign/spin", spin);
        Logger.recordOutput("AutoAlign/TargetPose",targetPose);
        Logger.recordOutput("AutoAlign/distance", distance);
    }

    /**
     * Will return if the robot is within the tolerance of the target pose.
     */
    @Override
    public boolean isFinished() {
        return MathUtil.isNear(tx, 0.0,toleranceR.getAsDouble()) && MathUtil.isNear(ty, 0.0,toleranceB.getAsDouble()) && MathUtil.isNear(tr, 0.0,(toleranceB.getAsDouble()+toleranceR.getAsDouble())/2.0);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.runVelocity(new ChassisSpeeds());
    }
}
