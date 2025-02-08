package frc.robot;

import static edu.wpi.first.units.Units.*;

import javax.xml.crypto.dsig.Transform;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.VirtualSubsystem;

public class RobotState extends VirtualSubsystem {
  public static RobotState instance;
  private MutDistance elevatorPosition = Inches.mutable(0);
  private MutAngle armPosition = Degrees.mutable(0);
  private MutAngle climberTwist = Degrees.mutable(0);
  public MutAngle rotateTwist = Degrees.mutable(0);

  private final Mechanism2d primaryMechanism2d;
  private final MechanismRoot2d primaryMechanismRoot2d;
  private final MechanismLigament2d armLigament2d;
  private final MechanismLigament2d elevatorLigament2d;
  private final MechanismLigament2d climberMount2d;
  private final MechanismRoot2d climberMountRoot2d;
  private final MechanismLigament2d climberLigament2d;

  private final MechanismRoot2d robotBaseRoot;
  private final MechanismLigament2d robotBaseLigament2d = new MechanismLigament2d("RobotBase", 100, 0, 10, new Color8Bit(255, 0, 0));

  private final String key;

  private RobotState(String k) {
    key = k;

    primaryMechanism2d = new Mechanism2d(500, 500);
    elevatorLigament2d = new MechanismLigament2d("ElevatorLigament", elevatorPosition.in(Centimeters), 90, 10, new Color8Bit(255, 0, 255));
    armLigament2d = new MechanismLigament2d("ArmLigament", Centimeters.convertFrom(10, Inches), armPosition.in(Degrees), 10, new Color8Bit(0, 255, 0));
    climberLigament2d = new MechanismLigament2d("ClimberLigament", Centimeters.convertFrom(10, Inches), climberTwist.in(Degrees), 10, new Color8Bit(0, 225, 255));
    climberMount2d = new MechanismLigament2d("ClimberMount2d", Centimeters.convertFrom(5, Inches), 90, 10, new Color8Bit(255, 255, 255));

    primaryMechanismRoot2d = primaryMechanism2d.getRoot("Primary2d", 150, 13);
    primaryMechanismRoot2d.append(elevatorLigament2d);
    elevatorLigament2d.append(armLigament2d);

    climberMountRoot2d = primaryMechanism2d.getRoot("ClimberMount2d", 105, 10);
    climberMountRoot2d.append(climberMount2d);
    climberMount2d.append(climberLigament2d);

    robotBaseRoot = primaryMechanism2d.getRoot("RobotBase2d", 100, 5);
    robotBaseRoot.append(robotBaseLigament2d);

    SmartDashboard.putData(key, primaryMechanism2d);
  }

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState("RobotState");
    }
    return instance;
  }

  @Override
  public void periodic() {
    visualize();
  }

  public Distance getElevatorPosition() {
    return elevatorPosition;
  }

  public Angle getArmPosition() {
    return armPosition;
  }

  public Angle getClimberTwist() {
    return climberTwist;
  }

  public Angle getRotateTwist() {
    return rotateTwist;
  }

  public void setElevatorPosition(Distance position) {
    elevatorPosition.mut_replace(position);
  }

  public void setArmPosition(Angle position) {
    armPosition.mut_replace(position);
  }

  public void setRotateTwist(Angle twist) {
    rotateTwist.mut_replace(twist);
  }

  public void setClimberTwist(Angle twist) {
    climberTwist.mut_replace(twist);
  }

  public void setElevatorSource(MutDistance elevatorposition) {
    elevatorPosition = elevatorposition;
  }

  public void setArmSource(MutAngle armposition) {
    armPosition = armposition;
  }

  public void setRotateSource(MutAngle rotatetwist) {
    rotateTwist = rotatetwist;
  }

  public void setClimberSource(MutAngle climbertwist) {
    climberTwist = climbertwist;
  }

  private void visualize() {
    Pose3d elevatorPose =
      new Pose3d(elevatorAttachOffset.getTranslation(), elevatorAttachOffset.getRotation())
      .transformBy(
        new Transform3d(
          new Translation3d(
            Meters.zero(), elevatorPosition, Meters.zero()), 
            new Rotation3d())
    );

    Pose3d armPose =
    elevatorPose
      .transformBy(armAttachOffset)
      .transformBy(
        new Transform3d(
          new Translation3d(), 
            new Rotation3d(
              armPosition, Degrees.zero(), Degrees.zero())))
      .transformBy(armPivotOffset.inverse()
      );
    
    Pose3d climberPose =
    armPose
      .transformBy(climberAttachOffset)
      .transformBy(
        new Transform3d(
          new Translation3d(), 
            new Rotation3d(
              Degrees.zero(), climberTwist, Degrees.zero())))
      .transformBy(climberPivotOffset.inverse()
      ); 

    elevatorLigament2d.setLength(elevatorPosition.in(Centimeters));
    armLigament2d.setAngle(armPosition.in(Degrees) + 180);
    climberLigament2d.setAngle(climberTwist.in(Degrees));

    Logger.recordOutput("RobotState/Elevator/" + key, elevatorPose);
    Logger.recordOutput("RobotState/Arm/" + key, armPose);
    Logger.recordOutput("RobotState/Climber/" + key, climberPose);
  }
  
  private static final Transform3d elevatorAttachOffset =
    new Transform3d(
      new Translation3d(Inches.of(-1), Inches.of(-0.5), Inches.of(20)),
      new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0))
    );

  private static final Transform3d armAttachOffset =
    new Transform3d(
      new Translation3d(Inches.of(0), Inches.of(0), Inches.of(0)),
      new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0))
    );

  private static final Transform3d armPivotOffset =
    new Transform3d(
      new Translation3d(Inches.of(-1), Inches.of(-0.5), Inches.of(20)),
      new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0))
    );

  private static final Transform3d climberAttachOffset =
    new Transform3d(
      new Translation3d(Inches.of(-1), Inches.of(-0.5), Inches.of(20)),
      new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0))
    );
  
    private static final Transform3d climberPivotOffset =
    new Transform3d(
      new Translation3d(Inches.of(-1), Inches.of(-0.5), Inches.of(20)),
      new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0))
    );
}
