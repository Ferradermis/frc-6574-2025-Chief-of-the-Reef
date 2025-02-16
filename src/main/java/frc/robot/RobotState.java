package frc.robot;

import static edu.wpi.first.units.Units.*;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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
  private MutAngle armAngle = Degrees.mutable(90);
  private MutAngle climberAngle = Degrees.mutable(0);
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

  // Create and assemble the 2d visualization of the robot
  private RobotState(String k) {
    key = k;

    // Create the 2d ligaments for each section of the robot
    primaryMechanism2d = new Mechanism2d(500, 500);
    elevatorLigament2d = new MechanismLigament2d("ElevatorLigament", elevatorPosition.in(Centimeters), 90, 10, new Color8Bit(255, 0, 255));
    armLigament2d = new MechanismLigament2d("ArmLigament", Centimeters.convertFrom(10, Inches), armAngle.in(Degrees), 10, new Color8Bit(0, 255, 0));
    climberLigament2d = new MechanismLigament2d("ClimberLigament", Centimeters.convertFrom(10, Inches), climberAngle.in(Degrees), 10, new Color8Bit(0, 225, 255));
    climberMount2d = new MechanismLigament2d("ClimberMount2d", Centimeters.convertFrom(5, Inches), 90, 10, new Color8Bit(255, 255, 255));

    // Create the primary mechanism root and append the ligaments
    // Elevator attaches to the primary mechanism root, arm attaches to the elevator
    primaryMechanismRoot2d = primaryMechanism2d.getRoot("Primary2d", 150, 13);
    primaryMechanismRoot2d.append(elevatorLigament2d);
    elevatorLigament2d.append(armLigament2d);

    // Create the climber mount root and append the ligaments
    // Climber mount attaches to the primary mechanism root, climber attaches to the climber mount
    climberMountRoot2d = primaryMechanism2d.getRoot("ClimberMount2d", 105, 10);
    climberMountRoot2d.append(climberMount2d);
    climberMount2d.append(climberLigament2d);

    // Create the robot base root and append the ligaments
    // Robot base attaches to the primary mechanism root
    robotBaseRoot = primaryMechanism2d.getRoot("RobotBase2d", 100, 5);
    robotBaseRoot.append(robotBaseLigament2d);

    // Add the primary mechanism to the SmartDashboard
    SmartDashboard.putData(key, primaryMechanism2d);
  }

  // If the instance is null, create a new instance
  // If an instance already exists, return the instance
  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState("RobotState");
    }
    return instance;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void periodic() {
    visualize();
  }

  // Get the current position of the elevator
  public Distance getElevatorPosition() {
    return elevatorPosition;
  }

  // Get the current angle of the arm
  public Angle getArmAngle() {
    return armAngle;
  }

  // Get the current angle of the climber
  public Angle getClimberAngle() {
    return climberAngle;
  }

  // Get the current angle of the rotate
  public Angle getRotateTwist() {
    return rotateTwist;
  }

  // Set the position of the elevator
  public void setElevatorPosition(Distance position) {
    elevatorPosition.mut_replace(position);
  }

  // Set the angle of the arm
  public void setArmAngle(Angle angle) {
    armAngle.mut_replace(angle);
  }

  // Set the angle of the rotate
  public void setRotateTwist(Angle twist) {
    rotateTwist.mut_replace(twist);
  }

  // Set the angle of the climber
  public void setClimberTwist(Angle angle) {
    climberAngle.mut_replace(angle);
  }

  // Set the initial position of the elevator
  public void setElevatorSource(MutDistance elevatorposition) {
    elevatorPosition = elevatorposition;
  }

  // Set the initial angle of the arm
  public void setArmSource(MutAngle angle) {
    armAngle = angle;
  }

  // Set the initial angle of the rotate
  public void setRotateSource(MutAngle rotatetwist) {
    rotateTwist = rotatetwist;
  }

  // Set the initial angle of the climber
  public void setClimberSource(MutAngle angle) {
    climberAngle = angle;
  }

  // Create and assemble the 3d visualization of the robot
  private void visualize() {
    // Creates a new pose3d to change the height of the elevator in the 3d visualizer
    Pose3d elevatorPose =
      new Pose3d(elevatorAttachOffset.getTranslation(), elevatorAttachOffset.getRotation())
      .transformBy(
        new Transform3d(
          new Translation3d(
            Meters.zero(), Meters.zero(), elevatorPosition), 
            new Rotation3d())
    );

    // Creates a new pose3d to change the angle of the arm in the 3d visualizer
    Pose3d armPose =
    elevatorPose
      .transformBy(armAttachOffset)
      .transformBy(
        new Transform3d(
          new Translation3d(), 
            new Rotation3d(
              Degrees.zero(), armAngle, rotateTwist)))
      .transformBy(armPivotOffset.inverse()
      );
    
    // Creates a new pose3d to change the angle of the climber in the 3d visualizer
    Pose3d climberPose =
    new Pose3d(climberAttachOffset.getTranslation(), climberAttachOffset.getRotation())
      .transformBy(climberAttachOffset)
      .transformBy(
        new Transform3d(
          new Translation3d(), 
            new Rotation3d(
              Degrees.zero(), climberAngle, Degrees.zero())))
      .transformBy(climberPivotOffset.inverse()
      ); 

    // Set the length of the elevator, angle of the arm, and angle of the climber in the 2d visualizer  
    elevatorLigament2d.setLength(elevatorPosition.in(Centimeters));
    armLigament2d.setAngle(armAngle.in(Degrees) + 180);
    climberLigament2d.setAngle(climberAngle.in(Degrees));

    // Records the current pose of the elevator, arm, and climber in the 3d visualizer
    Logger.recordOutput("RobotState/Elevator/" + key, elevatorPose);
    Logger.recordOutput("RobotState/Arm/" + key, armPose);
    Logger.recordOutput("RobotState/Climber/" + key, climberPose);
  }
  
  // Set the initial position of the elevator in the 3d visualizer
  private static final Transform3d elevatorAttachOffset =
    new Transform3d(
      new Translation3d(Inches.of(-1), Inches.of(-0.5), Inches.of(20)),
      new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0))
    );

  // Set the initial position of the arm attach point in the 3d visualizer
  private static final Transform3d armAttachOffset =
    new Transform3d(
      new Translation3d(Inches.of(0), Inches.of(0), Inches.of(0)),
      new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0))
    );

  // Set the initial position of the arm pivot point in the 3d visualizer
  private static final Transform3d armPivotOffset =
    new Transform3d(
      new Translation3d(Inches.of(-1), Inches.of(-0.5), Inches.of(20)),
      new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0))
    );

  // Set the initial position of the climber attach point in the 3d visualizer
  private static final Transform3d climberAttachOffset =
    new Transform3d(
      new Translation3d(Inches.of(-10), Inches.of(0), Inches.of(0)),
      new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0))
    );
  
  // Set the initial position of the climber pivot point in the 3d visualizer
  private static final Transform3d climberPivotOffset =
    new Transform3d(
      new Translation3d(Inches.of(-10), Inches.of(0), Inches.of(0)),
      new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0))
    );
}
