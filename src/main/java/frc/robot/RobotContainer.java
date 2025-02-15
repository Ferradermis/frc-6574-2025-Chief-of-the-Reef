// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Intake;
import frc.robot.commands.Release;
import frc.robot.commands.FullTeleopSystemCommands.GrabAlgae;
import frc.robot.commands.FullTeleopSystemCommands.PickupAlgaeFromGround;
import frc.robot.commands.FullTeleopSystemCommands.PickupCoralFromChute;
import frc.robot.commands.FullTeleopSystemCommands.PickupCoralFromGround;
import frc.robot.commands.FullTeleopSystemCommands.ReturnToHome;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelFour;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelOne;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelThree;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelTwo;
import frc.robot.commands.FullTeleopSystemCommands.ScoreProcessor;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIONEO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOMotors;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOKraken;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffectorIOKraken;
import frc.robot.subsystems.endEffector.EndEffectorIOSim;
import frc.robot.subsystems.rotate.Rotate;
import frc.robot.subsystems.rotate.RotateConstants;
import frc.robot.subsystems.rotate.RotateIONEO;
import frc.robot.subsystems.rotate.RotateIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

//TODO: go back and actually document all classes in the project (putting this here cuz we'll see this the most)
//note: copilot needs to stop insulting me /j
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public static Vision vision;
  public static Drive drive;
  public static Elevator elevator;
  public static Arm arm;
  public static Climber climber;
  public static EndEffector endEffector;
  public static Rotate rotate;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  final LoggedTunableNumber setElevator = new LoggedTunableNumber("RobotState/Elevator/setElevator", 30);
  final LoggedTunableNumber returnToZero = new LoggedTunableNumber("RobotState/Elevator/returnToZero", 10);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        elevator = new Elevator(new ElevatorIOKraken(16, 17));
        arm = new Arm(new ArmIONEO(18));
        climber = new Climber(new ClimberIOMotors(15));
        endEffector = new EndEffector(new EndEffectorIOKraken(20));
        rotate = new Rotate(new RotateIONEO(19));
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));
        // vision =
        //     new Vision(
        //         demoDrive::addVisionMeasurement,
        //         new VisionIOPhotonVision(camera0Name, robotToCamera0),
        
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        elevator = new Elevator(new ElevatorIOSim());
        arm = new Arm(new ArmIOSim(new ArmConstants()));
        climber = new Climber(new ClimberIOSim());
        endEffector = new EndEffector(new EndEffectorIOSim());
        rotate = new Rotate(new RotateIOSim(new RotateConstants()));
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        break;

      default:
      // Replayed robot, disable IO implementations
        elevator = new Elevator(new ElevatorIOSim());
        arm = new Arm(new ArmIOSim(new ArmConstants()));
        climber = new Climber(new ClimberIOSim());
        endEffector = new EndEffector(new EndEffectorIOSim());
        rotate = new Rotate(new RotateIOSim(new RotateConstants()));
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() { // TODO: create commands and button bindings (last thing before code is mostly finished :D )

    // Driver buttons
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Lock to 0° when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when Y button is pressed
    driverController
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    driverController.x().onTrue(new ReturnToHome());
    driverController.rightBumper().whileTrue(new Intake());
    driverController.leftBumper().whileTrue(new Release());

    // Operator buttons
    operatorController.a().onTrue(new ScoreLevelOne());
    operatorController.b().onTrue(new ScoreLevelTwo());
    operatorController.x().onTrue(new ScoreLevelThree());
    operatorController.y().onTrue(new ScoreLevelFour());
    operatorController.povUp().onTrue(new PickupAlgaeFromGround()); // Could be moved to driver instead (with an auto intake added)
    operatorController.povDown().onTrue(new ScoreProcessor());
    operatorController.povLeft().onTrue(new PickupCoralFromGround()); // Could be moved to driver instead (with an auto intake added)
    operatorController.povRight().onTrue(new PickupCoralFromChute());
    operatorController.rightBumper().onTrue(new GrabAlgae());
    operatorController.leftBumper().onTrue(new ReturnToHome());

    // Test buttons
    // driverController.povUp().onTrue(elevator.getNewSetDistanceCommand(setElevator));//.onFalse(elevator.getNewSetDistanceCommand(returnToZero));
    // driverController.povDown().onTrue(elevator.getNewSetDistanceCommand(returnToZero));
    //driverController.povUp().onTrue(climber.getNewPivotTurnCommand(57)).onFalse(climber.getNewPivotTurnCommand(90));
    driverController.povLeft().onTrue(arm.getNewSetAngleCommand(57));//.onFalse(arm.getNewSetAngleCommand(0));
    driverController.povRight().onTrue(arm.getNewSetAngleCommand(180));
    // driverController.rightBumper().onTrue(climber.getNewPivotTurnCommand(Degrees.of(37)));
    // driverController.leftBumper().onTrue(climber.getNewPivotTurnCommand(Degrees.of(90)));

    driverController.rightBumper().onTrue(new RunCommand(() -> climber.setVoltageTest(4), climber)).onFalse(new RunCommand(() -> climber.setVoltageTest(0), climber));

    // driverController.povUp().whileTrue(climber.setVoltageTest(4)).onFalse(climber.setVoltageTest(0));
    // driverController.povDown().whileTrue(climber.setVoltageTest(-4)).onFalse(climber.setVoltageTest(0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
