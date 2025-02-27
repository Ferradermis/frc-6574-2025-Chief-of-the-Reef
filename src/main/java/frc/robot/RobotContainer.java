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

import static edu.wpi.first.units.Units.*;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Intake;
import frc.robot.commands.Release;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetTurretAngle;
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
import frc.robot.subsystems.LockingServo;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOMotors;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOKraken;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffectorIO;
import frc.robot.subsystems.endEffector.EndEffectorIOKraken;
import frc.robot.subsystems.endEffector.EndEffectorIOSim;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOKraken;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOKraken;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

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
  public static Pivot pivot;
  public static Climber climber;
  public static EndEffector endEffector;
  public static Turret turret;
  public static LockingServo pinServo;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        elevator = new Elevator(new ElevatorIOKraken(Constants.CANConstants.ELEVATOR_LEFT_ID, Constants.CANConstants.ELEVATOR_RIGHT_ID));
        pivot = new Pivot(new PivotIOKraken(Constants.CANConstants.PIVOT_ID));
        climber = new Climber(new ClimberIOMotors(Constants.CANConstants.CLIMBER_ID)); 
        endEffector = new EndEffector(new EndEffectorIOKraken(Constants.CANConstants.END_EFFECTOR_ID));
        turret = new Turret(new TurretIOKraken(Constants.CANConstants.TURRET_ID));
        pinServo = new LockingServo(Constants.CANConstants.LOCKING_SERVO_ID);
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
        pivot = new Pivot(new PivotIOSim(new PivotConstants()));
        climber = new Climber(new ClimberIOSim());
        endEffector = new EndEffector(new EndEffectorIOSim());
        turret = new Turret(new TurretIOSim(new TurretConstants()));
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
        elevator = new Elevator(new ElevatorIO() {});
        pivot = new Pivot(new PivotIO() {});
        climber = new Climber(new ClimberIO() {});
        endEffector = new EndEffector(new EndEffectorIO() {});
        turret = new Turret(new TurretIO() {});
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

    // // Lock to 0° when A button is held
    // driverController
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -driverController.getLeftY(),
    //             () -> -driverController.getLeftX(),
    //             () -> new Rotation2d()));

    // // Switch to X pattern when X button is pressed
    // driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when Y button is pressed
    driverController
        .povUp()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // driverController.x().onTrue(new ReturnToHome()); - not used currently
    driverController.leftBumper().whileTrue(endEffector.getNewSetVoltsCommand(-8)).whileFalse(endEffector.getNewSetVoltsCommand(0));
    driverController.rightBumper().whileTrue(endEffector.getNewSetVoltsCommand(3.5)).whileFalse(endEffector.getNewSetVoltsCommand(0));
    driverController.y().onTrue(elevator.getNewSetDistanceCommand(1208.659)); // 30.7
    driverController.x().onTrue(elevator.getNewSetDistanceCommand(0)); //* 39.37
    driverController.b().onTrue(elevator.getNewSetDistanceCommand(15 * 39.37));
    driverController.a().onTrue(elevator.getNewSetDistanceCommand(4.5 * 39.37));
    driverController.povDown().onTrue(elevator.resetEncoder());
    driverController.povLeft().onTrue(turret.reset());

    // Operator buttons
    // operatorController.a().onTrue(new ScoreLevelOne());
    // operatorController.b().onTrue(new ScoreLevelTwo());
    // operatorController.x().onTrue(new ScoreLevelThree());
    // operatorController.y().onTrue(new ScoreLevelFour());
    // operatorController.povUp().onTrue(new PickupAlgaeFromGround()); // Could be moved to driver instead (with an auto intake added)
    // operatorController.povDown().onTrue(new ScoreProcessor());
    // operatorController.povLeft().onTrue(new PickupCoralFromGround()); // Could be moved to driver instead (with an auto intake added)
    // operatorController.povRight().onTrue(new PickupCoralFromChute());
    // operatorController.rightBumper().onTrue(new GrabAlgae());
    // operatorController.leftBumper().onTrue(new ReturnToHome());
    operatorController.y().onTrue(elevator.getNewSetDistanceCommand(1208.659)); // 30.7
    operatorController.x().onTrue(elevator.getNewSetDistanceCommand(0)); //* 39.37
    operatorController.b().onTrue(elevator.getNewSetDistanceCommand(15 * 39.37));
    operatorController.a().onTrue(elevator.getNewSetDistanceCommand(4.5 * 39.37)); // Add more elevator positions as needed for Duluth
    // operatorController.a().onTrue(elevator.getNewSetVoltageCommand(2)).onFalse(elevator.stopMotors());
    // operatorController.b().onTrue(elevator.getNewSetVoltageCommand(-2)).onFalse(elevator.stopMotors());
    operatorController.leftBumper().onTrue(elevator.resetEncoder());
    operatorController.rightBumper().onTrue(pinServo.getNewSetAngleCommand(-90));
    //operatorController.povLeft().whileTrue(turret.setVoltageTest(1)).whileFalse(turret.setVoltageTest(0)); // Rotate clockwise
    operatorController.povLeft().onTrue(turret.getNewSetAngleCommand(0));
    //operatorController.povRight().whileTrue(turret.setVoltageTest(-1)).whileFalse(turret.setVoltageTest(0)); // Rotate counterclockwise
    operatorController.povRight().onTrue(turret.getNewSetAngleCommand(2.225));
    operatorController.povUp().whileTrue(pivot.setVoltageTest(-1.5)).whileFalse(pivot.setVoltageTest(0)); // Pivot up
    operatorController.povDown().whileTrue(pivot.setVoltageTest(0.75)).whileFalse(pivot.setVoltageTest(0)); // Pivot down
    operatorController.rightTrigger().whileTrue(climber.setVoltageTest(2)).onFalse(climber.setVoltageTest(0));
    operatorController.leftTrigger().whileTrue(climber.setVoltageTest(-4)).onFalse(climber.setVoltageTest(0));
    // operatorController.a().onTrue(turret.getNewSetAngleCommand(1.846)).onFalse(turret.stopMotors());
    // operatorController.b().onTrue(turret.getNewSetAngleCommand(-0.480)).onFalse(turret.stopMotors());

    // Test buttons
    //driverController.povUp().onTrue(elevator.getNewSetDistanceCommand(0.1));
    //driverController.povDown().onTrue(elevator.getNewSetDistanceCommand(540.0)).onFalse(elevator.getNewSetDistanceCommand(40.0));
    // operatorController.povLeft().whileTrue(rotate.setVoltageTest(1)).whileFalse(rotate.setVoltageTest(0));//.onFalse(rotate.getNewSetAngleCommand(0));
    // operatorController.povRight().whileTrue(rotate.setVoltageTest(-1)).whileFalse(rotate.setVoltageTest(0));//.onFalse(rotate.getNewSetAngleCommand(0));
    // operatorController.povLeft().onTrue(rotate.getNewSetAngleCommand(1));
    //operatorController.povUp().onTrue(arm.getNewSetAngleCommand(-0.295)); //.onFalse(arm.getNewSetAngleCommand(-0.060));
    //operatorController.b().onTrue(elevator.resetEncoder());
    // operatorController.povUp().whileTrue(arm.setVoltageTest(2)).whileFalse(arm.setVoltageTest(0));//.onFalse(arm.getNewSetAngleCommand(0));
    // operatorController.povDown().whileTrue(arm.setVoltageTest(-1)).whileFalse(arm.setVoltageTest(0));//.onFalse(arm.getNewSetAngleCommand(0));
    // driverController.rightBumper().whileTrue(endEffector.getNewSetVoltsCommand(12)).whileFalse(endEffector.getNewSetVoltsCommand(0));
    // driverController.leftBumper().whileTrue(endEffector.getNewSetVoltsCommand(-12)).whileFalse(endEffector.getNewSetVoltsCommand(0));
    // // driverController.rightBumper().onTrue(climber.getNewPivotTurnCommand(Degrees.of(37)));
    // // driverController.leftBumper().onTrue(climber.getNewPivotTurnCommand(Degrees.of(90)));

    // //driverController.rightBumper().onTrue(new RunCommand(() -> climber.setVoltageTest(4), climber)).onFalse(new RunCommand(() -> climber.setVoltageTest(0), climber));
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
