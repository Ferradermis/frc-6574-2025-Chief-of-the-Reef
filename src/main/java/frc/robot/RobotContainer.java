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
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.PositionConstants;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Intake;
import frc.robot.commands.Release;
import frc.robot.commands.SelectorCommandFactory;
import frc.robot.commands.LowerClimber;
import frc.robot.commands.RaiseClimber;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetPivotAngle;
import frc.robot.commands.SetTurretAngle;
import frc.robot.commands.FullAutoSystemCommands.GrabAlgaeInAuto;
import frc.robot.commands.FullAutoSystemCommands.ReleaseAlgaeInAuto;
import frc.robot.commands.FullAutoSystemCommands.ReleaseInAuto;
import frc.robot.commands.FullAutoSystemCommands.ScoreL1InAuto;
import frc.robot.commands.FullAutoSystemCommands.ScoreL3InAuto;
import frc.robot.commands.FullAutoSystemCommands.TomfooleryInAuto;
import frc.robot.commands.FullTeleopSystemCommands.AlgaeReturnToHome;
import frc.robot.commands.FullTeleopSystemCommands.AlignToReef;
import frc.robot.commands.FullTeleopSystemCommands.Climb;
import frc.robot.commands.FullTeleopSystemCommands.GrabAlgaeOne;
import frc.robot.commands.FullTeleopSystemCommands.GrabAlgaeTwo;
import frc.robot.commands.FullTeleopSystemCommands.PickupAlgaeFromGround;
import frc.robot.commands.FullTeleopSystemCommands.PickupCoralFromChute;
import frc.robot.commands.FullTeleopSystemCommands.PickupCoralFromGround;
import frc.robot.commands.FullTeleopSystemCommands.ReturnToHome;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelFour;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelOne;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelThree;
import frc.robot.commands.FullTeleopSystemCommands.ScoreLevelTwo;
import frc.robot.commands.FullTeleopSystemCommands.ScoreProcessor;
import frc.robot.commands.FullTeleopSystemCommands.Tomfoolery;
import frc.robot.commands.FullTeleopSystemCommands.AlignToReef.ReefPosition;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LockingServo;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIONEO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climberGate.ClimberGate;
import frc.robot.subsystems.climberGate.ClimberGateIO;
import frc.robot.subsystems.climberGate.ClimberGateIOKraken;
import frc.robot.subsystems.climberGate.ClimberGateIOSim;
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
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.ReefPositionsUtil;
import frc.robot.util.ReefPositionsUtil.AutoAlignSide;

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
  public static AprilTagVision vision;
  public static Drive drive;
  public static Elevator elevator;
  public static Pivot pivot;
  public static Climber climber;
  public static ClimberGate climberGate;
  public static EndEffector endEffector;
  public static Turret turret;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private ReefPositionsUtil reefPositions;
  private boolean teleopInitialized = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations and start camera feed
        // UsbCamera camera = CameraServer.startAutomaticCapture();
        // camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
        // camera.setVideoMode(PixelFormat.kYUYV, 320, 240, 30);
        // camera.setBrightness(50);
        elevator = new Elevator(new ElevatorIOKraken(Constants.CANConstants.ELEVATOR_LEFT_ID, Constants.CANConstants.ELEVATOR_RIGHT_ID));
        pivot = new Pivot(new PivotIOKraken(Constants.CANConstants.PIVOT_ID));
        climber = new Climber(new ClimberIONEO(Constants.CANConstants.CLIMBER_ID));
        climberGate = new ClimberGate(new ClimberGateIOKraken(22));
        endEffector = new EndEffector(new EndEffectorIOKraken(Constants.CANConstants.END_EFFECTOR_ID));
        turret = new Turret(new TurretIOKraken(Constants.CANConstants.TURRET_ID));
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new AprilTagVision(
              drive::setPose, 
              drive::addVisionMeasurement, 
              drive::addVisionMeasurementAutoAlign, 
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
        climberGate = new  ClimberGate(new ClimberGateIOSim());
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision = 
            new AprilTagVision(
              drive::setPose, 
              drive::addVisionMeasurement, 
              drive::addVisionMeasurementAutoAlign, 
              new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose));
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
        //         new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        break;

      default:
      // Replayed robot, disable IO implementations
        elevator = new Elevator(new ElevatorIO() {});
        pivot = new Pivot(new PivotIO() {});
        climber = new Climber(new ClimberIO() {});
        endEffector = new EndEffector(new EndEffectorIO() {});
        turret = new Turret(new TurretIO() {});
        climberGate = new ClimberGate(new ClimberGateIO() {});
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        //vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    NamedCommands.registerCommand("Release", new ReleaseInAuto());
    NamedCommands.registerCommand("ScoreLevelOne", new ScoreL1InAuto());
    NamedCommands.registerCommand("ScoreLevelThree", new ScoreL3InAuto());
    NamedCommands.registerCommand("ReturnToHome", new ReturnToHome());
    NamedCommands.registerCommand("ScoreBarge", new TomfooleryInAuto());
    NamedCommands.registerCommand("GrabAlgae", new GrabAlgaeInAuto());
    NamedCommands.registerCommand("ReleaseAlage", new ReleaseAlgaeInAuto());

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

    reefPositions = ReefPositionsUtil.getInstance();
    AlignToReef.initialize();
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

    // driverController.a().whileTrue(
    //     DriveCommands.joystickDriveAtAngle(
    //         drive, 
    //         () -> -driverController.getLeftY(),
    //         () -> -driverController.getLeftX(),
    //         () -> vision.getTargetX(0)));

    // driverController.b().whileTrue(
    //     DriveCommands.joystickDrive(
    //         drive, 
    //         () -> -driverController.getLeftY(),
    //         () -> -driverController.getLeftX(),
    //         () -> -driverController.getRightX()));

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

    // Driver buttons
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

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
    driverController.rightBumper().whileTrue(new Intake(13)).whileFalse(new Intake(0));
    driverController.leftBumper().whileTrue(new Release(10)).whileFalse(new Intake(0));
    driverController.povRight().onTrue(climberGate.getNewPivotTurnCommand(1.9));
    driverController.povLeft().onTrue(climberGate.getNewPivotTurnCommand(0));
    //driverController.b().onTrue(climberGate.getNewPivotTurnCommand(0));
    //driverController.povDown().whileTrue(climber.setVoltageTest(-4)).onFalse(climber.setVoltageTest(0)); //up
    driverController.povDown().onTrue(new Climb());
    // driverController.povLeft()
    // .and(() -> reefPositions.getIsAutoAligning())
    // .and(() -> {return reefPositions.getAutoAlignSide() == AutoAlignSide.Left;})
    // .whileTrue(
    //   AlignToReef.getNewReefCoralScoreSequence(
    //     ReefPosition.Left, 
    //     true, 
    //     SelectorCommandFactory.getCoralLevelPrepCommandSelector(), 
    //     SelectorCommandFactory.getCoralLevelScoreCommandSelector(), 
    //     SelectorCommandFactory.getCoralLevelStopScoreCommandSelector(),
    //      drive)
    // ).onFalse(
    //   new ReturnToHome());
    //driverController.povLeft().whileTrue(new AutoAlign(AlignToReef.getGetTargetPositionFunction(ReefPosition.Left, false), drive));

    // Operator buttons
    operatorController.a().onTrue(new ScoreLevelOne());
    operatorController.b().onTrue(new ScoreLevelTwo());
    operatorController.x().onTrue(new ScoreLevelThree());
    operatorController.y().onTrue(new ScoreLevelFour());
    operatorController.povDown().onTrue(new GrabAlgaeOne());
    operatorController.povUp().onTrue(new GrabAlgaeTwo());
    operatorController.povLeft().onTrue(new Tomfoolery());
    operatorController.povRight().onTrue(new ScoreProcessor());
    operatorController.rightBumper().onTrue(new PickupCoralFromChute());
    operatorController.leftBumper().onTrue(new AlgaeReturnToHome());
    //operatorController.rightTrigger().whileTrue(climber.setVoltageTest(6)).onFalse(climber.setVoltageTest(0)); //down
    operatorController.rightTrigger().onTrue(new LowerClimber());
    operatorController.leftTrigger().onTrue(new SetPivotAngle(Constants.PositionConstants.PIVOT_LOWER_ANGLE));
    //operatorController.leftTrigger().whileTrue(climber.setVoltageTest(-4)).onFalse(climber.setVoltageTest(0)); //up
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void teleopInit() {
    if (!this.teleopInitialized) {
      vision.setRobotPositionBasedOnAlliance();
      vision.enableUpdateOdometryBasedOnApriltags();
      teleopInitialized = true;
    }
  }
}
