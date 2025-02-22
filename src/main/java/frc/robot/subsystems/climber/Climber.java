package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private ClimberIO climberIO;
  ClimberInputsAutoLogged loggedClimber = new ClimberInputsAutoLogged();

  // Create a new instance of the Climber subsystem
  // Grabs the IO layer for the Climber subsystem, could be a simulation or real hardware
  public Climber(ClimberIO io) {
    climberIO = io;
    loggedClimber.climberAngle = 0;
    loggedClimber.climberAngularVelocity = DegreesPerSecond.mutable(0);
    loggedClimber.climberSetpoint = 0;
    loggedClimber.supplyCurrent = Amps.mutable(0);
    loggedClimber.torqueCurrent = Amps.mutable(0);
    loggedClimber.voltageSetPoint = Volts.mutable(0);
    
    // Set the climber source in the visualizer
    //RobotState.getInstance().setClimberTwist(loggedClimber.climberAngle);
  }

  // Set the angle of the climber
  public void setAngle(double climbAngle) {
    climberIO.setClimberTarget(climbAngle);
    System.out.println("Setting Climber Target");
  }

  // Create a new command to set the angle of the climber
  public Command getNewPivotTurnCommand(double a) {
    return new InstantCommand(
        () -> {
          setAngle(a);
        },
        this);
  }

  // Set the voltage of the climber - test command (will probably be used at Sussex)
  public Command setVoltageTest(double voltage) {
    return new InstantCommand(
        () -> {
          climberIO.setVoltage(voltage);
        },
        this);
  }

  // Called periodically to update the Climber subsystem with the new inputs and log them
  @Override
  public void periodic() {
    climberIO.updateInputs(loggedClimber);
    Logger.processInputs("RobotState/Climber", loggedClimber);
    SmartDashboard.putNumber("Climber Angle", loggedClimber.climberAngle);
  }
}
