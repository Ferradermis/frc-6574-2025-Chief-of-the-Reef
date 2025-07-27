package frc.robot.subsystems.climberGate;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class ClimberGate extends SubsystemBase {
  private ClimberGateIO climberIO;
  ClimberGateInputsAutoLogged loggedClimber = new ClimberGateInputsAutoLogged();

  // Create a new instance of the Climber subsystem
  // Grabs the IO layer for the Climber subsystem, could be a simulation or real hardware
  public ClimberGate(ClimberGateIO io) {
    climberIO = io;
    loggedClimber.angle = 0;
    loggedClimber.angularVelocity = DegreesPerSecond.mutable(0);
    loggedClimber.setpoint = 0;
    loggedClimber.supplyCurrent = Amps.mutable(0);
    loggedClimber.torqueCurrent = Amps.mutable(0);
    loggedClimber.voltageSetpoint = Volts.mutable(0);
    
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
    Logger.processInputs("RobotState/ClimberGate", loggedClimber);
    SmartDashboard.putNumber("Climber Angle", loggedClimber.angle);
    if (loggedClimber.angle >= 1.6) {
      SmartDashboard.putBoolean("Climber Gate Fully Closed", true);
    }
    else if (loggedClimber.angle <= 1.6) {
      SmartDashboard.putBoolean("Climber Gate Fully Closed", false);
    }
  }
}
