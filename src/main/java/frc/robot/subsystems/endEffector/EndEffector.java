package frc.robot.subsystems.endEffector;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  private EndEffectorIO endEffectorIO;
  EndEffectorInputsAutoLogged loggedEndEffector = new EndEffectorInputsAutoLogged();

  // Create a new instance of the EndEffector subsystem
  // Grabs the IO layer for the EndEffector subsystem, could be a simulation or real hardware
  public EndEffector(EndEffectorIO io) {
    endEffectorIO = io;
    loggedEndEffector.angularVelocity = DegreesPerSecond.mutable(0);
    loggedEndEffector.supplyCurrent = Amps.mutable(0);
    loggedEndEffector.statorCurrent = Amps.mutable(0);
    loggedEndEffector.voltage = Volts.mutable(0);
    loggedEndEffector.voltageSetpoint = Volts.mutable(0);
  }

  // Set the target voltage of the EndEffector subsystem
  public void setTarget(Voltage setpoint) {
    endEffectorIO.setTarget(setpoint);
  }

  // Create a new command to set the target voltage of the EndEffector subsystem
  public Command getNewSetVoltsCommand(double i) {
    return new InstantCommand(
        () -> {
          setTarget(Volts.of(i));
        },
        this);
  }

  // Called periodically to update the EndEffector subsystem with the new inputs and log them
  @Override
  public void periodic() {
    endEffectorIO.updateInputs(loggedEndEffector);
    Logger.processInputs("RobotState/Intake", loggedEndEffector);
  }
}
