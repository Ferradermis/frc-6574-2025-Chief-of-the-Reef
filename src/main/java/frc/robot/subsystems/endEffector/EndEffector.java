package frc.robot.subsystems.endEffector;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  private EndEffectorIO endEffectorIO;

  EndEffectorInputsAutoLogged loggedEndEffector = new EndEffectorInputsAutoLogged();

  public EndEffector(EndEffectorIO io) {
    endEffectorIO = io;
    loggedEndEffector.angularVelocity = DegreesPerSecond.mutable(0);
    loggedEndEffector.supplyCurrent = Amps.mutable(0);
    loggedEndEffector.statorCurrent = Amps.mutable(0);
    loggedEndEffector.voltage = Volts.mutable(0);
    loggedEndEffector.voltageSetpoint = Volts.mutable(0);
  }

  public void setTarget(Voltage setpoint) {
    endEffectorIO.setTarget(setpoint);
  }

  public Command getNewSetVoltsCommand(double i) {
    return new InstantCommand(
        () -> {
          setTarget(Volts.of(i));
        },
        this);
  }

  @Override
  public void periodic() {
    endEffectorIO.updateInputs(loggedEndEffector);
    Logger.processInputs("RobotState/Intake", loggedEndEffector);
  }
}
