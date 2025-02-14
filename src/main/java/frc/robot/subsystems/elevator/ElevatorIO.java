package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import org.littletonrobotics.junction.AutoLog;

// Interface for the Elevator subsystem IO
// Instantiates the ElevatorInputs class with the necessary inputs for the different IO layers
public interface ElevatorIO {
  @AutoLog
  class ElevatorInputs {
    public MutDistance distance;
    public MutLinearVelocity velocity;
    public MutDistance setpoint;
    public MutVoltage voltageSetpoint;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
  }

  public void updateInputs(ElevatorInputs inputs);

  public void setTarget(Distance target);

  public void stop();
}
