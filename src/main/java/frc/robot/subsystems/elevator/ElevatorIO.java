package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  class ElevatorIOInputs {
    public MutDistance distance;
    public MutLinearVelocity velocity;
    public MutDistance setpoint;
    public MutVoltage voltageSetpoint;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
  }

  public void updateInputs(ElevatorIOInputs inputs);

  public void setTarget(Distance target);

  public void stop();
}
