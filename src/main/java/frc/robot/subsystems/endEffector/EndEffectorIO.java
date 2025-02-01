package frc.robot.subsystems.endEffector;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

  @AutoLog
  public static class EndEffectorInputs {
    public MutAngularVelocity angularVelocity;
    public MutVoltage voltageSetpoint;
    public MutVoltage voltage;
    public MutCurrent supplyCurrent;
    public MutCurrent statorCurrent;
  }

  public void setTarget(Voltage setpoint);

  public void updateInputs(EndEffectorInputs inputs);

  public void stop();
}
