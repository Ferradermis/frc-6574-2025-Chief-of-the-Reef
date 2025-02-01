package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  /** The time in seconds from the FPGA start and the creation of this set of inputs */
  @AutoLog
  public static class ClimberInputs {

    public double timestamp;

    public MutAngle climberAngle;
    public MutAngularVelocity climberAngularVelocity;
    public MutAngle climberSetPoint;
    public MutVoltage voltageSetPoint;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
  }

  public void setClimberTarget(Angle target);

  public void updateInputs(ClimberInputs inputs);
}
