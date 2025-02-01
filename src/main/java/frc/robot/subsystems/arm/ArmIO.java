package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;

public interface ArmIO {
    
    @AutoLog
    public static class ArmInputs {
        public MutAngle angle;
        public MutAngularVelocity angularVelocity;
        public MutAngle setpoint;
        public MutVoltage voltageSetpoint;
        public MutCurrent supplyCurrent;
        public MutCurrent torqueCurrent;
    }

    public void setTarget(Angle target);

    public void updateInputs(ArmInputs inputs);

    public void stop();

    public ArmConstants getConstants();
}
