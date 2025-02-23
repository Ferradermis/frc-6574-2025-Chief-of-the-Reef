package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;

// Interface for the Turret subsystem IO
// Instantiates the TurretInputs class with the necessary inputs for the different IO layers
public interface TurretIO {
    @AutoLog
    public static class TurretInputs {
        public double angle = 0;
        public MutAngularVelocity angularVelocity = DegreesPerSecond.mutable(0);
        public double setpoint = 0;
        public MutVoltage voltageSetpoint = Volts.mutable(0);
        public MutCurrent supplyCurrent = Amps.mutable(0);
        public MutCurrent torqueCurrent = Amps.mutable(0);
    }

    public default void setTarget(double target) {}

    public default void updateInputs(TurretInputs inputs) {}

    public default void stop() {}

    public default void setVoltage(double voltage) {}

    public default TurretConstants getConstants() {
        return new TurretConstants();
    }
}
