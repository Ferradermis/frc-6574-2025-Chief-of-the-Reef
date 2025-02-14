package frc.robot.subsystems.rotate;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;

// Interface for the Rotate subsystem IO
// Instantiates the RotateInputs class with the necessary inputs for the different IO layers
public interface RotateIO {
    @AutoLog
    public static class RotateInputs {
        public MutAngle angle;
        public MutAngularVelocity angularVelocity;
        public MutAngle setpoint;
        public MutVoltage voltageSetpoint;
        public MutCurrent supplyCurrent;
        public MutCurrent torqueCurrent;
    }

    public void setTarget(Angle target);

    public void updateInputs(RotateInputs inputs);

    public void stop();

    public RotateConstants getConstants();
}
