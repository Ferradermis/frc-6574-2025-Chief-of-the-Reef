package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.*;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    private PivotIO pivotIO;
    private PivotConstants pivotConstants;
    PivotInputsAutoLogged loggedPivot = new PivotInputsAutoLogged();

    // Create a new instance of the pivot subsystem
    // Grabs the IO layer for the pivot subsystem, could be a simulation or real hardware
    public Pivot(PivotIO io) {
        pivotIO = io;
        loggedPivot.angle = Degrees.mutable(0);
        loggedPivot.angularVelocity = DegreesPerSecond.mutable(0);
        loggedPivot.setpoint = 0;
        loggedPivot.supplyCurrent = Amps.mutable(0);
        loggedPivot.torqueCurrent = Amps.mutable(0);
        loggedPivot.voltageSetpoint = Volts.mutable(0);

        pivotConstants = pivotIO.getConstants();
        // Set the pivot source in the visualizer
        pivotConstants.mechanismSimCallback.accept(loggedPivot.angle);
    }

    // Set the angle of the pivot
    public void setAngle(double angle) {
        pivotIO.setTarget(angle);
    }

    // Create a new command to set the angle of the pivot
    public Command getNewSetAngleCommand(double i) {
        return new InstantCommand(
            () -> {
                setAngle(i);
            },
            this);
    }

    public Command setVoltageTest(double voltage) {
        return new InstantCommand(
            () -> {
              pivotIO.setVoltage(voltage);
            },
            this);
      }

    // Called periodically to update the pivot subsystem with the new inputs and log them
    @Override
    public void periodic() {
        pivotIO.updateInputs(loggedPivot);
        Logger.processInputs("RobotState/" + pivotConstants.loggedName, loggedPivot);
    }
}
