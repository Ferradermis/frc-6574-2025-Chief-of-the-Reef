package frc.robot.subsystems.rotate;

import org.littletonrobotics.junction.Logger;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rotate extends SubsystemBase{
    private RotateIO rotateIO;
    private RotateConstants rotateConstants;
    RotateInputsAutoLogged loggedRotate = new RotateInputsAutoLogged();

    // Create a new instance of the Rotate subsystem
    // Grabs the IO layer for the Rotate subsystem, could be a simulation or real hardware
    public Rotate(RotateIO io) {
        rotateIO = io;
        loggedRotate.angle = Degrees.mutable(0);
        loggedRotate.angularVelocity = DegreesPerSecond.mutable(0);
        loggedRotate.setpoint = Degrees.mutable(0);
        loggedRotate.supplyCurrent = Amps.mutable(0);
        loggedRotate.torqueCurrent = Amps.mutable(0);
        loggedRotate.voltageSetpoint = Volts.mutable(0);

        rotateConstants = rotateIO.getConstants();
        rotateConstants.mechanismSimCallback.accept(loggedRotate.angle);
    }

    // Set the angle of the Rotate subsystem
    public void setAngle(Angle angle) {
        rotateIO.setTarget(angle);
    }

    // Create a new command to set the angle of the Rotate subsystem
    public Command getNewSetAngleCommand(double i) {
        return new InstantCommand(
            () -> {
                setAngle(Degrees.of(i));
            },
            this);
    }

    public Command setVoltageTest(double voltage) {
        return new InstantCommand(
            () -> {
              rotateIO.setVoltage(voltage);
            },
            this);
      }

    // Called periodically to update the Rotate subsystem with the new inputs and log them
    @Override
    public void periodic() {
        rotateIO.updateInputs(loggedRotate);
        Logger.processInputs("RobotState/" + rotateConstants.loggedName, loggedRotate);
    }
}
