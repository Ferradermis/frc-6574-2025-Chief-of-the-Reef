package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private ArmIO armIO;
    private ArmConstants armConstants;
    ArmInputsAutoLogged loggedArm = new ArmInputsAutoLogged();

    // Create a new instance of the Arm subsystem
    // Grabs the IO layer for the Arm subsystem, could be a simulation or real hardware
    public Arm(ArmIO io) {
        armIO = io;
        loggedArm.angle = Degrees.mutable(0);
        loggedArm.angularVelocity = DegreesPerSecond.mutable(0);
        loggedArm.setpoint = Degrees.mutable(0);
        loggedArm.supplyCurrent = Amps.mutable(0);
        loggedArm.torqueCurrent = Amps.mutable(0);
        loggedArm.voltageSetpoint = Volts.mutable(0);

        armConstants = armIO.getConstants();
        // Set the arm source in the visualizer
        armConstants.mechanismSimCallback.accept(loggedArm.angle);
    }

    // Set the angle of the arm
    public void setAngle(Angle angle) {
        armIO.setTarget(angle);
    }

    // Create a new command to set the angle of the arm
    public Command getNewSetAngleCommand(double i) {
        return new InstantCommand(
            () -> {
                setAngle(Degrees.of(i));
            },
            this);
    }

    // Called periodically to update the Arm subsystem with the new inputs and log them
    @Override
    public void periodic() {
        armIO.updateInputs(loggedArm);
        Logger.processInputs("RobotState/" + armConstants.loggedName, loggedArm);
    }
}
