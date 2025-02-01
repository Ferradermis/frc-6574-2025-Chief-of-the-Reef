package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private ArmIO armIO;
    private ArmConstants armConstants;
    ArmInputsAutoLogged loggedArm = new ArmInputsAutoLogged();

    public Arm(ArmIO io) {
        armIO = io;
        loggedArm.angle = Degrees.mutable(0);
        loggedArm.angularVelocity = DegreesPerSecond.mutable(0);
        loggedArm.setpoint = Degrees.mutable(0);
        loggedArm.supplyCurrent = Amps.mutable(0);
        loggedArm.torqueCurrent = Amps.mutable(0);
        loggedArm.voltageSetpoint = Volts.mutable(0);

        armConstants = armIO.getConstants();
        armConstants.mechanismSimCallback.accept(loggedArm.angle);
    }

    public void setAngle(Angle angle) {
        armIO.setTarget(angle);
    }

    public Command getNewSetAngleCommand(double i) {
        return new InstantCommand(
            () -> {
                setAngle(Degrees.of(i));
            },
            this);
    }
}
