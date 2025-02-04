package frc.robot.subsystems.rotate;

import org.littletonrobotics.junction.Logger;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.rotate.RotateIO.RotateInputs;

public class Rotate extends SubsystemBase{
    private RotateIO rotateIO;
    private RotateConstants rotateConstants;
    RotateInputsAutoLogged loggedRotate = new RotateInputsAutoLogged();

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

    public void setAngle(Angle angle) {
        rotateIO.setTarget(angle);
    }

    public Command getNewSetAngleCommand(double i) {
        return new InstantCommand(
            () -> {
                setAngle(Degrees.of(i));
            },
            this);
    }

    public Trigger getNewAngleTrigger(Angle angle, Angle tolerance) {
        return new Trigger(() -> {
            return MathUtil.isNear(angle.baseUnitMagnitude(), loggedRotate.angle.baseUnitMagnitude(), tolerance.baseUnitMagnitude());
        });
    }

    public Trigger getNewSetpointTrigger(Angle angle, Angle tolerance) {
        return new Trigger(() -> {
            return MathUtil.isNear(loggedRotate.setpoint.baseUnitMagnitude(), loggedRotate.angle.baseUnitMagnitude(), Degrees.of(0.25).baseUnitMagnitude());
        });
    }

    @Override
    public void periodic() {
        rotateIO.updateInputs(loggedRotate);
        Logger.processInputs("RobotState/" + rotateConstants.loggedName, loggedRotate);
    }
}
