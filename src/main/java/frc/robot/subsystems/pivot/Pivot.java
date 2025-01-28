package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class Pivot extends SubsystemBase {
    private PivotIO m_PivotIO;

    PivotInputsAutoLogged loggedPivot = new PivotInputsAutoLogged();

    public Pivot(PivotIO pivotIO) {
        m_PivotIO = pivotIO;
        loggedPivot.pivotAngle = Degrees.mutable(0);
        loggedPivot.pivotAngularVelocity = DegreesPerSecond.mutable(0);
        loggedPivot.pivotSetPoint = Degrees.mutable(0);
        loggedPivot.supplyCurrent = Amps.mutable(0);
        loggedPivot.torqueCurrent = Amps.mutable(0);
        loggedPivot.voltageSetPoint = Volts.mutable(0);

        RobotState.instance.setPivotSource(loggedPivot.pivotAngle);
    }

    public void setAngle(Angle angle) {
        m_PivotIO.setTarget(angle);
    }

    public Command getNewPivotTurnCommand(double i) {
        return new InstantCommand(
        () -> {
          setAngle(Degrees.of(i));
        },
        this);
    }

    @Override
    public void periodic() {
        m_PivotIO.updateInputs(loggedPivot);
        Logger.processInputs("RobotState/Pivot", loggedPivot);
    }
}
