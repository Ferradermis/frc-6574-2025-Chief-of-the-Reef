package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private ClimberIO climberIO;

  ClimberInputsAutoLogged loggedClimber = new ClimberInputsAutoLogged();

  public Climber(ClimberIO io) {
    climberIO = io;
    loggedClimber.climberAngle = Degrees.mutable(0);
    loggedClimber.climberAngularVelocity = DegreesPerSecond.mutable(0);
    loggedClimber.climberSetPoint = Degrees.mutable(0);
    loggedClimber.supplyCurrent = Amps.mutable(0);
    loggedClimber.torqueCurrent = Amps.mutable(0);
    loggedClimber.voltageSetPoint = Volts.mutable(0);

   RobotState.getInstance().setClimberSource(loggedClimber.climberAngle);
  }

  public void setAngle(Angle angle) {
    climberIO.setClimberTarget(angle);
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
    climberIO.updateInputs(loggedClimber);
    Logger.processInputs("RobotState/Climber", loggedClimber);
  }
}
