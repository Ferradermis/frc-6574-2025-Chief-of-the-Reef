package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO elevatorIO;

  ElevatorIOInputsAutoLogged loggedElevator = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    elevatorIO = io;
    loggedElevator.distance = Meters.mutable(0);
    loggedElevator.velocity = MetersPerSecond.mutable(0);
    loggedElevator.setpoint = Meters.mutable(0);
    loggedElevator.supplyCurrent = Amps.mutable(0);
    loggedElevator.torqueCurrent = Amps.mutable(0);
    loggedElevator.voltageSetpoint = Volts.mutable(0);
  }

  public void setDistance(Distance target) {
    elevatorIO.setTarget(target);
  }

  public Command getNewSetDistanceCommand(LoggedTunableNumber distance) {
    return new InstantCommand(
        () -> {
          setDistance(Meter.of((Meters.convertFrom(distance.get(), Inches))));
        },
        this);
  }

  public Trigger getNewAtAngleTrigger(Distance dist, Distance tolerance) {
    return new Trigger(
        () -> {
          return MathUtil.isNear(
              dist.baseUnitMagnitude(),
              loggedElevator.distance.baseUnitMagnitude(),
              tolerance.baseUnitMagnitude());
        });
  }

  @Override
  public void periodic() {
    elevatorIO.updateInputs(loggedElevator);
    Logger.processInputs("RobotState/Elevator", loggedElevator);
  }
}
