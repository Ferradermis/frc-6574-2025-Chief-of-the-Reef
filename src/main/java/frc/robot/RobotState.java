package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;

public class RobotState {
  public static RobotState instance = new RobotState();
  public MutDistance elevatorPosition;
  public MutAngle armPosition;
  private MutAngle pivotTwist = Degrees.mutable(0);
  private MutAngle climberTwist = Degrees.mutable(0);

  private RobotState() {
    elevatorPosition = Inches.mutable(0);
    armPosition = Degrees.mutable(0);
  }

  public void updateElevatorPosition(MutDistance position) {
    elevatorPosition.mut_replace(position);
  }

  public void updateArmPosition(MutAngle position) {
    armPosition.mut_replace(position);
  }

  public void setPivotSource(MutAngle pivottwist) {
    pivotTwist = pivottwist;
  }

  public void setClimberSource(MutAngle climbertwist) {
    climberTwist = climbertwist;
  }
}
