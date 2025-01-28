package frc.robot.subsystems.pivot;

public class PivotConstants {
  private PivotConstants instance;

  private PivotConstants() {}

  public PivotConstants getInstance() {
    if (instance == null) {
      instance = new PivotConstants();
    }
    return instance;
  }
}
