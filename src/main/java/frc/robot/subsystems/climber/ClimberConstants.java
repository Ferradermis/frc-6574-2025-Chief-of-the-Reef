package frc.robot.subsystems.climber;

public class ClimberConstants {
  private ClimberConstants instance;

  private ClimberConstants() {}

  public ClimberConstants getInstance() {
    if (instance == null) {
      instance = new ClimberConstants();
    }
    return instance;
  }
}
