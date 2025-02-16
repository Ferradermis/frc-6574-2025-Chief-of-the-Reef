package frc.robot.subsystems.rotate;

import java.util.function.Consumer;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MutAngle;
import frc.robot.RobotState;
import frc.robot.util.CanDef;
import frc.robot.util.CanDef.CanBus;
import frc.robot.util.Gains;

public class RotateConstants {
    public CanDef leaderProfile = CanDef.builder().id(0).bus(CanBus.Rio).build();

    public Gains simGains =
        Gains.builder()
            .kS(0)
            .kG(0)
            .kV(0)
            .kA(0)
            .kP(0.1).kI(0).kD(0).build();

    public Gains NEOGains =
        Gains.builder().kS(0).kG(0).kV(0).kA(0).kP(0).kI(0).kD(0).build();

    public AngularVelocity maxVelocity = DegreesPerSecond.of(360);
    public AngularAcceleration maxAcceleration = DegreesPerSecondPerSecond.of(360);
    public double maxJerk = 0.0;
    public Current torqueCurrentLimit = Amps.of(120);
    public Current supplyCurrentLimit = Amps.of(40);
    public Current forwardTorqueLimit =  Amps.of(80);
    public Current reverseTorqueLimit = Amps.of(-80);

    public int numMotors = 1;
    public double gearing = 75;
    public Distance length = Inches.of(15);
    public Mass weight = Pounds.of(8.5);
    public DCMotor motors = DCMotor.getKrakenX60(numMotors);
    public Angle maximumAngle = Degrees.of(360);
    public Angle minimumAngle = Degrees.of(0);
    public Angle startingAngle = Degrees.zero();

    public Distance xPosition = Meters.of(0.07);
    public Distance yPosition = Meters.of(0);
    public Distance zPosition = Meters.of(0.377);
    public Angle pitchModifier = Degrees.of(84);

    public String loggedName = "Rotate";

    public Consumer<MutAngle> mechanismSimCallback = (d) -> {
        RobotState.getInstance().setRotateSource(d);
    };
}
