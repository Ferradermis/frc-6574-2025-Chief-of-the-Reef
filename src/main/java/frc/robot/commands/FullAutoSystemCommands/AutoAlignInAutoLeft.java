package frc.robot.commands.FullAutoSystemCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.FullTeleopSystemCommands.AlignToReef;
import frc.robot.commands.FullTeleopSystemCommands.AlignToReef.ReefPosition;

public class AutoAlignInAutoLeft extends SequentialCommandGroup {
    public AutoAlignInAutoLeft() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new AutoAlign(AlignToReef.getGetTargetPositionFunction(ReefPosition.Left, false), RobotContainer.drive).withTimeout(6)
        );
    }
}
