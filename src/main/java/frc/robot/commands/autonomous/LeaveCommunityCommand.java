package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.swerve.Swerve;

public class LeaveCommunityCommand extends SequentialCommandGroup {
  public LeaveCommunityCommand(Swerve swerve){
    super(
          swerve.tankDriveCommand(()-> -0.4, ()-> 0, false)
                .withTimeout(4) // TODO: find minimal time
    );
  }
}
