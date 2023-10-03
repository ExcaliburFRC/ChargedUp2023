package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.Swerve;

public class LeaveCommunityCommand extends SequentialCommandGroup {
  public LeaveCommunityCommand(Swerve swerve, boolean isForward){
    super(
          swerve.tankDriveCommand(()-> isForward? 0.4 : -0.4, swerve::getAngleDC, true).withTimeout(4)
    );
  }
}
