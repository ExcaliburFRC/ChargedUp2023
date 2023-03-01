package frc.robot.commands.autonomous.noRamsete.noGamePiece;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.swerve.Swerve;

public class LeaveCommunity extends SequentialCommandGroup {

  public LeaveCommunity(Swerve swerve){
    super(
          swerve.driveSwerveCommand(()-> 0, ()-> 0.3, ()-> 0, ()-> true)
                .withTimeout(4)
    );
  }
}
