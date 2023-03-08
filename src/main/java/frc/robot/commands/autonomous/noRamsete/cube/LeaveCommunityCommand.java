package frc.robot.commands.autonomous.noRamsete.cube;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.swerve.Swerve;
import frc.robot.utility.Calculation;

public class LeaveCommunityCommand extends SequentialCommandGroup {

  public LeaveCommunityCommand(Swerve swerve){
    super(
            swerve.driveSwerveCommand(
                    ()-> 0, ()-> 0.3, ()-> 0, ()-> true)
                    .withTimeout(10) // TODO: find time
    );
  }
}
