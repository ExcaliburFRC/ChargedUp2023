package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.Swerve;

public class ClimbOverRampCommand extends SequentialCommandGroup {
  public ClimbOverRampCommand(Swerve swerve, boolean isForward){
    super(
            swerve.driveToRampCommand(isForward),
            swerve.driveSwerveCommand(()-> isForward? 0.2 : -0.2, ()-> 0, swerve::getAngleDC, ()-> true)
                    .until(()-> swerve.getRobotPitch() > 10),
            swerve.driveSwerveCommand(()-> isForward? 0.2 : -0.2, ()-> 0, swerve::getAngleDC, ()-> true).withTimeout(1.65),
            swerve.climbCommand(!isForward)
    );
  }
}
