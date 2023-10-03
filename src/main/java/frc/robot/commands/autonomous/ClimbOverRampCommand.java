package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.Swerve;

public class ClimbOverRampCommand extends SequentialCommandGroup {
  public ClimbOverRampCommand(Swerve swerve, boolean isForward){
    super(
            swerve.driveToRampCommand(isForward),
            swerve.driveSwerveCommand(()-> isForward? 0.35 : -0.35, ()-> 0, () -> swerve.getAngleDC(getFwd(isForward)), ()-> true)
                    .until(()-> isOverRamp(isForward, swerve.getRobotPitch())),
            swerve.driveSwerveCommand(()-> isForward? 0.2 : -0.2, ()-> 0, () -> swerve.getAngleDC(getFwd(isForward)), ()-> true).withTimeout(1.23),
            swerve.climbCommand(!isForward)
    );
  }

  private static boolean isOverRamp(boolean isFwd, double pitch){
    if (isFwd) return pitch > 10;
    return pitch < - 10;
  }

  private static double getFwd(boolean isForward){
    return isForward? 0 : 180;
  }
}
