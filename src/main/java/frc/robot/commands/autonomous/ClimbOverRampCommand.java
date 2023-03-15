package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.swerve.Swerve;

public class ClimbOverRampCommand extends SequentialCommandGroup {
  public ClimbOverRampCommand(Swerve swerve, boolean isForward){
    super(
          swerve.driveToRampCommand(isForward),
          swerve.tankDriveCommand(()-> isForward? 0.2 : 0.2,()-> 0, false)
                .until(()-> swerve.getRampAngle() < 0),
          // TODO fix positive / negative gyro pitch
          swerve.tankDriveCommand(()-> isForward? 0.2 : 0.2,()-> 0, false).withTimeout(2),
          swerve.climbCommand(!isForward)
    );
  }
}
