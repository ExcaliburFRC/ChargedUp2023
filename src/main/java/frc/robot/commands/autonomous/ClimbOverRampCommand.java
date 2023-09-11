package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.Swerve;

public class ClimbOverRampCommand extends SequentialCommandGroup {
  public ClimbOverRampCommand(Swerve swerve, boolean isForward){
    super(
          swerve.driveToRampCommand(isForward),
          swerve.tankDriveCommand(()-> isForward? 0.3 : -0.3, ()-> 0, true)
                  .until(()-> swerve.getRampAngle() < 0));
          // TODO fix positive / negative gyro pitch
//          swerve.tankDriveCommand(()-> isForward? 0.3 : -0.3, ()-> 0, true).withTimeout(3),
//          swerve.climbCommand(!isForward)
//    );
  }
}
