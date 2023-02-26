package frc.robot.commands.autonomous.noRamsete.cube;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.swerve.Swerve;

public class CubeAndLeave extends SequentialCommandGroup {

  public CubeAndLeave(Swerve swerve, Intake intake, int height){
    super(
          intake.shootCubeCommand(height).withTimeout(0.5),
          swerve.driveSwerveCommand(()-> 0, ()-> 0.3, ()-> 0, ()-> true)
                .withTimeout(4)
    );
  }
}