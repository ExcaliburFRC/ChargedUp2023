package frc.robot.commands.autonomous.noRamsete.cube;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.swerve.Swerve;

public class CubeAndClimb extends SequentialCommandGroup {

  public CubeAndClimb(Swerve swerve, Intake intake, int height){
    super(
          intake.shootCubeCommand(height, ()-> 0).withTimeout(1.5),
          swerve.driveSwerveCommand(()-> -0.2, ()-> 0, ()-> 0, ()-> false)
                .withTimeout(6.5)
    );
  }
}
