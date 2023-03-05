package frc.robot.commands.autonomous.noRamsete.cube;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.swerve.Swerve;
import frc.robot.utility.Calculation;

public class CubeAndLeave extends SequentialCommandGroup {

  public CubeAndLeave(Swerve swerve, Intake intake, int height){
    super(
          intake.shootCubeCommand(height).withTimeout(3),
          new ConditionalCommand(
                new InstantCommand(
                      ()-> swerve.resetOdometryCommand(
                            new Pose2d(1.95, 1.06, new Rotation2d(180)))
                ),
                new InstantCommand(
                      ()-> swerve.resetOdometryCommand(
                            new Pose2d(14.62, 1.07, new Rotation2d(0)))
                ),
                Calculation::isBlueAlliance
          ),
          swerve.driveSwerveCommand(()-> -0.25, ()-> 0, ()-> 0, ()-> false)
                .withTimeout(8)
    );
  }
}
