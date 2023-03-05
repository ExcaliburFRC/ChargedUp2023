package frc.robot.commands.autonomous.noRamsete.cube;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.swerve.Swerve;
import frc.robot.utility.Calculation;

public class CubeAndClimb extends SequentialCommandGroup {

  public CubeAndClimb(Swerve swerve, Intake intake, int height){
    super(
          intake.shootCubeCommand(height).withTimeout(1.5),
          new ConditionalCommand(
                new InstantCommand(
                      ()-> swerve.resetOdometryCommand(
                            new Pose2d(1.97, 2.74, new Rotation2d(180)))
                ),
                new InstantCommand(
                      ()-> swerve.resetOdometryCommand(
                            new Pose2d(14.61, 2.74, new Rotation2d(0)))
                ),
                Calculation::isBlueAlliance
          ),
          swerve.driveSwerveCommand(()-> -0.2, ()-> 0, ()-> 0, ()-> false)
                .withTimeout(6.5)
    );
  }
}
