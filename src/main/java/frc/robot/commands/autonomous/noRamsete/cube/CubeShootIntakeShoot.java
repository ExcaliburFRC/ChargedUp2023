package frc.robot.commands.autonomous.noRamsete.cube;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.swerve.Swerve;
import frc.robot.utility.Calculation;

public class CubeShootIntakeShoot extends SequentialCommandGroup {
    public CubeShootIntakeShoot(Swerve swerve, Intake intake, int firstHeight, int secondHeight) {
        super(intake.shootCubeCommand(firstHeight).withTimeout(0.5)
                        .andThen(intake.intakeCommand(0.5)),
                new ConditionalCommand(
                        new InstantCommand(() ->
                                swerve.autoMotionCommand
                                                (new Pose2d(2.49, 4.61, new Rotation2d(-173.35)),
                                                        new Pose2d(4.53, 4.96, new Rotation2d(171.93)),
                                                        new Pose2d(6.69, 4.68, new Rotation2d(171.93))).withTimeout(0.5)
                                        .andThen(
                                                intake.intakeCommand(0),
                                                swerve.autoMotionCommand(new Pose2d(1.96, 4.39, new Rotation2d(179.03))),
                                                (intake.shootCubeCommand(secondHeight)))),

                        new InstantCommand(() ->
                                swerve.autoMotionCommand
                                                (new Pose2d(13.15, 4.81, new Rotation2d(4.19)),
                                                        new Pose2d(9.49, 4.57, new Rotation2d(4.19))).withTimeout(0.5)
                                        .andThen(
                                                intake.intakeCommand(0),
                                                swerve.autoMotionCommand(new Pose2d(14.51, 4.42, new Rotation2d(-179.43))),
                                                intake.shootCubeCommand(secondHeight)
                                        )
                        ),

                        Calculation::isBlueAlliance
                )

        );
    }
}
