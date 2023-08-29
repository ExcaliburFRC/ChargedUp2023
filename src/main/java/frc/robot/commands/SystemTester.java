package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Rollergripper;
import frc.robot.subsystems.swerve.Swerve;

import static frc.robot.Constants.IntakeConstants.LOW_RPM;

public class SystemTester extends SequentialCommandGroup {
  public SystemTester(Swerve swerve, Intake intake, Rollergripper rollerGripper){
    super(
          swerve.tankDriveCommand(()-> 0, ()-> 0.3, false).withTimeout(5),
          intake.shootCubeCommand(LOW_RPM).withTimeout(2),
          intake.intakeCommand(0).withTimeout(3),
          intake.toggleEjectPistonCommand().withTimeout(1),
          rollerGripper.intakeCommand().withTimeout(2),
          rollerGripper.holdConeCommand());
  }
}
