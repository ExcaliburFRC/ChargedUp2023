package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.Cuber;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.Swerve;

public class SystemTester extends SequentialCommandGroup {
  public SystemTester(Swerve swerve, Cuber cuber, Superstructure superstructure, Trigger next){
    super(
            swerve.tankDriveCommand(()-> 0, ()-> 0.3, false).until(next),
            new WaitCommand(0.5),
            cuber.intakeCommand(Constants.CuberConstants.CUBER_ANGLE.INTAKE_SLIDE),
            new WaitCommand(0.5),
            cuber.shootCubeToLowerCommand(next),
            new WaitCommand(1),
            superstructure.intakeFromShelfCommand(),
            new WaitUntilCommand(next),
            new WaitCommand(0.5),
            superstructure.lockArmCommand()
            );
  }
}
