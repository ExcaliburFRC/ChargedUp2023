package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.drivetrain.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ClawConstants.GamePiece;
import static frc.robot.Constants.ArmConstants.Setpoints.*;

public class Superstructure extends SubsystemBase {
  private final Swerve swerve = new Swerve();
  private final Arm arm = new Arm();
  private final Claw claw = new Claw();
  private final Spindexer spindexer = new Spindexer();
  private Constants.ClawConstants.GamePiece currentGamePiece;
  public Superstructure(){
    currentGamePiece = GamePiece.EMPTY;
  }
  //add led stuff
  public Command intakeCommand(){
    return new SequentialCommandGroup(
          new InstantCommand(()->this.currentGamePiece = spindexer.currentPiece),//check
          spindexer.straightenGamePieceCommand().until(spindexer::isStraight),
          arm.holdSetpoint(SPINDEXER_SET_POINT),
          claw.autoClawCommand()
    );
  }
  public Command putOnUpperCommand(){
    return new SequentialCommandGroup(
          swerve.rotateToGridCommand(),
           arm.holdSetpoint(
           currentGamePiece == GamePiece.CUBE?
           CUBE_HIGH_LEVEL_POINT:
           CONE_HIGH_LEVEL_POINT),
          claw.openClawCommand(), 
            arm.holdSetpoint(SPINDEXER_SET_POINT));
  }
  public Command putOnMiddleCommand(){
    return new SequentialCommandGroup(
            swerve.rotateToGridCommand(),
           arm.holdSetpoint(
           currentGamePiece == GamePiece.CUBE?
           CUBE_MID_LEVEL_POINT:
           CONE_MID_LEVEL_POINT),
          claw.openClawCommand(),
            arm.holdSetpoint(SPINDEXER_SET_POINT)
    );
  }
  public Command putOnLowerCommand(){
    return new SequentialCommandGroup(
            swerve.rotateToGridCommand(),
           arm.holdSetpoint(
           currentGamePiece == GamePiece.CUBE?
           CUBE_LOW_LEVEL_POINT:
           CONE_LOW_LEVEL_POINT),
          claw.openClawCommand(),
            arm.holdSetpoint(SPINDEXER_SET_POINT)
    );
  }

  public Command driveCommand(DoubleSupplier xSpeed,
                              DoubleSupplier ySpeed,
                              DoubleSupplier xAngle,
                              DoubleSupplier yAngle,
                              BooleanSupplier fieldOriented,
                              BooleanSupplier withAngle){
    return swerve.dualDriveSwerveCommand(
          xSpeed,
          ySpeed,
          xAngle,
          yAngle,
          fieldOriented,
          withAngle
    );
  }
}
