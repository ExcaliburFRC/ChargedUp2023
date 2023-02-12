package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;
import frc.robot.drivetrain.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SuperStructure extends SubsystemBase {
  private final Swerve swerve;
  private final Spindexer spindexer;
  //private final Arm arm;
  private final Claw claw;
  private GamePiece currentGamePiece;
  public SuperStructure(){
    swerve = new Swerve();
    spindexer = new Spindexer();
    //arm = new Arm();
    claw = new Claw();
    currentGamePiece = GamePiece.EMPTY;
  }
  //add led stuff
  public Command intakeCommand(){
    return new SequentialCommandGroup(
          new InstantCommand(()->this.currentGamePiece = spindexer.currentPiece),//check
          spindexer.straightenGamePieceCommand().until(spindexer::isStraight),
          //arm.goToCommand(Setpoints.SPINDEXER_SET_POINT),
          claw.autoClawCommand()
    );
  }
  public Command putOnUpperCommand(){
    return new SequentialCommandGroup(
        //  swerve.rotateToGridCommand(),
          // arm.holdSetPoint(
          // currentGamePiece == GamePiece.CUBE?
          // setPoints.CUBE_HIGH_LEVEL_POINT:
          // setPoints.CONE_HIGH_LEVEL_POINT),
          claw.openClawCommand()
          //,arm.holdPoint(setPoints.SPINDEXER_SET_POINT)
    );
  }
  public Command putOnMiddleCommand(){
    return new SequentialCommandGroup(
          //  swerve.rotateToGridCommand(),
          // arm.holdSetPoint(
          // currentGamePiece == GamePiece.CUBE?
          // setPoints.CUBE_MID_LEVEL_POINT:
          // setPoints.CONE_MID_LEVEL_POINT),
          claw.openClawCommand()
          //,arm.holdPoint(setPoints.SPINDEXER_SET_POINT)
    );
  }
  public Command putOnLowerCommand(){
    return new SequentialCommandGroup(
          //  swerve.rotateToGridCommand(),
          // arm.holdSetPoint(
          // currentGamePiece == GamePiece.CUBE?
          // setPoints.CUBE_LOW_LEVEL_POINT:
          // setPoints.CONE_LOW_LEVEL_POINT),
          claw.openClawCommand()
          //,arm.holdPoint(setPoints.SPINDEXER_SET_POINT)
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
