package frc.robot.utiliy;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.drivetrain.Swerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Spindexer;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ArmConstants.Setpoints.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.IntakeConstants.GamePiece.*;

public class Superstructure {
    public final Swerve swerve;
    public final Spindexer spindexer;
    public final Arm arm;
    public final Claw claw;
    private Constants.IntakeConstants.GamePiece currentGamePiece;

    public Superstructure() {
        swerve = new Swerve();
        spindexer = new Spindexer();
        arm = new Arm();
        claw = new Claw();
        currentGamePiece = GamePiece.EMPTY;
    }

    //add led stuff
    public Command intakeCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> this.currentGamePiece = spindexer.currentPiece), //check
                spindexer.straightenGamePieceCommand().until(spindexer::isStraight),
                arm.holdSetpoint(SPINDEXER_SET_POINT),
                claw.autoClawCommand()
        );
    }

    public Command putOnUpperCommand() {
        return new SequentialCommandGroup(
                swerve.rotateToGridCommand(),
                arm.holdSetpoint(
                        currentGamePiece == CUBE ?
                                CUBE_HIGH_LEVEL_POINT :
                                CONE_HIGH_LEVEL_POINT),
                claw.openClawCommand(),
                arm.holdSetpoint(SPINDEXER_SET_POINT)
        );
    }

    public Command putOnMiddleCommand() {
        return new SequentialCommandGroup(
                swerve.rotateToGridCommand(),
                arm.holdSetpoint(
                        currentGamePiece == CUBE ?
                                CUBE_MID_LEVEL_POINT :
                                CONE_MID_LEVEL_POINT),
                claw.openClawCommand(),
                arm.holdSetpoint(SPINDEXER_SET_POINT)
        );
    }

    public Command putOnLowerCommand() {
        return new SequentialCommandGroup(
                swerve.rotateToGridCommand(),
                arm.holdSetpoint(
                        currentGamePiece == CUBE ?
                                CUBE_LOW_LEVEL_POINT :
                                CONE_LOW_LEVEL_POINT),
                claw.openClawCommand(),
                arm.holdSetpoint(SPINDEXER_SET_POINT)
        );
    }

    public Command driveCommand(
            DoubleSupplier xSpeed,
            DoubleSupplier ySpeed,
            DoubleSupplier xAngle,
            DoubleSupplier yAngle,
            BooleanSupplier fieldOriented,
            BooleanSupplier withAngle) {
        return swerve.dualDriveSwerveCommand(xSpeed, ySpeed, xAngle, yAngle, fieldOriented, withAngle);
    }
}
