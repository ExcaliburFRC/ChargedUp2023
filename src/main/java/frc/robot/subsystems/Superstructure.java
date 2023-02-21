package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants.Setpoints;
import frc.robot.drivetrain.Swerve;

import java.util.concurrent.atomic.AtomicReference;

import static frc.robot.Constants.ClawConstants.GamePiece;
import static frc.robot.subsystems.LEDs.LedMode.*;

public class Superstructure extends SubsystemBase {

   private final Arm arm = new Arm();
   private final Claw claw = new Claw();
   private final Spindexer spindexer = new Spindexer();
   private final Intake intake = new Intake();

    static AtomicReference<GamePiece> currentGamePiece = new AtomicReference<>(GamePiece.EMPTY);
    static AtomicReference<GamePiece> lastRequestedGamePiece = new AtomicReference<>();

    public Superstructure() {
        arm.setDefaultCommand(arm.holdSetpointCommand(Setpoints.SPINDEXER.gamePiece));
    }

    private boolean isCone(){
        return currentGamePiece.equals(GamePiece.CONE);
    }

    private Command setCurrentGamePiece(GamePiece gamePiece){
        return new InstantCommand(()-> currentGamePiece.set(gamePiece));
    }
    public Command setLastRequestedGamePiece(GamePiece gamePiece){
        return new InstantCommand(()-> lastRequestedGamePiece.set(gamePiece));
    }

    public Command intakeFromClawCommand() {
        return new SequentialCommandGroup(
                claw.openClawCommand(),
                arm.holdSetpointCommand(Setpoints.INTAKE.gamePiece),
                claw.autoCloseCommand(),
                setCurrentGamePiece(lastRequestedGamePiece.get()));
    }

    public Command intakeCommand(){
        return new SequentialCommandGroup(
                claw.openClawCommand(),
                intake.intakeCommand().until(spindexer.beambreakTrigger),
                spindexer.straightenGamePieceCommand(),
                setCurrentGamePiece(spindexer.getCurrentGamePiece()));
    }

    public Command intakeFromShelfCommand(){
        return new SequentialCommandGroup(
                claw.openClawCommand(),
                arm.holdSetpointCommand(Setpoints.SHELF.gamePiece),
                claw.autoCloseCommand(),
                setCurrentGamePiece(lastRequestedGamePiece.get()));
    }

    public Command placeOnHighCommand(Trigger release) {
        return new SequentialCommandGroup(
                claw.closeClawCommand(),
                arm.holdSetpointCommand(isCone()? Setpoints.HIGH.cone : Setpoints.HIGH.cube),
                claw.releaseCommand(release),
                setCurrentGamePiece(GamePiece.EMPTY));
    }

    public Command placeOnMidCommand(Trigger release) {
        return new SequentialCommandGroup(
                claw.closeClawCommand(),
                arm.holdSetpointCommand(isCone()? Setpoints.MID.cone : Setpoints.MID.cube),
                claw.releaseCommand(release),
                setCurrentGamePiece(GamePiece.EMPTY));
    }

    public Command placeOnLowCommand(Trigger release) {
        return new SequentialCommandGroup(
                claw.closeClawCommand(),
                arm.holdSetpointCommand(isCone()? Setpoints.LOW.cone : Setpoints.LOW.cube),
                claw.releaseCommand(release),
                setCurrentGamePiece(GamePiece.EMPTY));
    }
}
