package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants.Setpoints;
import frc.robot.drivetrain.Swerve;

import java.util.concurrent.atomic.AtomicReference;

import static frc.robot.Constants.ClawConstants.GamePiece;

public class Superstructure extends SubsystemBase {

    private final Swerve swerve = new Swerve();
    private final Arm arm = new Arm();
    private final Claw   claw = new Claw();
    private final Spindexer spindexer = new Spindexer();

    public static AtomicReference<GamePiece> currentGamePiece;
    public static AtomicReference<Setpoints> currentSetpoint;

    public Superstructure() {
        currentGamePiece.set(GamePiece.EMPTY);
        currentSetpoint.set(Setpoints.SPINDEXER);

        arm.setDefaultCommand(arm.holdSetpointCommand(Setpoints.spindexer));
    }

    private boolean isCone(){
        return currentGamePiece.equals(GamePiece.CONE);
    }

    private Command setCurrentSetpoint(Setpoints setpoint){
        return new InstantCommand(()-> currentSetpoint.set(setpoint));
    }

    //TODO: add led signalling
    public Command intakeFromGroundCommand() {
        return new SequentialCommandGroup(
                setCurrentSetpoint(Setpoints.INTAKE),
                arm.holdSetpointCommand(Setpoints.intake),
                claw.openClawCommand(),
                claw.autoClawCommand());
    }

    public Command placeOnHighCommand() {
        return new SequentialCommandGroup(
                claw.closeClawCommand(),
                swerve.rotateToGridCommand(),
                arm.holdSetpointCommand(isCone()? Setpoints.HIGH.cone : Setpoints.HIGH.cube),
                Commands.runOnce(()-> currentSetpoint.set(Setpoints.HIGH)),
                claw.openClawCommand(),
                new WaitCommand(0.1),
                arm.holdSetpointCommand(Setpoints.spindexer));
    }

    public Command placeOnMidCommand() {
        return new SequentialCommandGroup(
                claw.closeClawCommand(),
                swerve.rotateToGridCommand(),
                arm.holdSetpointCommand(isCone()? Setpoints.MID.cone : Setpoints.MID.cube),
                Commands.runOnce(()-> currentSetpoint.set(Setpoints.MID)),
                claw.openClawCommand(),
                new WaitCommand(0.1),
                arm.holdSetpointCommand(Setpoints.spindexer)
        );
    }

    public Command placeOnLowCommand() {
        return new SequentialCommandGroup(
                claw.closeClawCommand(),
                swerve.rotateToGridCommand(),
                arm.holdSetpointCommand(isCone()? Setpoints.LOW.cone : Setpoints.LOW.cube),
                Commands.runOnce(()-> currentSetpoint.set(Setpoints.LOW)),
                claw.openClawCommand(),
                new WaitCommand(0.1),
                arm.holdSetpointCommand(Setpoints.spindexer)
        );
    }
}
