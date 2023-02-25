package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ArmConstants.CubeDutyCycle.*;
import static frc.robot.Constants.ClawConstants.GamePiece;

public class Superstructure extends SubsystemBase {
   public final Arm arm = new Arm();
   private final RollerGripper rollerGripper = new RollerGripper();

    static AtomicReference<GamePiece> currentGamePiece = new AtomicReference<>(GamePiece.EMPTY);
    static AtomicReference<GamePiece> lastRequestedGamePiece = new AtomicReference<>();

    public Superstructure() {
//      arm.setDefaultCommand(
//            arm.defaultCommand());
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

    public Command intakeFromShelfCommand(BooleanSupplier accel){
        return new ParallelCommandGroup(
                rollerGripper.intakeCommand(),
                arm.holdArmCommand(SHELF.dc, accel, SHELF.telescope))
              .until(rollerGripper.buttonTrigger);
    }

    public Command placeOnHighCommand(Trigger release, BooleanSupplier accel) {
        return new ParallelCommandGroup(
                arm.holdArmCommand(HIGH.dc, accel, HIGH.telescope),
              rollerGripper.releaseCommand(release));
    }

    public Command placeOnMidCommand(Trigger release, BooleanSupplier accel) {
      return new ParallelCommandGroup(
            arm.holdArmCommand(MID.dc, accel, MID.telescope),
            rollerGripper.releaseCommand(release));
    }

    public Command placeOnLowCommand(Trigger release, BooleanSupplier accel) {
      return new ParallelCommandGroup(
            arm.holdArmCommand(LOW.dc, accel, LOW.telescope),
            rollerGripper.releaseCommand(release));
    }

    public Command manualCommand(
          DoubleSupplier angle,
          DoubleSupplier pov,
          BooleanSupplier intake,
          BooleanSupplier outtake,
          BooleanSupplier stop){
      return arm.povManualCommand(()-> angle.getAsDouble() / 4, pov)
            .alongWith(rollerGripper.manualCommand(intake, outtake, stop));
    }

    public Command floatCommand(){
      return arm.floatCommand();
    }
}
