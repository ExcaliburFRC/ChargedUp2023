package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ArmConstants.ConeDutyCycle.*;
import static frc.robot.Constants.ClawConstants.GamePiece;

public class Superstructure extends SubsystemBase {
   public final Arm arm = new Arm();
   public final RollerGripper rollerGripper = new RollerGripper();

    static AtomicReference<GamePiece> currentGamePiece = new AtomicReference<>(GamePiece.EMPTY);
    static AtomicReference<GamePiece> lastRequestedGamePiece = new AtomicReference<>();

    public Superstructure() {
      arm.setDefaultCommand(
            arm.defaultCommand());

      rollerGripper.setDefaultCommand(
            rollerGripper.holdConeCommand());
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

    public Command intakeFromShelfCommand(BooleanSupplier accel, BooleanSupplier reduce){
        return new ParallelCommandGroup(
                rollerGripper.intakeCommand(),
                arm.holdArmCommand(SHELF.dc, accel, reduce, SHELF.telescope))
              .until(rollerGripper.buttonTrigger);
    }

    public Command placeOnHighCommand(Trigger release, BooleanSupplier accel, BooleanSupplier reduce) {
        return new ParallelCommandGroup(
                arm.holdArmCommand(HIGH.dc, accel, reduce, HIGH.telescope),
              rollerGripper.releaseCommand(release))
              .until(rollerGripper.buttonTrigger.negate().debounce(0.3));
    }

    public Command placeOnMidCommand(Trigger release, BooleanSupplier accel, BooleanSupplier reduce) {
      return new ParallelCommandGroup(
            arm.holdArmCommand(MID.dc, accel, reduce, MID.telescope),
            rollerGripper.releaseCommand(release))
            .until(rollerGripper.buttonTrigger.negate().debounce(0.3));
    }

    public Command placeOnLowCommand(Trigger release, BooleanSupplier accel, BooleanSupplier reduce) {
      return new ParallelCommandGroup(
            arm.holdArmCommand(LOW.dc, accel, reduce, LOW.telescope),
            rollerGripper.releaseCommand(release))
            .until(rollerGripper.buttonTrigger.negate().debounce(0.3));
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
