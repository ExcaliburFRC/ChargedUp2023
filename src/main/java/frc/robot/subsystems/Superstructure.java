package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.utiliy.ToggleCommand;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ArmConstants.CubeDutyCycle.*;
import static frc.robot.Constants.ClawConstants.GamePiece;

public class Superstructure extends SubsystemBase {

   public final Arm arm = new Arm();
   private final Claw claw = new Claw();
//   private final Spindexer spindexer = new Spindexer();
//   private final Intake intake = new Intake();

    static AtomicReference<GamePiece> currentGamePiece = new AtomicReference<>(GamePiece.EMPTY);
    static AtomicReference<GamePiece> lastRequestedGamePiece = new AtomicReference<>();

    public Superstructure() {
      arm.setDefaultCommand(
            arm.retractTelescopeCommand());
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
                claw.autoCloseCommand(),
                setCurrentGamePiece(lastRequestedGamePiece.get()));
    }

//    public Command intakeCommand(){
//        return new SequentialCommandGroup(
//              arm.holdArmCommand(Constants.ArmConstants.DutyCycle.HIGH.angle, driveJoystick.R1(), Constants.ArmConstants.DutyCycle.HIGH.telescope)
//                          new WaitUntilCommand(driveJoystick.square()),
//                          claw.closeClawCommand()
//      );
//    }

    public Command intakeFromShelfCommand(BooleanSupplier accel){
        return new SequentialCommandGroup(
                claw.openClawCommand(),
                arm.holdArmCommand(SHELF.dc, accel, SHELF.telescope).alongWith(
                claw.autoCloseCommand())).until(claw.isClawOpenedTrigger.negate());
    }

    public Command placeOnHighCommand(Trigger release, BooleanSupplier accel) {
        return new SequentialCommandGroup(
                claw.closeClawCommand(),
                arm.holdArmCommand(HIGH.dc, accel, HIGH.telescope)
                            .alongWith(claw.releaseCommand(release))).until(release);
    }

    public Command placeOnMidCommand(Trigger release, BooleanSupplier accel) {
      return new SequentialCommandGroup(
            claw.closeClawCommand(),
            arm.holdArmCommand(MID.dc, accel, MID.telescope)
                  .alongWith(claw.releaseCommand(release))).until(release);
    }

    public Command placeOnLowCommand(Trigger release, BooleanSupplier accel) {
      return new SequentialCommandGroup(
            claw.closeClawCommand(),
            arm.holdArmCommand(LOW.dc, accel, LOW.telescope)
                  .alongWith(claw.releaseCommand(release))).until(release);
    }

    public Command manualCommand(DoubleSupplier angle, DoubleSupplier pov, BooleanSupplier togglePiston){
      return arm.povManualCommand(()-> angle.getAsDouble() / 4, pov)
            .alongWith(claw.toggleClawCommand(togglePiston));
    }

    public Command floatCommand(){
      return arm.floatCommand();
    }
}
