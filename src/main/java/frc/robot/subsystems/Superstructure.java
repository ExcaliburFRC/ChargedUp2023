package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants.Setpoints;

import static frc.robot.Constants.ArmConstants.Setpoints.*;

public class Superstructure extends SubsystemBase {
  public final Arm arm = new Arm();
  public final RollerGripper rollerGripper = new RollerGripper();

  public Superstructure() {
    rollerGripper.setDefaultCommand(rollerGripper.holdConeCommand());

//    arm.setDefaultCommand(arm.closeArmCommand());

    Shuffleboard.getTab("Arm").add("arm", arm);
  }

  public Command intakeFromShelfCommand() {
    return new ParallelCommandGroup(
          rollerGripper.intakeCommand(),
          arm.holdSetpointCommand(SHELF_EXTENDED.setpoint))
          .until(rollerGripper.beambreakTrigger)
          .andThen(arm.holdSetpointCommand(SHELF_RETRACTED.setpoint).withTimeout(0.5));
  }

  public Command placeOnHighCommand(Trigger release) {
    return new SequentialCommandGroup(
          arm.holdSetpointCommand(HIGH_CHECKPOINT.setpoint).withTimeout(1),
          arm.holdSetpointCommand(HIGH.setpoint))
          .raceWith(new WaitUntilCommand(release))
          .andThen(arm.lowerArmCommand().alongWith(
                            rollerGripper.ejectCommand(0.035)))
          .until(rollerGripper.beambreakTrigger.negate().debounce(0.3));
  }

  public Command placeOnMidCommand(Trigger release) {
    return arm.holdSetpointCommand(MID.setpoint)
          .raceWith(new WaitUntilCommand(release))
          .andThen(arm.lowerArmCommand().alongWith(
                rollerGripper.ejectCommand()))
          .until(rollerGripper.beambreakTrigger.negate().debounce(0.3));
  }

  public Command placeOnLowCommand(Trigger release) {
    return  arm.holdSetpointCommand(LOW.setpoint)
          .raceWith(new WaitUntilCommand(release))
          .andThen(arm.lowerArmCommand().alongWith(
                            rollerGripper.ejectCommand()))
          .until(rollerGripper.beambreakTrigger.negate().debounce(0.3));
  }

  public Command initArmCommand(Setpoints setpoint) {
          return Commands.parallel(
          arm.resetLengthCommand(),
          Commands.waitSeconds(1).andThen(arm.moveToAngleCommand(setpoint.setpoint)))
                .unless(rollerGripper.beambreakTrigger.negate());
  }

  public Command switchCommand(double height){
    if (height == Constants.IntakeConstants.LOW_RPM) return placeOnLowCommand(new Trigger(()-> true).debounce(5)); //TODO: find minimal time for cone placement
    if (height == Constants.IntakeConstants.MID_RPM) return placeOnMidCommand(new Trigger(()-> true).debounce(5));
    if (height == Constants.IntakeConstants.HIGH_RPM) return placeOnHighCommand(new Trigger(()-> true).debounce(5));
    return new InstantCommand(()->{});
  }

}
