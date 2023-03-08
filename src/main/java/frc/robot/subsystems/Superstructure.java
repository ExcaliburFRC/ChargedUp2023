package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;

import static frc.robot.Constants.ArmConstants.Setpoints.*;

public class Superstructure extends SubsystemBase {
   public final Arm arm = new Arm();
   public final RollerGripper rollerGripper = new RollerGripper();

    public Superstructure() {
      rollerGripper.setDefaultCommand(rollerGripper.holdConeCommand());
      arm.setDefaultCommand(arm.closeArmCommand());

      Shuffleboard.getTab("Arm").add("arm", arm);
    }

    public Command intakeFromShelfCommand(){
        return new ParallelCommandGroup(
                rollerGripper.intakeCommand(),
                arm.holdSetpointCommand(SHELF_EXTENDED.setpoint))
              .until(rollerGripper.beambreakTrigger)
              .andThen(arm.holdSetpointCommand(SHELF_RETRACTED.setpoint).withTimeout(0.5));
    }

    public Command placeOnHighCommand(Trigger release) {
        return new ParallelCommandGroup(
              arm.holdSetpointCommand(HIGH.setpoint),
              rollerGripper.releaseCommand(release))
              .until(rollerGripper.beambreakTrigger.negate().debounce(0.3));
    }

    public Command placeOnMidCommand(Trigger release) {
      return new ParallelCommandGroup(
            arm.holdSetpointCommand(MID.setpoint),
            rollerGripper.releaseCommand(release))
            .until(rollerGripper.beambreakTrigger.negate().debounce(0.3));
    }

    public Command placeOnLowCommand(Trigger release) {
      return new ParallelCommandGroup(
            arm.holdSetpointCommand(LOW.setpoint),
            rollerGripper.releaseCommand(release))
            .until(rollerGripper.beambreakTrigger.negate().debounce(0.3));
    }

    public Command switchCommand(double height){
            if (height == Constants.IntakeConstants.LOW_RPM) return placeOnLowCommand(new Trigger(()-> true).debounce(5)); //TODO: find minimal time for cone placement
            if (height == Constants.IntakeConstants.MID_RPM) return placeOnMidCommand(new Trigger(()-> true).debounce(5));
            if (height == Constants.IntakeConstants.HIGH_RPM) return placeOnHighCommand(new Trigger(()-> true).debounce(5));
            return new InstantCommand(()->{});
    }

    public Command resetArmCommand(){
      return arm.resetLengthCommand();
    }
}
