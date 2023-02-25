package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.utiliy.ToggleCommand;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static edu.wpi.first.wpilibj.PneumaticsModuleType.REVPH;
import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
    private final CANSparkMax intakeMotor = new CANSparkMax(INTAKE_MOTOR_ID, kBrushless);

    private final DoubleSolenoid intakePiston = new DoubleSolenoid(REVPH, INTAKE_FWD_CHANNEL, INTAKE_REV_CHANNEL);
    private final DoubleSolenoid ejectPiston = new DoubleSolenoid(REVPH, EJECT_FWD_CHANNEL, EJECT_REV_CHANNEL);

    private final DigitalInput beambreak = new DigitalInput(BEAMBREAK_CHANNEL);

    private final Trigger beambreakTrigger = new Trigger(()-> !beambreak.get());

    public Intake() {
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT);
        intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intakeMotor.clearFaults();
    }

    public Command openPistonCommand() {
        return new InstantCommand(() -> {
            intakePiston.set(DoubleSolenoid.Value.kForward);
        }, this);
    }

    public Command closePistonCommand() {
        return new InstantCommand(() -> {
            intakePiston.set(DoubleSolenoid.Value.kReverse);
        },this);
    }

    public Command intakeCommand(double intakeSpeed){
        return new StartEndCommand(
              ()-> {
                  intakePiston.set(DoubleSolenoid.Value.kForward);
                  intakeMotor.set(intakeSpeed);
              },
              ()-> {
                  intakePiston.set(DoubleSolenoid.Value.kReverse);
                  intakeMotor.stopMotor();
              }, this)
              .until(beambreakTrigger);
    }

    private Command ejectCubeCommand(){
        return new InstantCommand((()-> ejectPiston.set(DoubleSolenoid.Value.kForward)));
    }

    private Command retractPistonCommand(){
        return new InstantCommand((()-> ejectPiston.set(DoubleSolenoid.Value.kReverse)));
    }

    public Command pulseMotorCommand(){
        return new RunCommand(()-> intakeMotor.set(-0.3)).withTimeout(0.1);
    }

    public Command shootCubeCommand(int height){
        switch (height){
            case 1:
                return Commands.repeatingSequence(
                      Commands.runEnd(()-> intakeMotor.set(-0.01), intakeMotor::stopMotor, this).withTimeout(0.3), pulseMotorCommand())
                      .alongWith(ejectCubeCommand())
                      .finallyDo((__)-> ejectPiston.set(DoubleSolenoid.Value.kForward));
            case 2:
                return Commands.runEnd(()-> intakeMotor.set(-0.3), intakeMotor::stopMotor, this)
                      .alongWith(new WaitCommand(0.2).andThen(ejectCubeCommand()))
                      .finallyDo((__)-> ejectPiston.set(DoubleSolenoid.Value.kForward));
            case 3:
                return Commands.runEnd(()-> intakeMotor.set(-0.54), intakeMotor::stopMotor, this)
                      .alongWith(new WaitCommand(0.4).andThen(ejectCubeCommand()))
                      .finallyDo((__)-> ejectPiston.set(DoubleSolenoid.Value.kForward));
            default:
              return new InstantCommand(()-> {});
        }
    }

    // top - 54
    // middle - 30
    // low - 5
}
