package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static edu.wpi.first.wpilibj.PneumaticsModuleType.REVPH;
import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
    private final CANSparkMax intakeMotor = new CANSparkMax(INTAKE_MOTOR_ID, kBrushless);
    private final DoubleSolenoid piston = new DoubleSolenoid(REVPH, FWD_CHANNEL, REV_CHANNEL);

    public Intake() {
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT);
        intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intakeMotor.setInverted(false);
    }

    public Command openPistonCommand() {
        return new InstantCommand(() -> {
            piston.set(DoubleSolenoid.Value.kForward);
        }, this);
    }

    public Command closePistonCommand() {
        return new InstantCommand(() -> {
            piston.set(DoubleSolenoid.Value.kReverse);
        },this);
    }

    public Command stopMotorCommand(){
        return new RunCommand(()-> intakeMotor.set(0), this);
    }

    public Command manualCommand(DoubleSupplier intakeSpeed, BooleanSupplier togglePiston) {
        return new RunCommand(
                () -> {
                    if (!piston.get().equals(DoubleSolenoid.Value.kReverse))
                    intakeMotor.set(intakeSpeed.getAsDouble());
                    else intakeMotor.stopMotor();

                    if (togglePiston.getAsBoolean()) {
                        if (piston.get().equals(DoubleSolenoid.Value.kReverse))
                            piston.set(DoubleSolenoid.Value.kForward);
                        else piston.set(DoubleSolenoid.Value.kReverse);
                    }
                },
                this);
    }

    public Command buttonBasedManualCommand(
          BooleanSupplier intakeButton,
          double intakeSpeed,
          BooleanSupplier cubeButton,
          double cubeSpeed,
          BooleanSupplier togglePiston){
        return new RunCommand(()->{
            if (intakeButton.getAsBoolean()) intakeMotor.set(intakeSpeed);
            else intakeMotor.set(0);

            if (cubeButton.getAsBoolean() && !intakeButton.getAsBoolean()) intakeMotor.set(cubeSpeed);

            if (togglePiston.getAsBoolean()) {
                if (piston.get().equals(DoubleSolenoid.Value.kReverse))
                    piston.set(DoubleSolenoid.Value.kForward);
                else piston.set(DoubleSolenoid.Value.kReverse);
            }
        }, this);
    }

    public Command intakeCommand(DoubleSupplier intakeSpeed){
        return new FunctionalCommand(
                ()-> piston.set(DoubleSolenoid.Value.kForward),
                ()-> intakeMotor.set(intakeSpeed.getAsDouble()),
                (__) -> {
                    intakeMotor.set(0);
                    piston.set(DoubleSolenoid.Value.kReverse);
                },
                ()-> false,
                this);
    }
}
