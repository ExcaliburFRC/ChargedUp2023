package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static com.revrobotics.CANSparkMax.SoftLimitDirection.kForward;
import static com.revrobotics.CANSparkMax.SoftLimitDirection.kReverse;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.CuberConstants.*;
import static frc.robot.subsystems.LEDs.LEDPattern.SOLID;
import static frc.robot.utility.Colors.GREEN;
import static frc.robot.utility.Colors.RED;

public class Cuber extends SubsystemBase {
    private final CANSparkMax angleMotor = new CANSparkMax(ANGLE_MOTOR_ID, kBrushless);
    private final CANSparkMax shooterMotor = new CANSparkMax(ROLLERS_MOTOR_ID, kBrushless);

    private final Servo servo = new Servo(SERVO_CHANNEL);

    private final RelativeEncoder shooterEncoder = shooterMotor.getEncoder();
    private final RelativeEncoder angleRelativeEncoder = angleMotor.getEncoder();

    private final DutyCycleEncoder angleEncoder = new DutyCycleEncoder(ENCODER_CHANNEL);
    private final Ultrasonic ultrasonic = new Ultrasonic(ULTRASONIC_PING_CHANNEL, ULTRASONIC_ECHO_CHANNEL);

    private final LEDs leds = LEDs.getInstance();

    public static final ShuffleboardTab cuberTab = Shuffleboard.getTab("Cuber");

    private final PIDController shooterPID = new PIDController(Kp_SHOOTER, 0, Kd_SHOOTER);
    private final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(Ks_SHOOTER, Kv_SHOOTER, Ka_SHOOTER);

    private final PIDController anglePID = new PIDController(Kp_ANGLE, 0, Kd_ANGLE);
    private final ArmFeedforward angleFF = new ArmFeedforward(Ks_ANGLE, Kg_ANGLE, Kv_ANGLE, Ka_ANGLE);

    public final Trigger hasCubeTrigger = new Trigger(() -> ultrasonic.getRangeMM() <= ULTRASONIC_THRESHOLD).debounce(0.2)
            .onTrue(leds.applyPatternCommand(SOLID, GREEN.color).withTimeout(0.25))
            .onFalse(leds.applyPatternCommand(SOLID, RED.color).withTimeout(0.25));

    public int targetVel = 0;
    public final Trigger isAtTargetVelTrigger = new Trigger(() -> Math.abs(targetVel - shooterEncoder.getVelocity()) < VEL_THRESHOLD).debounce(0.2);

    public int targetPos = 0;
    public final Trigger isAtTargetPosTrigger = new Trigger(() -> Math.abs(angleEncoder.getDistance() - targetPos) < POS_THRESHOLD).debounce(0.2);

    public Cuber() {
        angleMotor.restoreFactoryDefaults();
        angleMotor.clearFaults();
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        angleMotor.setSoftLimit(kForward, FWD_SOFT_LIMIT);
        angleMotor.enableSoftLimit(kForward, true);
        angleMotor.setSoftLimit(kReverse, REV_SOFT_LIMIT);
        angleMotor.enableSoftLimit(kReverse, true);
        angleMotor.setInverted(false);

        shooterMotor.restoreFactoryDefaults();
        shooterMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shooterMotor.clearFaults();
        shooterMotor.setInverted(false);

        angleEncoder.setPositionOffset(ABS_ENCODER_OFFSET);
        angleEncoder.setDistancePerRotation(2 * Math.PI);

        angleRelativeEncoder.setPositionConversionFactor(ANGLE_CONVERSION_FACTOR);
        angleRelativeEncoder.setPosition(angleEncoder.getDistance());

        ultrasonic.setEnabled(true);
        Ultrasonic.setAutomaticMode(true);

        cuberTab.addDouble("ultrasonicMM", ultrasonic::getRangeMM);
        cuberTab.addBoolean("hasCubeTrigger", hasCubeTrigger);

        setDefaultCommand(closeCuberCommand());
    }

    // Servo Commands
    private Command setServoAngleCommand(SERVO_ANGLE angle) {
        return new RunCommand(() -> servo.setAngle(angle.angle)).until(() -> servo.getAngle() == angle.angle);
    }

    public double getServoAngle() {
        return servo.getAngle();
    }

    private Command retractServoCommand() {
        return setServoAngleCommand(SERVO_ANGLE.RETRACTED);
    }

    private Command pushCubeCommand() {
        return setServoAngleCommand(SERVO_ANGLE.EXTENDED);
    }

    // Shooter Commands
    private Command setShooterVelocityCommand(SHOOTER_VELOCITIY vel) {
        return new FunctionalCommand(
                () -> this.targetVel = vel.velocity,
                () -> {
                    double pid = shooterPID.calculate(shooterEncoder.getVelocity(), vel.velocity);
                    double ff = shooterFF.calculate(vel.velocity);

                    shooterMotor.set(pid + ff);
                },
                (__) -> {
                    shooterMotor.stopMotor();
                    targetVel = 0;
                },
                () -> false
        );
    }

    private Command stopShooterCommand() {
        return new InstantCommand(shooterMotor::stopMotor);
    }

    // Angle Commands
    private Command setCuberAngleCommand(CUBER_ANGLE angle) {
        return new FunctionalCommand(
                () -> targetPos = angle.angle,
                () -> {
                    double pid = anglePID.calculate(angleEncoder.getDistance(), angle.angle);
                    double ff = angleFF.calculate(angle.angle, 0);

                    angleMotor.set(pid + ff);
                },
                (__) -> {
                },
                () -> false
        );
    }

    // mainCommands
    private Command requirement() {
        return new RunCommand(() -> {
        }, this);
    }

    public Command closeCuberCommand() {
        return new ParallelCommandGroup(
                setCuberAngleCommand(CUBER_ANGLE.CLOSED),
                stopShooterCommand(),
                retractServoCommand(),
                requirement());
    }

    public Command intakeCommand(CUBER_ANGLE cuberAngle) {
        return new ParallelCommandGroup(
                setCuberAngleCommand(cuberAngle),
                setShooterVelocityCommand(SHOOTER_VELOCITIY.INTAKE),
                requirement())
                .until(hasCubeTrigger.debounce(0.2));
    }

    public Command shootCubeCommand(SHOOTER_VELOCITIY vel, CUBER_ANGLE angle) {
        return new ParallelCommandGroup(
                setCuberAngleCommand(angle),
                setShooterVelocityCommand(vel),
                new WaitUntilCommand(isAtTargetVelTrigger.and(isAtTargetPosTrigger)).andThen(pushCubeCommand()),
                requirement())
                .until(hasCubeTrigger.negate().debounce(0.2));
    }
}