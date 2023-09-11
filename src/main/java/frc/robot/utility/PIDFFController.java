package frc.robot.utility;

import edu.wpi.first.math.controller.*;

public class PIDFFController extends PIDController {
    private ArmFeedforward armFFController = new ArmFeedforward(0, 0, 0, 0);
    private ElevatorFeedforward elevatorFFController = new ElevatorFeedforward(0, 0, 0, 0);
    private SimpleMotorFeedforward motorFFController = new SimpleMotorFeedforward(0, 0, 0);

    public PIDFFController(double kp, double ki, double kd){
        super(kp, ki, kd);
    }

    public void setArmFFconstants(double ks, double kg, double kv, double ka){
        this.armFFController = new ArmFeedforward(ks, kg, kv, ka);
    }

    public void setElevatorFFconstants(double ks, double kg, double kv, double ka){
        this.elevatorFFController = new ElevatorFeedforward(ks, kg, kv, ka);
    }

    public void setMotorFFconstants(double ks, double kv, double ka){
        this.motorFFController = new SimpleMotorFeedforward(ks, kv, ka);
    }

    /**
     * armFF - positionRadians, velocityRadPerSec
     * motorFF - velocity, accel
     * ElevatorFF - velocity, accel
     * @param measurement The current measurement of the process variable.
     * @param setpoint The new setpoint of the controller.
     * @return the calculated output
     */

    @Override
    public double calculate(double measurement, double setpoint){
        return this.calculate(measurement, setpoint) +
                armFFController.calculate(setpoint, 0) +
                motorFFController.calculate(setpoint, 0) +
                elevatorFFController.calculate(setpoint, 0);
    }
}
