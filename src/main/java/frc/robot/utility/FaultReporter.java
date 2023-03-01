package frc.robot.utility;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;

import java.lang.reflect.Field;
import java.util.HashSet;
import java.util.Set;

public class FaultReporter {
  PowerDistribution pd = new PowerDistribution();
  PneumaticHub ph = new PneumaticHub();

  public void check() {
    // pdh checks
    try {
      var pdhFaults = checkFaultsObject(pd.getFaults());
      if (!pdhFaults.isEmpty()) {
        DriverStation.reportError("PowerDistributionFaults: " + pdhFaults, false);
      }
    } catch (IllegalAccessException e) {
      DriverStation.reportError(e.getMessage(), e.getStackTrace());
    }

    // ph checks
    try {
      var phFaults = checkFaultsObject(ph.getFaults());
      if (!phFaults.isEmpty()) {
        DriverStation.reportError("PneumaticsHubFaults: " + phFaults, false);
      }
    } catch (IllegalAccessException e) {
      DriverStation.reportError(e.getMessage(), e.getStackTrace());
    }
  }

  public static void checkSparkMax(CANSparkMax sparkMax){
    Set<String> result = new HashSet<>();
    if (sparkMax.getFaults() != 0) {
      for (CANSparkMax.FaultID fault : CANSparkMax.FaultID.values()){
        if (sparkMax.getFault(fault)) {
          result.add(fault.name());
        }
      }
      DriverStation.reportError("SPARK MAX " + sparkMax.getDeviceId() + ": " + result, false);
    }
  }

  private Set<String> checkFaultsObject(Object faults) throws IllegalAccessException {
    Set<String> result = new HashSet<>();
    for (Field field : faults.getClass().getFields()) {
      if (field.getBoolean(faults)) result.add(field.getName());
    }

    return result;
  }
}
