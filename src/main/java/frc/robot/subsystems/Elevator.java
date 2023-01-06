package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Cvator;
import frc.robot.Constants.Settings;
import java.util.Arrays;
import java.util.List;

public class Elevator extends SubsystemBase {

  private final WPI_TalonFX baseMotorMaster, baseMotorFollower;
  private final List<WPI_TalonFX> allMotors;
  private final DigitalInput limitSwitch;

  private static Elevator instance = new Elevator();

  public static Elevator getInstance() {
    return instance;
  }

  private Elevator() {
    baseMotorMaster = new WPI_TalonFX(Cvator.baseMotorMaster);
    baseMotorFollower = new WPI_TalonFX(Cvator.baseMotorFollower);

    allMotors = Arrays.asList(baseMotorMaster, baseMotorFollower);

    allMotors.forEach(
        motor -> {
          motor.configFactoryDefault();
          motor.setNeutralMode(Cvator.EL_MODE);
          motor.configOpenloopRamp(Cvator.rampRate, Settings.timeoutMs);
          motor.configStatorCurrentLimit(Cvator.statorLimit);
          motor.configSupplyCurrentLimit(Cvator.supplyLimit);
        });
  }
}
