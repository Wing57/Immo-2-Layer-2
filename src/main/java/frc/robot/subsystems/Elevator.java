package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.rambots4571.rampage.tools.PIDTuner;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Cvator;
import frc.robot.Constants.Settings;
import frc.utils.MotorController;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.swing.text.Position;

public class Elevator extends SubsystemBase {

  private final WPI_TalonFX baseMotorMaster, baseMotorFollower;
  private final List<WPI_TalonFX> allMotors;

  private final DigitalInput limitSwitch;

  private final MotorController baseMotorController;

  private static Elevator instance = new Elevator();

  private static Map<Position, Double> heights = new HashMap<>();

  static {
    
  }

  public static Elevator getInstance() {
    return instance;
  }

  private Elevator() {
    baseMotorMaster = new WPI_TalonFX(Cvator.BASE_MOTOR_MASTER);
    baseMotorFollower = new WPI_TalonFX(Cvator.BASE_MOTOR_FOLLOWER);

    allMotors = Arrays.asList(baseMotorMaster, baseMotorFollower);

    allMotors.forEach(
        motor -> {
          motor.configFactoryDefault();
          motor.setNeutralMode(Cvator.MODE);
          motor.configOpenloopRamp(Cvator.rampRate, Settings.timeoutMs);
          motor.configStatorCurrentLimit(Cvator.statorLimit);
          motor.configSupplyCurrentLimit(Cvator.supplyLimit);
          motor.enableVoltageCompensation(true);
          motor.configVoltageCompSaturation(12, Settings.timeoutMs);
        });
    
    baseMotorMaster.setInverted(Cvator.masterInvert);
    baseMotorFollower.setInverted(Cvator.followerInvert);

    baseMotorFollower.follow(baseMotorMaster);

    baseMotorController = new MotorController(baseMotorMaster);
    baseMotorController.setPIDF(Cvator.kP, Cvator.kI, Cvator.kD, Cvator.kF);
    baseMotorController.setTolerance(150);

    addChild("BaseMotor", baseMotorController);
    addChild("BaseMotor PID", baseMotorController.getTuner());

    limitSwitch = new DigitalInput(Cvator.LIMITSWITCH);

  }

  public void configMotionMagic() {
    baseMotorMaster.configSelectedFeedbackSensor(
        FeedbackDevice.IntegratedSensor, 
        0, Settings.timeoutMs);
    baseMotorMaster.setStatusFramePeriod(
        StatusFrameEnhanced.Status_13_Base_PIDF0, 
        10, Settings.timeoutMs);
    baseMotorMaster.setStatusFramePeriod(
        StatusFrameEnhanced.Status_10_MotionMagic, 
        10, Settings.timeoutMs);

    baseMotorMaster.configNominalOutputForward(0, Settings.timeoutMs);
    baseMotorMaster.configNominalOutputReverse(0, Settings.timeoutMs);
    baseMotorMaster.configPeakOutputForward(1, Settings.timeoutMs);
    baseMotorMaster.configPeakOutputReverse(-1, Settings.timeoutMs);

    baseMotorMaster.selectProfileSlot(0, 0);
    baseMotorMaster.configClosedloopRamp(0.15, Settings.timeoutMs);

    baseMotorMaster.configMotionCruiseVelocity(Cvator.cruiseVel, Settings.timeoutMs);
    baseMotorMaster.configMotionAcceleration(Cvator.motionAccel, Settings.timeoutMs);
  }
}
