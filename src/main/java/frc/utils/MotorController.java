package frc.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.rambots4571.rampage.tools.PIDTuner;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Settings;

public class MotorController implements Sendable {
  private final WPI_TalonFX motor;
  private final PIDTuner tuner;

  private int kPIDLoopIdx;
  private final int timeoutMs = Settings.timeoutMs;

  private double targetRPM, toleranceRPM;

  public MotorController(WPI_TalonFX motor) {
    this.motor = motor;
    this.tuner = new PIDTuner();
    this.tuner.setUpdater(this::updatePID);
    this.kPIDLoopIdx = 0;
  }

  public void setkPIDLoopIdx(int kPIDLoopIdx) {
    this.kPIDLoopIdx = kPIDLoopIdx;
  }

  public void setPIDF(double kP, double kI, double kD, double kF) {
    this.tuner.setPIDF(kP, kI, kD, kF);
    setMotorPIDF(kP, kI, kD, kF);
  }

  private void setMotorPIDF(double kP, double kI, double kD, double kF) {
    motor.config_kP(kPIDLoopIdx, kP, timeoutMs);
    motor.config_kI(kPIDLoopIdx, kI, timeoutMs);
    motor.config_kD(kPIDLoopIdx, kD, timeoutMs);
    motor.config_kF(kPIDLoopIdx, kF, timeoutMs);
  }

  /** This syncs up the tuner PIDF values with the motor. This should be run frequently */
  public void updatePID(PIDTuner tuner) {
    setMotorPIDF(tuner.getkP(), tuner.getkI(), tuner.getkD(), tuner.getkF());
  }

  public PIDTuner getTuner() {
    return this.tuner;
  }

  /**
   * How much RPM can it be off by
   *
   * @param rpm
   */
  public void setTolerance(double rpm) {
    toleranceRPM = rpm;
    motor.configAllowableClosedloopError(kPIDLoopIdx, convertRPMToRaw(rpm));
  }

  public double getRawSpeed() {
    return motor.getSelectedSensorVelocity(kPIDLoopIdx);
  }

  public double convertRawToRPM(double ticksPer100ms) {
    return ticksPer100ms * 600.0 / 2048.0;
  }

  public double convertRPMToRaw(double rpm) {
    return rpm * 2048.0 / 600.0;
  }

  public double getRPM() {
    return convertRawToRPM(getRawSpeed());
  }

  public void setRPM(double rpm) {
    targetRPM = rpm;
    double rawSpeed = convertRPMToRaw(rpm);
    motor.set(ControlMode.Velocity, rawSpeed);
  }

  public boolean isAtSpeed() {
    return Math.abs(targetRPM - getRPM()) < toleranceRPM;
  }

  // This only works for low gear with a 15.32:1 gear reduction
  public double nativeUnitsToDistanceMeters(double sensorCounts) {
    double motorRotations = (double) sensorCounts / DriveConstants.kEncoderResolution;
    double wheelRotations = motorRotations / DriveConstants.kGearRatio;
    double positionMeters =
        wheelRotations * (2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelWheelRadiusInch));
    return positionMeters;
  }

  // This only works for low gear with a 15.32:1 gear reduction
  public double nativeUnitsToVelocity(double sensorCountsPer100ms) {
    double motorRotationsPer100ms = sensorCountsPer100ms / DriveConstants.kEncoderResolution;
    double motorRotationsPerSecond = motorRotationsPer100ms * Settings.k100msPerSec;
    double wheelRotationsPerSecond = motorRotationsPerSecond * DriveConstants.kGearRatio;
    double VelocityMetersPerSec =
        wheelRotationsPerSecond
            * (2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelWheelRadiusInch));
    return VelocityMetersPerSec;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("MotorController");
    builder.addDoubleProperty("raw speed", this::getRawSpeed, null);
    builder.addDoubleProperty("rpm", this::getRPM, null);
    builder.addDoubleProperty("target RPM", () -> this.targetRPM, null);
    builder.addBooleanProperty("is at speed", this::isAtSpeed, null);
  }
}
