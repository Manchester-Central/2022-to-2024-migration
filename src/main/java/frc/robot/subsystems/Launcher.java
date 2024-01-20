// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chaos131.pid.PIDTuner;
import com.chaos131.pid.PIDFValue;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class Launcher extends SubsystemBase {
  private DoubleSolenoid m_solenoid;
  private TalonFX m_ControllerA;
  private TalonFX m_ControllerB;
  private TalonFXConfiguration configA;
  private TalonFXConfiguration configB;
  private VelocityVoltage m_velocityTarget = new VelocityVoltage(0);

  private PIDTuner m_pidTuner;

  private double m_speedTolerance = Constants.DefaultLauncherTolerance;

  /** Creates a new Launcher. */
  public Launcher() {
    m_solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.LauncherSolenoidForward,
        Constants.LauncherSolenoidReverse);
    configA = new TalonFXConfiguration();
    configB = new TalonFXConfiguration();
    m_ControllerA = new TalonFX(Constants.LauncherA);
    m_ControllerB = new TalonFX(Constants.LauncherB);
    configA.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.2;
    configB.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.2;
    configA.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configB.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  
    configA.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configB.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configA.MotorOutput.PeakReverseDutyCycle = 0;
    configB.MotorOutput.PeakReverseDutyCycle = 0;
    configA.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    configA.Feedback.SensorToMechanismRatio = Constants.TalonCountsPerRevolution / 100;
    configB.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    configB.Feedback.SensorToMechanismRatio = Constants.TalonCountsPerRevolution / 100;
    m_ControllerA.getConfigurator().apply(configA);
    m_ControllerB.getConfigurator().apply(configB);
   
    /*Revisit if Launcher inconsistancies
    m_ControllerA.configVoltageCompSaturation(12.4);
    configA.Voltage.
    m_ControllerB.configVoltageCompSaturation(12.4);

    m_ControllerA.enableVoltageCompensation(true);
    m_ControllerB.enableVoltageCompensation(true);*/

    // Lower CAN Utilization. We are reading data off controllerA, so we can slow the rate of status updates from B
    /*Revisit if Launcher inconsistancies
    m_ControllerB.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, Constants.MaxCANStatusFramePeriod);
    configB.MotorOutput.
    m_ControllerB.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, Constants.MaxCANStatusFramePeriod);*/

    double velocityP = 0.075;
    double velocityI = 0.000;
    double velocityD = 0.0;
    double velocityF = 0.055; // k_f = (PERCENT_POWER X 1023) / OBSERVED_VELOCITY 0.023
    m_pidTuner = new PIDTuner("Launcher", Robot.EnablePIDTuning, velocityP, velocityI, velocityD, velocityF, this::updatePIDF);

    Robot.LogManager.addNumber("Launcher/Speed2", () -> m_ControllerA.getVelocity().getValueAsDouble());
    
  }

  public void ManualLaunch(double power) {
    m_ControllerA.set(power);
    m_ControllerB.set(power);
  }

  public void SetTargetSpeed(double countsPer10ms) {
    // m_ControllerA.set(rpm / Constants.TalonCountsPerRevolution);
    // m_ControllerB.set(rpm / Constants.TalonCountsPerRevolution);
    m_velocityTarget.Slot = 0;
    // m_ControllerA.setControl(m_velocityTarget.withVelocity(countsPer10ms * Constants.LauncherWheelCircumference));
    // m_ControllerB.setControl(m_velocityTarget.withVelocity(countsPer10ms * Constants.LauncherWheelCircumference));
    m_ControllerA.setControl(m_velocityTarget.withVelocity(countsPer10ms));
    m_ControllerB.setControl(m_velocityTarget.withVelocity(countsPer10ms));

  }

  public void spinUpSpeed() {
    // if (DriverStation.isAutonomous()) {
    //   m_ControllerA.set(TalonFXControlMode.Velocity, Constants.DefaultLauncherSpinUpAuto);
    //   m_ControllerB.set(TalonFXControlMode.Velocity, Constants.DefaultLauncherSpinUpAuto);
    // } else {
    //   m_ControllerA.set(TalonFXControlMode.Velocity, Constants.DefaultLauncherSpinUpTeleop);
    //   m_ControllerB.set(TalonFXControlMode.Velocity, Constants.DefaultLauncherSpinUpTeleop);
    // }
    m_ControllerA.set(0);
    m_ControllerB.set(0);
  }

  public void coast() {
    m_ControllerA.set(0);
    m_ControllerB.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Launcher/Speed", m_ControllerA.getVelocity().getValueAsDouble());
    m_pidTuner.tune();
  }

  private void updatePIDF(PIDFValue update) {
    configA.Slot0.kP = (update.P);
    configB.Slot0.kP = (update.P);
    configA.Slot0.kI = (update.I);
    configB.Slot0.kI = (update.I);
    configA.Slot0.kD = (update.D);
    configB.Slot0.kD = (update.D);
    configA.Slot0.kV = (update.F);
    configA.Slot0.kV = (update.F);
    m_ControllerA.getConfigurator().apply(configA);
    m_ControllerB.getConfigurator().apply(configB);
}

  public void MoveHoodUp() {
    m_solenoid.set(Value.kForward);
  }

  public void MoveHoodDown() {
    m_solenoid.set(Value.kReverse);
  }

  public void setLauncherTolerance(double tolerance) {
    m_speedTolerance = tolerance;
  }

  public boolean isAtTargetSpeed(double targetRpm) {
    SmartDashboard.putNumber("Launcher/TargetSpeed", targetRpm);
    var currentRpm = m_ControllerA.getVelocity().getValueAsDouble();
    return currentRpm > targetRpm - m_speedTolerance && currentRpm < targetRpm + m_speedTolerance;
  }
}
