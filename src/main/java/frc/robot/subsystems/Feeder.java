// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  private TalonFX m_upperFeeder;
  private TalonFX m_lowerFeeder;
  // private TalonFX johngetnegativestars = new TalonFX(40);
  // private TalonFX johnget2negativestars = new TalonFX(41);
  // private TalonFX johnget3negativestars = new TalonFX(42);
  // private TalonFX johnget4negativestars = new TalonFX(43);
  // private TalonFX johnget5negativestars = new TalonFX(44);
  // private TalonFX johnget6negativestars = new TalonFX(45);
  // private TalonFX johnget7negativestars = new TalonFX(46);
  // private TalonFX johnget8negativestars = new TalonFX(47);
  // private TalonFX johnget9negativestars = new TalonFX(48);
  // private TalonFX johnget10negativestars = new TalonFX(49);
  // private TalonFX johnget11negativestars = new TalonFX(50);
  // private TalonFX johnget12negativestars = new TalonFX(51);
  // private TalonFX johnget13negativestars = new TalonFX(52);
  // private TalonFX johnget14negativestars = new TalonFX(53);
  private DigitalInput m_beamSensorTop, m_beamSensorMiddle;

  public enum FeederMode {
    DEFAULT, INTAKE, LAUNCH_LOW_BUMPER, LAUNCH_HIGH_BUMPER, LAUNCH_CAMERA, OUTPUT, BOTTOM_ONLY
  } 

  private FeederMode m_feederMode = FeederMode.DEFAULT;

  /** Creates a new Feeder. */
  public Feeder() {
    m_upperFeeder = new TalonFX(Constants.UpperFeeder);
    m_lowerFeeder = new TalonFX(Constants.LowerFeeder);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.2;

    m_upperFeeder.getConfigurator().apply(config);
    m_lowerFeeder.getConfigurator().apply(config);

    m_beamSensorTop = new DigitalInput(Constants.FeederBeamSensorTop);
    m_beamSensorMiddle = new DigitalInput(Constants.FeederBeamSensorMiddle);
  }

  public void setFeederMode(FeederMode feederMode) {
    m_feederMode = feederMode;
  }

  public FeederMode getFeederMode() {
    return m_feederMode;
  }

  public void ManualFeed(double powerTop, double powerBottom) {
    m_upperFeeder.set(powerTop);
    m_lowerFeeder.set(powerBottom);
  }

  public void Stop() {
    m_upperFeeder.set(0.0);
    m_lowerFeeder.set(0.0);
  }

  public void Both() {
    m_upperFeeder.set(0.0);
    m_lowerFeeder.set(0.3);
  }

  public void Bottom() {
    m_upperFeeder.set(0.0);
    m_lowerFeeder.set(0.3);
  }

  public boolean IsBallAtTopFeeder() {
    return !m_beamSensorTop.get();
  }

  public boolean IsBallAtMiddleFeeder() {
    return !m_beamSensorMiddle.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Feeder/Top", IsBallAtTopFeeder());
    SmartDashboard.putBoolean("Feeder/Middle", IsBallAtMiddleFeeder());
  }
}
