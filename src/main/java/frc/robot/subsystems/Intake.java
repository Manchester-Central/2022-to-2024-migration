// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private TalonFX m_intakeController;
  private DoubleSolenoid m_solenoid;
  /** Creates a new Intake. */
  public Intake() {
    m_intakeController = new TalonFX(Constants.Intake);

    // Create and set config options
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode =  NeutralModeValue.Coast;
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.4;

    // Apply config options
    m_intakeController.getConfigurator().apply(config);

    m_solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.IntakeSolenoidForward, Constants.IntakeSolenoidReverse);
  }

  public void ManualIntake(double power) {
    if (m_solenoid.get() == Value.kForward) {
      power = 0;
    }
    m_intakeController.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void MoveIntakeUp() {
    m_solenoid.set(Value.kForward);
  }

  public void MoveIntakeDown() {
    m_solenoid.set(Value.kReverse);
  }
}
