// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.AutoOutput;
import frc.robot.subsystems.Intake;

public class IntakeDefault extends Command {
  private Intake m_intake;
  /** Creates a new IntakeDefault. */
  public IntakeDefault(Intake intake) {
    m_intake = intake;
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(DriverStation.isAutonomousEnabled()) {
      m_intake.MoveIntakeDown();

      if(AutoOutput.isIntakeReversed) {
        m_intake.ManualIntake(-1.0);
      } else {
        m_intake.ManualIntake(1.0);
      }
    }
    else {
      m_intake.MoveIntakeUp();
      m_intake.ManualIntake(0);
    }
    //m_intake.MoveIntakeDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
