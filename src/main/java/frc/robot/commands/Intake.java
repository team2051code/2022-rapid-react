// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Subsystems.DriveTrain;

public class Intake extends CommandBase {
  public OI m_oi = new OI();
  public DriveTrain IntakeMethod;
  public DriveTrain M_DriveTrain;
  public DriveTrain SetIntakeSpeed;
  
  /** Creates a new Intake. */
  public Intake(DriveTrain Drivetrain) {
  M_DriveTrain = Drivetrain;
  addRequirements(M_DriveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
   {
   
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_oi.GetBButton())
    {
    IntakeMethod.SetIntakeSpeed(50);
    }
    else
    {
    IntakeMethod.SetIntakeSpeed(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
