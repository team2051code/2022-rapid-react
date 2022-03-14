// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Pneumatics;



public class RunIntake extends CommandBase {
  /** Creates a new RunIntake. */
  private DriveTrain m_driveTrain;
  private Pneumatics m_pneumatics;


  public RunIntake(DriveTrain driveTrain, Pneumatics pneumatics) {

    m_pneumatics = pneumatics;
    m_driveTrain = driveTrain;

    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_driveTrain.SetAutonomousIntake();
    m_pneumatics.AutonomousForward();

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
