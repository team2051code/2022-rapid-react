// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Wait extends CommandBase {

  private double m_wait;
  Timer m_timer = new Timer();

  // Timer m_timer = new Timer();

  // private double TargetTime;
  // private double AddedTime;

  /** Creates a new Wait. */
  public Wait(double wait) {
    m_wait = wait;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.

  // public double Timer(double time)
  // {
  // return m_timer.get();
  // }

  @Override
  public void initialize() {
    m_timer.start();
  }

  // // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  SmartDashboard.putNumber("CurrentTime", m_timer.get());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.reset();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentTime = m_timer.get();

    
    return currentTime >= m_wait;

  }
}
