// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class Turn extends CommandBase {
  private DriveTrain m_driveTrain;
  private double m_turnDegrees;
  /** Creates a new Turn. 
   * @param drivetrain */
  public Turn(DriveTrain driveTrain, double turnDegrees) {

  m_driveTrain = driveTrain;

  m_turnDegrees = turnDegrees;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  m_driveTrain.resetGyro();    


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_turnDegrees > 0)
    {
      m_driveTrain.tankDrive(-.5, .5);
    }
    else{
      m_driveTrain.tankDrive(.5, -.5);
    }




  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_driveTrain.tankDrive(0, 0);


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  System.out.println(m_driveTrain.getGyroAngleDegrees());

    if(m_turnDegrees < 0){
        
      return (m_driveTrain.getGyroAngleDegrees() <= m_turnDegrees);
      
      }

  return (m_driveTrain.getGyroAngleDegrees() >= m_turnDegrees);
  
  }
}
