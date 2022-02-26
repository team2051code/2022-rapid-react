// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class Turn extends CommandBase {
  private DriveTrain M_DriveTrain;
  private double M_TurnDegrees;
  /** Creates a new Turn. 
   * @param M_Drivetrain */
  public Turn(DriveTrain driveTrain, double TurnDegrees) {

  M_DriveTrain = driveTrain;

  M_TurnDegrees = TurnDegrees;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  M_DriveTrain.ResetGyro();    


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(M_TurnDegrees > 0)
    {
      M_DriveTrain.tankDrive(-.5, .5);
    }
    else{
      M_DriveTrain.tankDrive(.5, -.5);
    }




  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    M_DriveTrain.tankDrive(0, 0);


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  System.out.println(M_DriveTrain.getGyroAngleDegrees());

    if(M_TurnDegrees < 0){
        
      return (M_DriveTrain.getGyroAngleDegrees() <= M_TurnDegrees);
      
      }

  return (M_DriveTrain.getGyroAngleDegrees() >= M_TurnDegrees);
  
  }
}
