// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.ShootParamaters;

public class Shoot extends CommandBase {
    public OI m_oi = new OI();
    public DriveTrain Shooter1;
    public DriveTrain Shooter2;
    public DriveTrain setShootSpeed;
  public DriveTrain M_DriveTrain;
  public ShootParamaters M_Shoot;

  public Shoot(ShootParamaters shootParamaters, DriveTrain Drivetrain) {
    addRequirements(M_Shoot, M_DriveTrain);


    }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    System.out.println(m_oi.GetAButton());
    System.out.println(setShootSpeed);


    // boolean EnableShoot = m_oi.GetAButton(RobotMap.AButton);
        if(m_oi.GetAButton())
    {
        Shooter1.setShootSpeed(1);
        Shooter2.setShootSpeed(1);
    }
    else
    {
    Shooter1.setShootSpeed(0);
    Shooter2.setShootSpeed(0);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    

    

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
