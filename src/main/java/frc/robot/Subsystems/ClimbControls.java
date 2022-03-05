// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.RobotMap;

public class ClimbControls extends SubsystemBase {
  public OI m_oi = new OI();

  private CANSparkMax ClimbMotor1 = new CANSparkMax(RobotMap.ClimbMotor1, MotorType.kBrushed);
  private CANSparkMax ClimbMotor2 = new CANSparkMax(RobotMap.ClimbMotor2, MotorType.kBrushed);


  /** Creates a new ClimbControls. */

  public ClimbControls() {}
  
  public void SetClimbSpeed(){
    {
    if(m_oi.getRightBumper())
    {
      ClimbMotor1.set(.5);
      ClimbMotor2.set(.5);
    }else{
      ClimbMotor1.set(0);
      ClimbMotor2.set(0);
      }
    }
  }
}
