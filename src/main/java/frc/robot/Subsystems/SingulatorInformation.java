// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.RobotMap;

public class SingulatorInformation extends SubsystemBase {

  
  CANSparkMax SingulatorMotor2 = new CANSparkMax(RobotMap.SingulatorMotor2, MotorType.kBrushless);
  public OI m_oi = new OI();

  /** Creates a new SingulatorInformation. */
  public SingulatorInformation() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void SingulatorSpeed(double Speed)
  {
    
    SingulatorMotor2.set(Speed);

  }

  public void SetSingulatorSpeed()
  {
    SingulatorMotor2.setInverted(true);

    if(m_oi.GetLeftBumper2())
    {
        SingulatorMotor2.set(.80);
    }
    else{
        SingulatorMotor2.set(0);
    }
  }
}
