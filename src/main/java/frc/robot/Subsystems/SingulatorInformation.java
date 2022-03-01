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

  
  CANSparkMax m_singulatorMotor2 = new CANSparkMax(RobotMap.SINGULATOR_MOTOR_2, MotorType.kBrushless);
  public OI m_oi = new OI();

  /** Creates a new SingulatorInformation. */
  public SingulatorInformation() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void singulatorSpeed(double speed)
  {
    
    m_singulatorMotor2.set(speed);

  }

  public void setSingulatorSpeed()
  {
    m_singulatorMotor2.setInverted(true);

    if(m_oi.getLeftBumper2())
    {
        m_singulatorMotor2.set(.80);
    }
    else{
        m_singulatorMotor2.set(0);
    }
  }
}
