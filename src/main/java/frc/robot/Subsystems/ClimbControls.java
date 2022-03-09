// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.RobotMap;

public class ClimbControls extends SubsystemBase {
  private CANSparkMax ClimbMotor1 = new CANSparkMax(RobotMap.ClimbMotor1, MotorType.kBrushed);
  private CANSparkMax ClimbMotor2 = new CANSparkMax(RobotMap.ClimbMotor2, MotorType.kBrushed);
  public OI m_oi = new OI();

  //private CANSparkMax ClimbMotor1 = new CANSparkMax(RobotMap.ClimbMotor1, MotorType.kBrushed);
  //private CANSparkMax ClimbMotor2 = new CANSparkMax(RobotMap.ClimbMotor2, MotorType.kBrushed);

  private RelativeEncoder m_encoder;
  private RelativeEncoder m_encoder2;


  /** Creates a new ClimbControls. */
  public ClimbControls() {

    ClimbMotor1.getAlternateEncoder(Type.kQuadrature, 4096);
    m_encoder = ClimbMotor1.getAlternateEncoder(Type.kQuadrature, 4096);
    ClimbMotor2.getAlternateEncoder(Type.kQuadrature, 4096);
    m_encoder2 = ClimbMotor2.getAlternateEncoder(Type.kQuadrature, 4096);
    m_encoder.getPosition();
    m_encoder2.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void ForwardClimbSpeed() {
    ClimbMotor2.setInverted(true);
    
    if (m_oi.getRightBumper()) {
      ClimbMotor1.set(.2);
      ClimbMotor2.set(.2);

    SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
    SmartDashboard.putNumber("Encoder Position2", m_encoder2.getPosition());

    }
    else if (m_oi.getLeftBumper()){
      ClimbMotor1.set(-.2);
      ClimbMotor2.set(-.2);
    }
    else {

      ClimbMotor1.set(0);
      ClimbMotor2.set(0);
    }
  }


  public void ReadClimbEncoders() {
    //m_encoder.getPosition();
    //m_encoder2.getPosition();
    

    //SmartDashboard.putNumber("Encoder Position2", m_encoder2.getPosition());

  } 


  public void ResetEncoders(){


  }

  // public void ForwardClimbSpeed() {
  //   if (m_oi.getRightBumper()) {
  //     ClimbMotor1.set(.5);
  //     ClimbMotor2.set(.5);
  //   } else {

  //     ClimbMotor1.set(0);
  //     ClimbMotor2.set(0);
  //   }

  //}

  // public void BackwardsClimbSpeed() {
  // if (m_oi.getLeftBumper()) {
  // ClimbMotor1.set(-.5);
  // ClimbMotor2.set(-.5);
  // } else {

  // ClimbMotor1.set(0);
  // ClimbMotor2.set(0);

  // }

  // }

}
