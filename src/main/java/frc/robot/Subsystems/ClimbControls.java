// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.RobotMap;

public class ClimbControls extends SubsystemBase {
  TalonSRX talon = new TalonSRX(11);
  TalonSRX talon2 = new TalonSRX(12);

  public OI m_oi = new OI();

  public static final int kCPR = 4096;

  //private CANSparkMax ClimbMotor1 = new CANSparkMax(RobotMap.ClimbMotor1, MotorType.kBrushed);
  //private CANSparkMax ClimbMotor2 = new CANSparkMax(RobotMap.ClimbMotor2, MotorType.kBrushed);

  


  /** Creates a new ClimbControls. */
  public ClimbControls() {

    talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    talon.getSelectedSensorPosition();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void ForwardClimbSpeed() {
    talon2.setInverted(true);
    
    if (m_oi.getRightBumper()) {
      talon.set(ControlMode.PercentOutput, .3);
      talon2.set(ControlMode.PercentOutput, .3);

   talon.getSelectedSensorPosition();
   talon2.getSelectedSensorPosition();


    SmartDashboard.putNumber("Encoder Position", talon.getSelectedSensorPosition());
    SmartDashboard.putNumber("Encoder Position2", talon2.getSelectedSensorPosition());

    }
    else if (m_oi.getLeftBumper()){
      talon.set(ControlMode.PercentOutput, -.2);
      talon2.set(ControlMode.PercentOutput, -.2);
    }
    else {

      talon.set(ControlMode.PercentOutput, 0);
      talon2.set(ControlMode.PercentOutput, 0);
    }
  }


  public void ReadClimbEncoders() {
    
    

    //SmartDashboard.putNumber("Encoder Position2", m_encoder2.getPosition());

  } 


  public void ResetEncoders(){
   talon.setSelectedSensorPosition(0);
   talon2.setSelectedSensorPosition(0);


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
