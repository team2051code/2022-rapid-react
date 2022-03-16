// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.RobotMap;

public class ClimbControls extends SubsystemBase {

  public OI m_oi;

  private WPI_TalonSRX ClimbMotor1 = new WPI_TalonSRX(RobotMap.ClimbMotor1);
  private WPI_TalonSRX ClimbMotor2 = new WPI_TalonSRX(RobotMap.ClimbMotor2);


  /** Creates a new ClimbControls. */
  public ClimbControls(OI oi) {
        m_oi = oi;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void ForwardClimbSpeed() {
    ClimbMotor2.setInverted(true);
    
    if (m_oi.getRightBumper()) {
      ClimbMotor1.set(ControlMode.PercentOutput, .37);
      ClimbMotor2.set(ControlMode.PercentOutput, .4);
    }
    else if (m_oi.getLeftBumper()){
      ClimbMotor1.set(ControlMode.PercentOutput, -.17);
      ClimbMotor2.set(ControlMode.PercentOutput, -.2);
    }
    else {

      ClimbMotor1.set(ControlMode.PercentOutput, 0);
      ClimbMotor2.set(ControlMode.PercentOutput, 0);
    }
  }


  public void ReadClimbEncoders() {
    //m_encoder.getPosition();
    //m_encoder2.getPosition();
    

    //SmartDashboard.putNumber("Encoder Position2", m_encoder2.getPosition());

  } 


  public void ResetEncoders(){


  }

  

  public void BackwardsClimbSpeed() {
  if (m_oi.getLeftBumper()) {
  ClimbMotor1.set(ControlMode.PercentOutput, -.5);
  ClimbMotor2.set(ControlMode.PercentOutput, -.5);
  } else {

  ClimbMotor1.set(ControlMode.PercentOutput, 0);
  ClimbMotor2.set(ControlMode.PercentOutput, 0);

  }

  }

}
