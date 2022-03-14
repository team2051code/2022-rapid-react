// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallSensor extends SubsystemBase {

  private DigitalInput m_closeIntake; 
  private DigitalInput m_closeShooter; 

  
  /** Creates a new BallSensor. */
  public BallSensor() {

  m_closeIntake = new DigitalInput(0);
  m_closeShooter = new DigitalInput(1);

  }

  @Override
  public void periodic() {
    if (!m_closeIntake.get()) {
      
       SmartDashboard.putString("Ball?", "OneBall");
     } else {
      SmartDashboard.putString("Ball?", "NoBall");
     }

    // if (!CloseShooter.get()) {

    //   SmartDashboard.putString("Secondball?", "SecondBall");
    // } else {
    //   SmartDashboard.putString("SecondBall?", "NoSecondBall");
    // }

    // if (!CloseIntake.get() && !CloseShooter.get()) {
    //   SmartDashboard.putString("Bothballs?", "BothBalls");
    // } else {
    //   SmartDashboard.putString("BothBalls?", "NoBothBalls");
    // }
  }
    // This method will be called once per scheduler run
  

  public void ReadLineSensors() {

    m_closeIntake.get();
    m_closeShooter.get();
    //System.out.println(CloseShooter.get());
  }

  public boolean ReadFirstSensor(){

    return !m_closeIntake.get();
  }

  public boolean ReadSecondSensor(){
    return !m_closeShooter.get();
  }

  
public void PrintSensor(){
SmartDashboard.putBoolean("ReadFirstSensor", ReadFirstSensor());
SmartDashboard.putBoolean("ReadSecondSensor", ReadSecondSensor());
// System.out.println(ReadFirstSensor());
// System.out.print(ReadSecondSensor());


}
}