// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallSensor extends SubsystemBase {

  DigitalInput CloseIntake = new DigitalInput(0);
  DigitalInput CloseShooter = new DigitalInput(1);

  /** Creates a new BallSensor. */
  public BallSensor() {
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

  public void ReadLineSensors() {

    CloseIntake.get();
    CloseShooter.get();
    System.out.println(CloseShooter.get());
  }
  public void CurrentSensorState() {



    if (CloseIntake.get()) {
      
      SmartDashboard.putString("Ball?", "OneBall");
    } else {
      SmartDashboard.putString("Ball?", "NoBall");
    }

    if (CloseShooter.get()) {

      SmartDashboard.putString("Secondball?", "SecondBall");
    } else {
      SmartDashboard.putString("SecondBall?", "NoSecondBall");
    }

    if (CloseIntake.get() && CloseShooter.get()) {
      SmartDashboard.putString("Bothballs?", "BothBalls");
    } else {
      SmartDashboard.putString("BothBalls?", "NoBothBalls");
    }
  }

}
