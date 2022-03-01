// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.ShootParamaters;

public class Shoot extends CommandBase {
  

private double expiretime;
private double timeout;
public ShootParamaters M_shoot;
public PIDController m_shooterController;




  /** Creates a new Shoot. */
  public Shoot(ShootParamaters Shootparamaters, DriveTrain Drivetrain, PIDController m_Shootercontroller) {
  M_shoot = Shootparamaters;
  m_shooterController = m_Shootercontroller;

  timeout = 5;

  addRequirements(M_shoot);

    // Use addRequirements() here to declare subsystem dependencies.
  }





  public void CalculatedShootSpeed(ShootParamaters M_Shoot) {

    double targetRpm = M_Shoot.computeShooterVelocity();
    m_shooterController.setSetpoint(targetRpm);
    double measuredRpm = M_Shoot.ShooterLeft.getSelectedSensorVelocity();
    double outputValue =
     m_shooterController.calculate(measuredRpm);

     outputValue = Math.max(-1, Math.min(1, outputValue));

    if(measuredRpm <= targetRpm + 50 && measuredRpm >= targetRpm - 50){

      SmartDashboard.putBoolean("ShootReady", true);
      
    }
    else
    {
      SmartDashboard.putBoolean("ShootReady", false);
    }





  }
 
  public double computeShooterVelocity() {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    // acceration due to gravity
    final double G = 9.8;
    // ratio between linerar and angular velocity
    final double wheelToBall = 2;
    // Height for testing
    final double TestHeight = 1.5494;
    // distance from the ground to the Target
    final double TargetHeight = 2.64;
    // distance from ground to limelight
    final double LimeToGround = .679;
    // distance from the limelight to the shooter
    final double limeToShooter = 0;
    // angle of the ball shooter in degrees
    double shootangleD = 55;
    // angle between middle of limelight and target in degrees
    double limeangleD = ty + 35;
    // distance between ballshooter and target
    double distance = (((TestHeight - LimeToGround) / Math.tan(Math.toRadians((limeangleD)))) + limeToShooter) * 2;

    // speed of the ball needed to reach the target
    double ballspeed = Math.sqrt((distance * G) / Math.sin(2 * (Math.toRadians(shootangleD))));
    // final velocity of the ball
    double vf = wheelToBall * ballspeed;
    // speed of the wheel needed to accelerate the ball
    double wheelspeed = (vf) + 1 + (1 + .4) / ((2 * .8) + (671.31 / 270.00));
    // rpm of the wheel
    double rpm = (wheelspeed * 60) / (.0508 * 2 * Math.PI);

    // SmartDashboard.putNumber("Limelight angle", limeangleD);
    // SmartDashboard.putNumber("RPM", rpm);
    // SmartDashboard.putNumber("rpm percent", percent * 100);
    // SmartDashboard.putNumber("wheel speed", wheelspeed);
    // SmartDashboard.putNumber("ball speed", ballspeed);
    // SmartDashboard.putNumber("TY", Math.abs(ty));
    // SmartDashboard.putNumber("distance to target", distance);

    // TODO: boost output by 15% of max RPM (fudge factor)
    final double ENCODER_TICKS_PER_REVOLUTION = 2048;
    final double TENTHS_OF_A_SECOND_PER_MINUTE = 600;

    return (rpm * (ENCODER_TICKS_PER_REVOLUTION / TENTHS_OF_A_SECOND_PER_MINUTE));
  }





protected void startTime()
{




//expiretime = timeSinceInitialized() + timeout;
}

public void timeSinceInitialized()
{


}
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  
    




  //  return (timeSinceInitialized() >= expiretime);


  
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
