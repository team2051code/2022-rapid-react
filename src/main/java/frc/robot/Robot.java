// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.opencv.calib3d.StereoBM;

import edu.wpi.first.wpilibj.GenericHID.Hand;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.*;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  //defines the motor controllers for the Talon FX using their CAN ids
  public final WPI_TalonFX LeftMotor1 = new WPI_TalonFX(1);
  public final WPI_TalonFX LeftMotor2 = new WPI_TalonFX(2);
  public final WPI_TalonFX RightMotor1 = new WPI_TalonFX(3);
  public final WPI_TalonFX RightMotor2 = new WPI_TalonFX(4);

  public final SpeedControllerGroup LeftSide = new SpeedControllerGroup(LeftMotor1, LeftMotor2);
  public final SpeedControllerGroup RightSide = new SpeedControllerGroup(RightMotor1, RightMotor2);


  //Creates a new controller so the Steer and Drive Commands Can Be Created
  public final XboxController Controller = new XboxController(1);

  //Creates a DifferentDrive for the Drive and Steer Commands to use
  public final DifferentialDrive m_Drive = new DifferentialDrive(LeftSide, RightSide);

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //presets the limelight to not have a target until robot is told to
  private boolean m_LimelightHasValidTarget = false;

  //Doesnt allow the robot to move until told to
  private double DriveCommand = 0;
  private double SteerCommand = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);


  }


  @Override
  public void robotPeriodic() {}



  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  
    }


  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}



  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    Update_Limelight_Tracking();

    double Steer = Controller.getY();
    double Drive = -Controller.getX();
    boolean Go = Controller.getAButton();

    Steer *= 0.50;
    Drive *= 0.50;    
    
    // if the A button is pressed Run This Command
    if (Go)
    {
      //Only Starts if A button is pressed and Limelight has a Valid Target
      if(m_LimelightHasValidTarget){
        //Robot drives according to the drive and steer command
          m_Drive.tankDrive(DriveCommand, SteerCommand);
      }
        else
        {
          m_Drive.tankDrive(0, 0);
      }
    }
    else
    {
      m_Drive.tankDrive(Steer, Drive);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    
  }
  public void Update_Limelight_Tracking() {

    final double Steer_K = 0.03;        //How Fast The Robot Turns
    final double Drive_K = 0.1;         //How Fast The Robot Drives to The Robot
    final double HowFarAway = 15;       //Sets the Distance in Which The Robot Will Stop Before It Reaches A Target
    final double SpeedLimit = 0.7;      //Limits the Robot's Speed
  
  
    //This grabs the tv, tx, ty, and ta values from the limelight measuring value table
    //The tv Value Is Either 1 or 0 Meaning the Limelight has a Target(1) Or Not(0)
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  
    if (tv < 1) {
  
      m_LimelightHasValidTarget = false;
      DriveCommand = 0;
      SteerCommand = 0;
      return;
     }


     m_LimelightHasValidTarget = true;

     //The Equation to figure out the speed of the Robot
     double steer_cmd = tx * Steer_K;


     double drive_cmd = (HowFarAway - ta) * Drive_K;

     //uses the previously define SteerCommand Command to tie it with the steer_cmd Speed Equation
     SteerCommand = steer_cmd;

      //If the robot is driving too fast
    if(DriveCommand > SpeedLimit){

      //Sets the motor speed to the speedlimit value
        DriveCommand = SpeedLimit;
      }
      
      DriveCommand = drive_cmd;

   }
}
  
 
