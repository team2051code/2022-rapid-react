// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private XboxController xbox = new XboxController(0);

  CANSparkMax Right1 = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax Right2 = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax Left1 = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax Left2 = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax spin1 = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax spin2 = new CANSparkMax(5, MotorType.kBrushless);

  private WPI_TalonFX talonLeft = new WPI_TalonFX(1);
  private WPI_TalonFX talonRight = new WPI_TalonFX(2);

  private MotorControllerGroup LeftSide = new MotorControllerGroup(Left1, Left2);
  private MotorControllerGroup RightSide = new MotorControllerGroup(Right1, Right2);
  private MotorControllerGroup spinGroup = new MotorControllerGroup(spin1, spin2);

  private RelativeEncoder Encoder;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    Left1.setInverted(true);
    Left2.setInverted(true);
    talonRight.setInverted(true);
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    //Distance from the middle of the starting outline to the edge of the starting outline
    double DistanceFromMiddleStartingAreaToEdgeOfStartingArea = 50;
    //Distance from The Edge of the starting outline to the nearest blue ball
    double DistanceFromEdgeOfStartingAreaToBlueBall = 100;
    //Addition of the distances to group them together
    double DesiredDistance = (DistanceFromEdgeOfStartingAreaToBlueBall + DistanceFromMiddleStartingAreaToEdgeOfStartingArea);
    //Sets the distance to zero
    double DistanceInInches = 0;


    //While our desired distance is greater than our current distance keep running the drive motors
    while(DesiredDistance > DistanceInInches)
  {

    //sets the motors to use 30% of their power
    LeftSide.set(.3);
    RightSide.set(.3);

    
final double MotorTicks = (42.0 * 10.75);
//Math to calculate the number of motor pulses based on our rotations

final double Pulses = (MotorTicks / 2.0 * Math.PI) * 3;
//Math to calculate the current distance of the motor using the previous equation
DistanceInInches = (Encoder.getPosition() / Pulses);
SmartDashboard.putNumber("DistanceInInches", DistanceInInches);

   }
  }
  

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //change this to false to run autonomous aiming code
    boolean manualAim = true;
    if(manualAim)
    {
      double spinSpeed = 0;
      spinSpeed += xbox.getLeftBumper() ? 0.6 : 0;
      spinSpeed += xbox.getRightBumper() ? -0.6 : 0;
      spinSpeed += xbox.getLeftTriggerAxis() / 5;
      spinSpeed += xbox.getRightTriggerAxis() / 5 * -1;
      spinGroup.set(spinSpeed);

      double wheelSpeed = 0;
      wheelSpeed += xbox.getRawAxis(1) * -1 >= 0.1 ? xbox.getRawAxis(1) * -0.9 : 0;
      wheelSpeed += xbox.getRawAxis(5) * -1 >= 0.1 ? xbox.getRawAxis(5) / -10 : 0;
      talonLeft.set(wheelSpeed);
      talonRight.set(wheelSpeed);
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
  public void testPeriodic() {}
}
