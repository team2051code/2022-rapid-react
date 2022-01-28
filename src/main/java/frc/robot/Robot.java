// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveJoystick;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  
  //CANSparkMax spin1 = new CANSparkMax(5, MotorType.kBrushless);
  //CANSparkMax spin2 = new CANSparkMax(5, MotorType.kBrushless);


  private WPI_TalonFX talonLeft = new WPI_TalonFX(1);
  private WPI_TalonFX talonRight = new WPI_TalonFX(2);

  private DriveTrain m_DriveTrain = new DriveTrain();
  
  //private MotorControllerGroup spinGroup = new MotorControllerGroup(spin1, spin2);  

  XboxController controller = new XboxController(0);
  UsbCamera camera1 = CameraServer.startAutomaticCapture();
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    talonLeft.set(ControlMode.PercentOutput, 0);
    talonRight.set(ControlMode.PercentOutput, 0);
    talonLeft.follow(talonRight);
    System.out.println("Line");

    
    talonRight.setInverted(true);
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    final CommandBase joystickCommand = new DriveJoystick(m_DriveTrain, controller);
    m_DriveTrain.setDefaultCommand(joystickCommand);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  

  
  }


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
    SequentialCommandGroup commands = new SequentialCommandGroup(
    new Drive(m_DriveTrain)
    );
    CommandScheduler.getInstance().schedule(commands);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    CommandScheduler.getInstance().run();

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    UsbCamera camera1 = CameraServer.startAutomaticCapture();
    UsbCamera camera2 = CameraServer.startAutomaticCapture();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    //Drive.tankDrive(controller.getRawAxis(1), controller.getRawAxis(5));

    // //change this to false to run autonomous aiming code
    // boolean manualAim = true;
    // if(manualAim)
    // {
    //   double spinSpeed = 0;
    //   spinSpeed += xbox.getLeftBumper() ? 0.6 : 0;
    //   spinSpeed += xbox.getRightBumper() ? -0.6 : 0;
    //   spinSpeed += xbox.getLeftTriggerAxis() / 5;
    //   spinSpeed += xbox.getRightTriggerAxis() / 5 * -1;
    //   //spinGroup.set(spinSpeed);

    //   double wheelSpeed = 0;
    //   wheelSpeed += xbox.getRawAxis(1) * -1 >= 0.1 ? xbox.getRawAxis(1) * -0.9 : 0;
    //   wheelSpeed += xbox.getRawAxis(5) * -1 >= 0.1 ? xbox.getRawAxis(5) / -10 : 0;
    //   //talonLeft.set(wheelSpeed);
    //   //talonRight.set(wheelSpeed);
    // }

    // Right.set(xbox.getRawAxis(1) * .1);
    // RightFollow.set(xbox.getRawAxis(1) * .1);
    // Left.set(xbox.getRawAxis(1) * .1);
    // LeftFollow.set(xbox.getRawAxis(1) * .1);


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
