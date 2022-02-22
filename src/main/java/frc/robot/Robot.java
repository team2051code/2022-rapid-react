// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.DriveTrain;
//import frc.robot.Subsystems.LimeLight;
import frc.robot.Subsystems.Pneumatics;
import frc.robot.Subsystems.ShootParamaters;
import frc.robot.Subsystems.SingulatorInformation;
//import frc.robot.Subsystems.ShootParamaters;
import frc.robot.commands.Drive;
import frc.robot.commands.TankDrive;
import frc.robot.simulation.PoseEstimator;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public OI m_oi;
  private static final String kDefaultAuto = "Default";
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  
  private DriveTrain m_DriveTrain = new DriveTrain();
  private ShootParamaters m_ShootParamaters = new ShootParamaters();
  private Pneumatics m_Pneumatics = new Pneumatics();
  private SingulatorInformation m_Singulator = new SingulatorInformation();
  //private LimeLight m_LimeLight = new LimeLight();
  //private ShootParamaters m_shooter = new ShootParamaters();

   private PoseEstimator poseEstimator; // might be null
   private Field2d fieldInfo; // might be null

  
  //private MotorControllerGroup spinGroup = new MotorControllerGroup(spin1, spin2);  


  //UsbCamera camera1 = CameraServer.startAutomaticCapture();
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {    

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
  public void robotPeriodic() {
  }

  @Override
  public void simulationInit() {
    m_DriveTrain.simulationInit();
    poseEstimator = new PoseEstimator(m_DriveTrain);
    fieldInfo = new Field2d();
    SmartDashboard.putData("Field", fieldInfo);

  }

  @Override
  public void simulationPeriodic() {
    if (poseEstimator != null) {
      poseEstimator.periodic();
    }
    if (fieldInfo != null && poseEstimator != null) {
      fieldInfo.setRobotPose(poseEstimator.getPose());
    }
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
  public void autonomousPeriodic() {

    CommandScheduler.getInstance().run();

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    //UsbCamera camera1 = CameraServer.startAutomaticCapture();
    //camera1.setResolution(256, 144);
    //camera1.setFPS(30);

   // UsbCamera camera2 = CameraServer.startAutomaticCapture();

      CommandBase commands = new TankDrive(m_DriveTrain,m_Pneumatics, m_ShootParamaters, m_Singulator);
      
      CommandScheduler.getInstance().schedule(commands);
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
