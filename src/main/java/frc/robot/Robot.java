// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.BallSensor;
import frc.robot.Subsystems.ClimbControls;
import frc.robot.Subsystems.DriveTrain;
//import frc.robot.Subsystems.LimeLight;
import frc.robot.Subsystems.Pneumatics;
import frc.robot.Subsystems.ShootParamaters;
import frc.robot.Subsystems.SingulatorInformation;
import frc.robot.commands.AutonomousIntake;
import frc.robot.commands.AutonomousShooting;
//import frc.robot.Subsystems.ShootParamaters;
import frc.robot.commands.Drive;
import frc.robot.commands.Shoot;
import frc.robot.commands.StopAll;
import frc.robot.commands.TankDrive;
import frc.robot.commands.Turn;
import frc.robot.commands.Wait;
import frc.robot.simulation.PoseEstimator;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private OI m_oi = new OI();
  private static final String DEFAULT_AUTO = "Default";
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  
  private DriveTrain m_driveTrain = new DriveTrain(m_oi);
  private ShootParamaters m_ShootParamaters = new ShootParamaters(m_oi);
  private Pneumatics m_pneumatics = new Pneumatics(m_oi);
  private SingulatorInformation m_singulator = new SingulatorInformation(m_oi);
  public PIDController m_shooterController;
  public ClimbControls m_climb = new ClimbControls();
  public BallSensor m_ballz = new BallSensor();

  //private LimeLight m_LimeLight = new LimeLight();
  //private ShootParamaters m_shooter = new ShootParamaters();

   private PoseEstimator m_poseEstimator; // might be null
   private Field2d m_fieldInfo; // might be null

  
  //private MotorControllerGroup spinGroup = new MotorControllerGroup(spin1, spin2);  


  //UsbCamera camera1 = CameraServer.startAutomaticCapture();
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {    

    m_chooser.setDefaultOption("Default Auto", DEFAULT_AUTO);
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
    m_driveTrain.simulationInit();
    m_poseEstimator = new PoseEstimator(m_driveTrain);
    m_fieldInfo = new Field2d();
    SmartDashboard.putData("Field", m_fieldInfo);

  }

  @Override
  public void simulationPeriodic() {
    if (m_poseEstimator != null) {
      m_poseEstimator.periodic();
    }
    if (m_fieldInfo != null && m_poseEstimator != null) {
      m_fieldInfo.setRobotPose(m_poseEstimator.getPose());
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

     //Start of Autonomous Code for Blue Tarmac At Bottom of Field
    // new Drive(m_driveTrain, m_ShootParamaters, 24),
    //   new Wait(1),
    // new AutonomousShooting(m_driveTrain, m_pneumatics, m_ShootParamaters, m_singulator),
    // new Wait(1),
    //   new Turn(m_driveTrain, -82.5),
    //     new Wait(1), 
    //       new Drive(m_driveTrain, m_ShootParamaters, -129.2),
    //         new AutonomousIntake(m_driveTrain, m_pneumatics, m_ShootParamaters, m_singulator)

    // End of Autonomous Code For Blue Tarmac at Bottom Of Field

    //Start of Autonomous Code for Blue Tarmac On The Top Left of The Field
    // new Drive(m_driveTrain, m_ShootParamaters, 24),
     new AutonomousShooting(m_driveTrain, m_pneumatics, m_ShootParamaters, m_singulator),
      new Wait(1),
         //new StopAll(m_driveTrain, m_pneumatics, m_ShootParamaters, m_singulator),
           //new Wait(1),
             new Turn(m_driveTrain, 106),
              new Wait(1),
                new Drive(m_driveTrain, m_ShootParamaters, -88),
                    new AutonomousIntake(m_driveTrain, m_pneumatics, m_ShootParamaters, m_singulator),

    // //End of Autonomous Code for Blue Tarmac on The Top Left Of The Field

    // //Start of Autonomous Code for Red Tarmac On the Bottom Right Of The Field
      new Drive(m_driveTrain, m_ShootParamaters, 24),
        new StopAll(m_driveTrain, m_pneumatics, m_ShootParamaters, m_singulator),
          new Wait(1),
            new AutonomousShooting(m_driveTrain, m_pneumatics, m_ShootParamaters, m_singulator),
              new Wait(1),
         //new StopAll(m_driveTrain, m_pneumatics, m_ShootParamaters, m_singulator),
           //new Wait(1),
                  new Turn(m_driveTrain, 74),
                    new Wait(1),
                      new Drive(m_driveTrain, m_ShootParamaters, -88)
    // //End of Autonomous Code For Red Tarmac On The Bottom Right Of The Field

    // //Start of Autonomous Code For Red Tarmac On The Top Right Of The Field
    //     //new Shoot(m_ShootParamaters, m_driveTrain, m_shooterController, m_singulator),
    //     new Wait(1),
    //       new StopAll(m_driveTrain, m_pneumatics, m_ShootParamaters, m_singulator),
    //        new Wait(1),
    //         new Turn(m_driveTrain, 106),
    //           new Wait(1),
    //           new Drive(m_driveTrain, m_ShootParamaters, -88)
    //End Of Autonomous Code For Red Tarmac On The Top Right Of The Field














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

      CommandBase commands = new TankDrive(m_oi, m_driveTrain,m_pneumatics, m_ShootParamaters, m_singulator, m_climb, m_ballz);
      
      CommandScheduler.getInstance().schedule(commands);
     }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    CommandScheduler.getInstance().run();

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
