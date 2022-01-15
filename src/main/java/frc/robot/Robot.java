package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private CANSparkMax shooter;
  private CANEncoder Encoder;
  //Defines the motor controllers
  private WPI_VictorSPX m_Left1 = new WPI_VictorSPX(4);
  private WPI_VictorSPX m_Left2 = new WPI_VictorSPX(3);
  private WPI_VictorSPX m_Right3 = new WPI_VictorSPX(2);
  private WPI_VictorSPX m_Right4 = new WPI_VictorSPX(1);

 //groups the seperate sides of the robot into groups
  private SpeedControllerGroup LeftMotors = new SpeedControllerGroup(m_Left1, m_Left2);
  private SpeedControllerGroup RightMotors = new SpeedControllerGroup(m_Right3, m_Right4);

  //creates a drive train using the previously defined SpeedControllerGroups
  private DifferentialDrive m_Drive = new DifferentialDrive(LeftMotors,RightMotors);

  //creates a new Controller to control the robot
  private XboxController m_Controller = new XboxController(0);

  
  private boolean m_LimelightHasValidTarget = false;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
        

        shooter = new CANSparkMax(5, MotorType.kBrushless);

        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);
        
        shooter.restoreFactoryDefaults();

        Encoder = shooter.getEncoder();
        SmartDashboard.putNumber("ProcessVariable", Encoder.getPosition());
        SmartDashboard.putNumber("SetPoint", Encoder.getCountsPerRevolution());


  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

  
    
  }
 

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
        m_chooser.getSelected();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
      
        m_Drive.tankDrive(m_Controller.getRawAxis(1), m_Controller.getRawAxis(5));
        

        Update_Limelight_Tracking();

        shooter.restoreFactoryDefaults();
        boolean shootery = m_Controller.getYButton();
        double steer = m_Controller.getX(Hand.kRight);
        double drive = -m_Controller.getY(Hand.kLeft);
        boolean auto = m_Controller.getAButton();
        boolean shoot = m_Controller.getBButton();
        boolean EmergencyStop = m_Controller.getYButton();
        SmartDashboard.putBoolean("Shoot?", shootery);
        SmartDashboard.putBoolean("LimelightHasTarget?", m_LimelightHasValidTarget);
        SmartDashboard.putNumber("Encoder Position", Encoder.getPosition());
       SmartDashboard.putNumber("Encoder Velocity", Encoder.getVelocity());

        
        double Kp = -0.1f;
        double min_command = 0.05f;
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double left_command = 0;
        double right_command = 0;
        
        steer *= 0.40;
        drive *= 0.40;

       if (EmergencyStop)
        {
          shooter.set(0);
        }
        else
        {
          shooter.set(0);    
        }




        if (auto)
        {
          double steering_adjust = 0.0f;
          double heading = -tx;

          if (m_LimelightHasValidTarget)
          {
            if (tx > 1.0)
            {
                    steering_adjust = Kp*heading - min_command;
            }
            else if (tx < 1.0)
            {
                    steering_adjust = Kp*heading + min_command;
            }
            left_command += steering_adjust;
            right_command -= steering_adjust;
            
            m_Drive.tankDrive(left_command / 1.5, right_command / 1.5);
          }
        }
  
  double ty =  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  //acceration due to gravity
  final double G = 9.8;
  //ratio between linerar and angular velocity
  final double wheelToBall = 2;
  //distance from the ground to the Target
  final double TargetHeight = 1.6002;
  //distance from the limelight to the shooter
  final double limeToShooter = 0.6096;
  //angle of the ball shooter in degrees
  double shootangleD = 43; 
  //angle between middle of limelight and target in degrees
  double limeangleD = ty + 30;   
  //distance between ballshooter and target
  double distance = ((TargetHeight / Math.tan(Math.toRadians((limeangleD)))) + limeToShooter) * 2;

  //speed of the ball needed to reach the target
  double ballspeed = Math.sqrt((distance * G) / Math.sin(2 * (Math.toRadians(shootangleD))));
  //final velocity of the ball
  double vf = wheelToBall * ballspeed;
  //speed of the wheel needed to accelerate the ball
  double wheelspeed = (vf) + 1 + (1 + .4) / ((2 * .8) + (113.398 / 141.74)); 
  //rpm of the wheel
  double rpm = (wheelspeed * 60) / (.0619125 * 2 * Math.PI);
  //percentage out of the max rpm
  double percent = (rpm / 5750);

  SmartDashboard.putNumber("Limelight angle", limeangleD);
  SmartDashboard.putNumber("RPM", rpm);
  SmartDashboard.putNumber("rpm percent", percent * 100);
  SmartDashboard.putNumber("wheel speed", wheelspeed);
  SmartDashboard.putNumber("ball speed", ballspeed);
  SmartDashboard.putNumber("TY", Math.abs(ty));
  SmartDashboard.putNumber("distance to target", distance);

  if (shoot)
        {
          if (m_LimelightHasValidTarget)
          { 
            shooter.set(percent);
            SmartDashboard.putBoolean("Shooter running", true);
          }
          else
          {
            shooter.set(0); 
            SmartDashboard.putBoolean("Shooter running", false);   
          }
        }
        else
        {
          shooter.set(0);
          SmartDashboard.putBoolean("Shooter running", false);
        }

  }
  
  
  

  @Override
  public void testPeriodic() {
  }

  /**
   * This function implements a simple method of generating driving and steering commands
   * based on the tracking data from a limelight camera.
   */
  public void Update_Limelight_Tracking()
  {
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.30;                    // how hard to turn toward the target
        final double DRIVE_K = 0.30;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 20.0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.4;                   // Simple speed limit so we don't drive too fast

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        
        if (tv < 1.0)
        {
          m_LimelightHasValidTarget = false;
          return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
          drive_cmd = MAX_DRIVE;
        }
  }


@Override
  public void testInit() {  

  }

}