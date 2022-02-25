package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TankDrive;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.Subsystems.DriveTrain;


public class ShootParamaters extends SubsystemBase {
public ShootParamaters M_shoot;
XboxController controller = new XboxController(RobotMap.XboxControllerUsbPort);
boolean m_LimelightHasValidTarget = false;
public OI m_oi = new OI();
WPI_TalonFX ShooterLeft = new WPI_TalonFX(RobotMap.ShootingMotor1);
WPI_TalonFX ShooterRight = new WPI_TalonFX(RobotMap.ShootingMotor2);
CANSparkMax TurretRotator = new CANSparkMax(RobotMap.TurretRotator, MotorType.kBrushless);
RelativeEncoder TurretRotatorEncoder; 

//public CANSparkMax TurretRotator = new CANSparkMax(RobotMap.TurretRotator, MotorType.kBrushless);


         public double Update_Limelight_Tracking()
    {
         //double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
         double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
         //double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
         //double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

         double Kp = -0.1f;
         double min_command = 0.03f;
         //double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
         
         double steering_adjust = tx;
         double heading = -tx;

         //Update_Limelight_Tracking();

         if (m_LimelightHasValidTarget)
         {
           
           if (tx >= 1.0)
           {
                     steering_adjust = Kp*heading - min_command;
           }
        if (m_LimelightHasValidTarget)
        {

          if (tx < 1.0){
                     steering_adjust = Kp*heading + min_command;
                     System.out.println(steering_adjust);
         }  
       }
    }
         return -steering_adjust;             

         }
        
public void TurretRotatorSpeed(double steering_adjust)
   {
   TurretRotator.set(steering_adjust);
   }

 public void SetTurretRotatorSpeed(double steering_adjust) {
  this.TurretRotator.set(steering_adjust);
}
public double ShootParamaters()
  {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double ty =  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  //acceration due to gravity
  final double G = 9.8;
  //ratio between linerar and angular velocity
  final double wheelToBall = 2;
  // Height for testing
  final double TestHeight = 1.5494;
  //distance from the ground to the Target
  final double TargetHeight = 2.64;
  // distance from ground to limelight
  final double LimeToGround = .679;
//distance from the limelight to the shooter
  final double limeToShooter = 0;
  //angle of the ball shooter in degrees
  double shootangleD = 55; 
  //angle between middle of limelight and target in degrees
  double limeangleD = ty + 35;   
  //distance between ballshooter and target
  double distance = (((TargetHeight - LimeToGround) / Math.tan(Math.toRadians((limeangleD)))) + limeToShooter) * 2;

  //speed of the ball needed to reach the target
  double ballspeed = Math.sqrt((distance * G) / Math.sin(2 * (Math.toRadians(shootangleD))));
  //final velocity of the ball
  double vf = wheelToBall * ballspeed;
  //speed of the wheel needed to accelerate the ball
  double wheelspeed = (vf) + 1 + (1 + .4) / ((2 * .8) + (671.31 / 270.00)); 
  //rpm of the wheel
  double rpm = (wheelspeed * 60) / (.0508 * 2 * Math.PI);
  //percentage out of the max rpm
  double percent = (rpm / 6380);
  double finalspeed = percent * 1.15;

  //SmartDashboard.putNumber("Limelight angle", limeangleD);
  //SmartDashboard.putNumber("RPM", rpm);
  //SmartDashboard.putNumber("rpm percent", percent * 100);
  //SmartDashboard.putNumber("wheel speed", wheelspeed);
  //SmartDashboard.putNumber("ball speed", ballspeed);
  //SmartDashboard.putNumber("TY", Math.abs(ty));
  //SmartDashboard.putNumber("distance to target", distance);

  return finalspeed;
}










public void ShootSpeedLeft(double Speed)
{
  ShooterLeft.setInverted(false);
  ShooterLeft.set(Speed);
}

public void ShootSpeedRight(double Speed)
{
  ShooterRight.setInverted(true);
  ShooterRight.set(Speed);
}

// public void TurretRotatorLimits()
// {
// if (TurretRotatorEncoder.getPosition() >= 150)
// {
//   TurretRotator.set(0);
// }

// if(TurretRotatorEncoder.getPosition() <= -150)
// {
//   TurretRotator.set(0);
// }

// }

// public void EncoderLimitTesting()
//  {
// TurretRotatorEncoder = TurretRotator.getEncoder();
// System.out.println(TurretRotatorEncoder.getPosition());

//  }






    }
  





