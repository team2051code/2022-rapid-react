package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
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
//public CANSparkMax TurretRotator = new CANSparkMax(RobotMap.TurretRotator, MotorType.kBrushless);

public CANSparkMax TurretRotator = new CANSparkMax(RobotMap.TurretRotator, MotorType.kBrushless);


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
         return steering_adjust;             

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

  return percent;
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

public void CalculatedShoot()
{
    if(m_oi.GetAButton())
    {
        M_shoot.ShootSpeedRight(ShootParamaters());
        M_shoot.ShootSpeedLeft(ShootParamaters());
    }
    else
    {
    M_shoot.ShootSpeedLeft(0);
    M_shoot.ShootSpeedRight(0);
    }

}


    }
    





