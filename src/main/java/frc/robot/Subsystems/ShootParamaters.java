package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Drive;
import frc.robot.commands.TankDrive;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.Subsystems.DriveTrain;


public class ShootParamaters extends SubsystemBase {
XboxController controller = new XboxController(RobotMap.XboxControllerUsbPort);
boolean m_LimelightHasValidTarget = false;
public OI m_oi = new OI();
public CANSparkMax TurretRotator = new CANSparkMax(RobotMap.TurretRotator, MotorType.kBrushless);


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
         
         double steering_adjust = 0.0f;
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
    


}


