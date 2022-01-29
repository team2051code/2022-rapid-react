package frc.robot.Subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoot extends SubsystemBase {
private boolean m_LimelightHasValidTarget = true;



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
 
         double Kp = -0.1f;
         double min_command = 0.05f;
         double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
         double left_command = 0;
         double right_command = 0;
         
         Update_Limelight_Tracking();
         
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
           turret.set(steering_adjust);
         }
        
        
        }
    }


