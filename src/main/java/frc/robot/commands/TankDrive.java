package frc.robot.commands;

import java.lang.module.ModuleDescriptor.Requires;

import com.revrobotics.RelativeEncoder;

import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Subsystems.DriveTrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

   


public class TankDrive extends CommandBase {     

    public DriveTrain M_DriveTrain;
    public OI m_oi = new OI();
    
 public TankDrive(DriveTrain Drivetrain) {
    M_DriveTrain = Drivetrain;
     addRequirements(M_DriveTrain);
 }



@Override public void initialize(){


}

@Override public void execute(){

double LeftSide = m_oi.GetDriverRawAxis(RobotMap.LeftAxis);
double RightSide = m_oi.GetDriverRawAxis(RobotMap.RightAxis);

M_DriveTrain.LeftSide(LeftSide*0.5);
M_DriveTrain.RightSide(RightSide*0.5);



}

@Override public void end(boolean interrupt) {
    M_DriveTrain.LeftSide(0);
    M_DriveTrain.RightSide(0);


}
@Override public boolean isFinished() {
return false;



    }
}



 

 
