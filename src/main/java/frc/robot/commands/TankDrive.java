package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.ShootParamaters;
//import frc.robot.Subsystems.ShootParamaters;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

   


public class TankDrive extends CommandBase {
    public boolean m_LimelightHasValidTarget;
    public double steering_adjust;
    public ShootParamaters M_shoot;
    public DriveTrain IntakeMethod;
    public DriveTrain SetIntakeSpeed;
    public DriveTrain M_DriveTrain;
    public OI m_oi = new OI();
    public DriveTrain Shooter1;
    public DriveTrain Shooter2;
    public DriveTrain setShootSpeed;

    
 public TankDrive(DriveTrain Drivetrain, ShootParamaters Shootparameters) {
     M_shoot = Shootparameters;
    M_DriveTrain = Drivetrain;
     addRequirements(M_DriveTrain);
 }

// public ActuallyShoot(Shoot shoot)   {
//  M_shoot = shoot;
//     addRequirements(M_Shoot);
// }

@Override public void initialize(){
}

@Override public void execute(){
double SetTurretRotator = M_shoot.Update_Limelight_Tracking();
double LeftSide = m_oi.GetDriverRawAxis(RobotMap.LeftAxis);
double RightSide = m_oi.GetDriverRawAxis(RobotMap.RightAxis);

M_DriveTrain.tankDrive(LeftSide * 0.5, RightSide * 0.5);


    // boolean EnableShoot = m_oi.GetAButton(RobotMap.AButton);
    if(m_oi.GetAButton())
    {
        M_DriveTrain.ShootSpeedRight(.50);
        M_DriveTrain.ShootSpeedLeft(.50);
    }
    else
    {
    M_DriveTrain.ShootSpeedLeft(0);
    M_DriveTrain.ShootSpeedRight(0);
    }


    if(m_oi.GetBButton())
    {
    M_DriveTrain.SetIntakeSpeed(.50);
    }
    else
    {
    M_DriveTrain.SetIntakeSpeed(0);
    }
    
    M_shoot.TurretRotatorSpeed(M_shoot.Update_Limelight_Tracking());
    System.out.println(M_shoot.Update_Limelight_Tracking());
}


@Override public void end(boolean interrupt) {
    M_DriveTrain.tankDrive(0, 0);
}
@Override public boolean isFinished() {
return false;



    }
}



 

 
