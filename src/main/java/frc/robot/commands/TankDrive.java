package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Pneumatics;
import frc.robot.Subsystems.ShootParamaters;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.MotorSafety;
//import frc.robot.Subsystems.ShootParamaters;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

   


public class TankDrive extends CommandBase {
    public Pneumatics m_Pneumatics;
    public boolean m_LimelightHasValidTarget;
    public ShootParamaters M_shoot;
    public DriveTrain IntakeMethod;
    public DriveTrain SetIntakeSpeed;
    public DriveTrain M_DriveTrain;
    public OI m_oi = new OI();
    //public DriveTrain Shooter1;
    //public DriveTrain Shooter2;
    //public DriveTrain setShootSpeed;
    //public CANSparkMax TurretRotator = new CANSparkMax(RobotMap.TurretRotator, MotorType.kBrushless);
    
 public TankDrive(DriveTrain Drivetrain, ShootParamaters Shootparameters, Pneumatics Pneumatics) {
    M_shoot = Shootparameters;
    M_DriveTrain = Drivetrain;
    m_Pneumatics = Pneumatics;
     addRequirements(M_DriveTrain, M_shoot, m_Pneumatics);
 }

// public ActuallyShoot(Shoot shoot)   {
//  M_shoot = shoot;
//     addRequirements(M_Shoot);
// }

@Override public void initialize(){

}

@Override public void execute(){
m_Pneumatics.CompressorOnOrOff();
//double SetTurretRotator = M_shoot.Update_Limelight_Tracking();
double LeftSide = m_oi.GetDriverRawAxis(RobotMap.LeftAxis);
double RightSide = m_oi.GetDriverRawAxis(RobotMap.RightAxis);

M_DriveTrain.tankDrive(LeftSide , RightSide);
    

    // if(m_oi.GetXButton())
    // {
    //     M_shoot.SetTurretRotatorSpeed(SetTurretRotator / 10);
    // }
    // else{
    //     M_shoot.SetTurretRotatorSpeed(0);


    // }




    // boolean EnableShoot = m_oi.GetAButton(RobotMap.AButton);
    // if(m_oi.GetAButton())
    // {
    //     M_DriveTrain.ShootSpeedRight(.20);
    //     M_DriveTrain.ShootSpeedLeft(.20);
    // }
    // else
    // {
    // M_DriveTrain.ShootSpeedLeft(0);
    // M_DriveTrain.ShootSpeedRight(0);
    // }
    if(m_oi.GetYButton())
    {
    M_DriveTrain.SetIntakeSpeed(-.70);
    }
    else if ( m_oi.GetBButton())
    {
        M_DriveTrain.SetIntakeSpeed(1);
    }
    else 
    {
        M_DriveTrain.SetIntakeSpeed(0);
    }

    // if(m_oi.GetXButton())
    // {
    // }
    m_Pneumatics.forwards();
    m_Pneumatics.GearShift();

    
    
   //System.out.println(SetTurretRotator);
    //System.out.println(M_shoot.Update_Limelight_Tracking());
}   


@Override public void end(boolean interrupt) {
    M_DriveTrain.tankDrive(0, 0);
}
@Override public boolean isFinished() {
return false;



    }
}



 

 
