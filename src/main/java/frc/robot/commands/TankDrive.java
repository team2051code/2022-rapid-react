package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Subsystems.DriveTrain;
//import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.Pneumatics;
//import frc.robot.Subsystems.ShootParamaters;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.MotorSafety;
import frc.robot.Subsystems.ShootParamaters;
import frc.robot.Subsystems.SingulatorInformation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

   


public class TankDrive extends CommandBase {
    public Pneumatics m_Pneumatics;
    public boolean m_LimelightHasValidTarget;
    public ShootParamaters M_shoot;
    public DriveTrain IntakeMethod;
    public DriveTrain SetIntakeSpeed;
    public DriveTrain M_DriveTrain;
    public OI m_oi = new OI();
    public SingulatorInformation m_Singulator;
    //public DriveTrain Shooter1;
    //public DriveTrain Shooter2;
    //public DriveTrain setShootSpeed;
    //public CANSparkMax TurretRotator = new CANSparkMax(RobotMap.TurretRotator, MotorType.kBrushless);
    
    public TankDrive(DriveTrain Drivetrain, Pneumatics Pneumatics, ShootParamaters Shootparameters, SingulatorInformation Singulatorinformation){
    //These next four lines define our subsystems so our Commands can access them
    m_Singulator = Singulatorinformation;
    M_shoot = Shootparameters;
    M_DriveTrain = Drivetrain;
    m_Pneumatics = Pneumatics;
     addRequirements(M_DriveTrain, m_Pneumatics, m_Pneumatics, m_Singulator);
     //M_shoot );
 }

// public ActuallyShoot(Shoot shoot)   {
//  M_shoot = shoot;
//     addRequirements(M_Shoot);
// }

@Override public void initialize(){
M_DriveTrain.TestingMotors();
}

@Override public void execute(){
//M_DriveTrain.ReadEncoder();
m_Pneumatics.SolenoidState();
M_DriveTrain.SetIntakeSpeed();
m_oi.UpdateToggle();
//M_shoot.EncoderLimitTesting();
m_Pneumatics.forwards();
m_Pneumatics.GearShift();
double CalculatedShootSpeed = M_shoot.ShootParamaters();
double SetTurretRotator = M_shoot.Update_Limelight_Tracking();


double LeftSide = m_oi.GetDriverRawAxis(RobotMap.LeftAxis);
double RightSide = m_oi.GetDriverRawAxis(RobotMap.RightAxis);


M_DriveTrain.tankDrive(LeftSide , RightSide);

    if(m_oi.GetLeftBumper2())
    {
        m_Singulator.SetSingulatorSpeed(.80);
    }
    else{
        m_Singulator.SetSingulatorSpeed(0);

    }





    if(m_oi.GetRightBumper2())
{
    M_shoot.ShootSpeedRight(CalculatedShootSpeed);
    M_shoot.ShootSpeedLeft(CalculatedShootSpeed);
}
    else{
    M_shoot.ShootSpeedRight(0);
     M_shoot.ShootSpeedLeft(0);
     

    }

    
    

     if(m_oi.GetAButton2())
     {
      M_shoot.SetTurretRotatorSpeed(SetTurretRotator / 60);
     }
     else{
     M_shoot.SetTurretRotatorSpeed(0);
     }



    // if(m_oi.GetBButton())
    // {
    // M_DriveTrain.SetIntakeSpeed(-.70);
    // }
    


    // if(m_oi.GetXButton())
    // {
    // }
    
    //System.out.println(M_DriveTrain.getLeftEncoderValue());
    
   //System.out.println(SetTurretRotator);
    //System.out.println(M_shoot.Update_Limelight_Tracking());
    //System.out.println(M_DriveTrain.getLeftEncoderValue());

       // if(m_oi.toggleOn)
       // {
            

       // }

}   


@Override public void end(boolean interrupt) {
    M_DriveTrain.tankDrive(0, 0);
}
@Override public boolean isFinished() {
return false;



    }
}



 

 
