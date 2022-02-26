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
    //This forces the Command to require these 4 subsystems to fuction
     addRequirements(M_DriveTrain, m_Pneumatics, m_Singulator, M_shoot);
     //M_shoot );
 }

// public ActuallyShoot(Shoot shoot)   {
//  M_shoot = shoot;
//     addRequirements(M_Shoot);
// }

@Override public void initialize(){
//Sets the left and Right encoder sensors to 0 every time the robot starts up 
M_DriveTrain.TestingMotors();
}

@Override public void execute(){
//Reads the Right Side Encoder Values
//M_DriveTrain.ReadEncoder();
m_Singulator.SetSingulatorSpeed();
//Reads the current SolenoidState
m_Pneumatics.SolenoidState();
//CombinedShootSpeed
M_shoot.CombinedShootSpeed();
//Sets the speed of the shooter
M_shoot.CalculatedShootSpeed();
//Sets the Intake Speed To a Certain Value
M_DriveTrain.SetIntakeSpeed();

//m_oi.UpdateToggle();

//Commands to GearShift and Raise/Lower the intake
m_Pneumatics.forwards();
m_Pneumatics.GearShift();

//Command to set the Speed of the Singulator
m_Singulator.SetSingulatorSpeed();

//Subsytem for the Caluated Shoot Speed
double CalculatedShootSpeed = M_shoot.ShootParamaters();
//Subsytem for auto targeting
double SetTurretRotator = M_shoot.Update_Limelight_Tracking();

//Sets the axis in which the robot drives on
double LeftSide = m_oi.GetDriverRawAxis(RobotMap.LeftAxis);
double RightSide = m_oi.GetDriverRawAxis(RobotMap.RightAxis);

//Drivetrain command to allow TankDrive
M_DriveTrain.tankDrive(LeftSide , RightSide);


     
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



 

 
