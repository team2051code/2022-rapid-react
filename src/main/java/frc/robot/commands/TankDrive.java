package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.Subsystems.BallSensor;
import frc.robot.Subsystems.ClimbControls;
import frc.robot.Subsystems.DriveTrain;
//import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.Pneumatics;
import frc.robot.Subsystems.ShootParamaters;
import frc.robot.Subsystems.SingulatorInformation;

   


public class TankDrive extends CommandBase {
    public Pneumatics m_pneumatics;
    public boolean m_LimelightHasValidTarget;
    public ShootParamaters M_shoot;
    public DriveTrain m_intakeMethod;
    public DriveTrain m_setIntakeSpeed;
    public DriveTrain m_driveTrain;
    public OI m_oi;
    public SingulatorInformation m_singulator;
    public ClimbControls m_climb;
    public BallSensor m_ballz;
    //public DriveTrain Shooter1;
    //public DriveTrain Shooter2;
    //public DriveTrain setShootSpeed;
    //public CANSparkMax TurretRotator = new CANSparkMax(RobotMap.TurretRotator, MotorType.kBrushless);
    
    public TankDrive(OI oi, DriveTrain drivetrain, Pneumatics pneumatics, ShootParamaters Shootparameters, SingulatorInformation singulatorInformation, ClimbControls climbControls, BallSensor ballSensor){
    //These next four lines define our subsystems so our Commands can access them
    m_singulator = singulatorInformation;
    M_shoot = Shootparameters;
    m_driveTrain = drivetrain;
    m_pneumatics = pneumatics;
    m_climb = climbControls;
    m_ballz = ballSensor;
    m_oi = oi;
    
    //This forces the Command to require these 4 subsystems to fuction
     addRequirements(m_driveTrain, m_pneumatics, m_singulator, M_shoot);
 }

// public ActuallyShoot(Shoot shoot)   {
//  M_shoot = shoot;
//     addRequirements(M_Shoot);
// }

@Override public void initialize(){
//Sets the left and Right encoder sensors to 0 every time the robot starts up 
m_driveTrain.resetEncoders();
//m_climb.ResetEncoders();
}

@Override public void execute(){
//Reads the Right Side Encoder Values
//M_DriveTrain.ReadEncoder();
m_singulator.setSingulatorSpeed();
//Reads the current SolenoidState
m_pneumatics.solenoidState();
M_shoot.turretRotatorSpeed();
//Sets the speed of the shooter
M_shoot.calculatedShootSpeed();
//Sets the Intake Speed To a Certain Value
m_driveTrain.setIntakeSpeed();
//Sets the Speed That The Climb Raises
//m_ballz.CurrentSensorState();
m_ballz.ReadLineSensors();
m_ballz.PrintSensor();
//m_oi.UpdateToggle();

//Commands to GearShift and Raise/Lower the intake
m_pneumatics.forwards();
m_pneumatics.gearShift();
//m_pneumatics.backwards();

//m_climb.ReadClimbEncoders();

//Command to set the Speed of the Singulator
m_singulator.setSingulatorSpeed();
//Sets the Paramaters and Controls For The Climb Mechanism
m_climb.ForwardClimbSpeed();
//m_climb.ForwardClimbSpeed();

//m_climb.ReadClimbEncoders();
m_ballz.PrintSensor();
//Subsytem for the Caluated Shoot Speed
//double CalculatedShootSpeed = M_shoot.computeShooterVelocity();
//Subsytem for auto targeting
//double SetTurretRotator = M_shoot.Update_Limelight_Tracking();

//Sets the axis in which the robot drives on
double LeftSide = m_oi.getDriverRawAxis(RobotMap.LEFT_AXIS);
double RightSide = m_oi.getDriverRawAxis(RobotMap.RIGHT_AXIS);

//Drivetrain command to allow TankDrive
m_driveTrain.tankDrive(LeftSide , RightSide);

}   


@Override public void end(boolean interrupt) {
    m_driveTrain.tankDrive(0, 0);
}
@Override public boolean isFinished() {
return false;



    }
}



 

 
