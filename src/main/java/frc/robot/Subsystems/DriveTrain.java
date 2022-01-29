package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.commands.TankDrive;

public class DriveTrain extends SubsystemBase {
  XboxController controller = new XboxController(RobotMap.XboxControllerUsbPort);
  
    private CANSparkMax Right = new CANSparkMax(RobotMap.Motor_Right, MotorType.kBrushless);
    private CANSparkMax RightFollow = new CANSparkMax(RobotMap.Motor_RightFollow, MotorType.kBrushless);
    private CANSparkMax Left = new CANSparkMax(RobotMap.Motor_Left, MotorType.kBrushless);
    private CANSparkMax LeftFollow = new CANSparkMax(RobotMap.Motor_LeftFollow, MotorType.kBrushless);

  private MotorControllerGroup LeftSide = new MotorControllerGroup(Left, LeftFollow);
  private MotorControllerGroup RightSide = new MotorControllerGroup(Right, RightFollow);
  private RelativeEncoder encoder;
  DifferentialDrive M_DriveTrain = new DifferentialDrive(LeftSide, RightSide);


  

  public DriveTrain() {
    
    encoder = Left.getEncoder();

    encoder.getPositionConversionFactor();
    encoder.setPosition(0);
    System.out.println(encoder.getPositionConversionFactor());

    Left.setInverted(true);
    LeftFollow.setInverted(true);

  }

  public double encoderTicksPerInches(double ticks){

    //Possibility that GearRatio is actually 10.75
    final double GearRatio = (10.75);
    //Math to calculate the number of motor pulses based on our rotations
    
    final double PulsesPerInch = (2.0 * Math.PI) * 3 / GearRatio;
    //Math to calculate the current distance of the motor using the previous equation
     return(ticks * PulsesPerInch);

    }
    
    public void LeftSide(double Speed)
    {
    Left.set(Speed);
    LeftFollow.set(Speed);
    }

    public void RightSide(double Speed)
    {
    Right.set(Speed);
    RightFollow.set(Speed);

    }

    public void setMotors(double LeftSide, double RightSide) 
    {
        this.LeftSide.set(LeftSide);
        this.RightSide.set(RightSide);

}

public double GetEncoderInches()
    {
        return
        encoderTicksPerInches(encoder.getPosition());
        
    }

  public void initDefaultCommand(TankDrive tankDrive)
  {
    setDefaultCommand(tankDrive);
    
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("DistanceInInches", encoderTicksPerInches(encoder.getPosition()));
    SmartDashboard.putNumber("Encoder Position", encoder.getPosition());
  }

  



  
    
}
