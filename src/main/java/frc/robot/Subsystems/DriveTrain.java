package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    CANSparkMax Right = new CANSparkMax(1, MotorType.kBrushless);
    CANSparkMax RightFollow = new CANSparkMax(2, MotorType.kBrushless);
    CANSparkMax Left = new CANSparkMax(3, MotorType.kBrushless);
    CANSparkMax LeftFollow = new CANSparkMax(4, MotorType.kBrushless);

  private MotorControllerGroup LeftSide = new MotorControllerGroup(Left, LeftFollow);
  private MotorControllerGroup RightSide = new MotorControllerGroup(Right, RightFollow);

  DifferentialDrive M_Drive = new DifferentialDrive(LeftSide, RightSide);

  private RelativeEncoder encoder;

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

    public void setMotors(double LeftSide, double RightSide) 
    {
        this.LeftSide.set(LeftSide);
        this.RightSide.set(RightSide);

    }

public void tankDrive(double LeftSpeed, double RightSpeed)
{
M_Drive.tankDrive(LeftSpeed, RightSpeed);


}

    public double GetEncoderInches()
    {
        return
        encoderTicksPerInches(encoder.getPosition());
        
    }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("DistanceInInches", encoderTicksPerInches(encoder.getPosition()));
    SmartDashboard.putNumber("Encoder Position", encoder.getPosition());
  }






  
    
}
