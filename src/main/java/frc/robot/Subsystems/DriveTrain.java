package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.commands.TankDrive;
import frc.robot.simulation.SimpleSimulatedChassis;

public class DriveTrain extends SubsystemBase {
  XboxController controller = new XboxController(RobotMap.XboxControllerUsbPort);



  private WPI_TalonFX Right = new WPI_TalonFX(RobotMap.Motor_Right);
  private WPI_TalonFX RightFollow = new WPI_TalonFX(RobotMap.Motor_RightFollow);
  private WPI_TalonFX Left = new WPI_TalonFX(RobotMap.Motor_Left);
  private WPI_TalonFX LeftFollow = new WPI_TalonFX(RobotMap.Motor_LeftFollow);
  public OI m_oi = new OI();

  private CANSparkMax IntakeMethod = new CANSparkMax(RobotMap.IntakeMotor, MotorType.kBrushless);

  private MotorControllerGroup LeftSide = new MotorControllerGroup(Left, LeftFollow);
  private MotorControllerGroup RightSide = new MotorControllerGroup(Right, RightFollow);
  CANSparkMax SingulatorMotor = new CANSparkMax(RobotMap.SingulatorMotor, MotorType.kBrushless);
  DifferentialDrive M_DriveTrain = new DifferentialDrive(LeftSide, RightSide);
 
  private ADXRS450_Gyro gyro;

  // Additional state used for robot simulation
  private ADXRS450_GyroSim simulatedGyro; // might be null
  private SimpleSimulatedChassis simulatedChassis; // might be null
  

  public DriveTrain() {
    Left.setSelectedSensorPosition(0);
  //leftEncoder.setPosition(0);
  //rightEncoder.setPosition(0);
    gyro = new ADXRS450_Gyro();

    //leftEncoder.getPositionConversionFactor();
  
    Left.setInverted(true);
    LeftFollow.setInverted(true);

  }

  /**
   * Call only if the robot is running in simulation. Prepares simulated parts.
   */
  public void simulationInit() {
     simulatedGyro = new ADXRS450_GyroSim(gyro);
     simulatedGyro.setAngle(-30);
     simulatedChassis = new SimpleSimulatedChassis(this);
   }

  public double encoderTicksToInches(double ticks) {

  //   // Possibility that GearRatio is actually 10.75
    final double GearRatio = (18);
  //   // Math to calculate the number of motor pulses based on our rotations

    final double PulsesPerInch = (2.0 * Math.PI) * 3 / GearRatio;
  //   // Math to calculate the current distance of the motor using the previous
  //   // equation
     return ((Left.getSelectedSensorPosition() / 2061) * PulsesPerInch);
   }

  public void LeftSide(double Speed) {
    Left.set(Speed);
    LeftFollow.set(Speed);
  }

  public void RightSide(double Speed) {
    Right.set(Speed);
    RightFollow.set(Speed);
  }


  public void IntakeSpeed(double Speed)
  {
    IntakeMethod.set(Speed);
    SingulatorMotor.set(Speed);
  }

  public void SetIntakeSpeed()
  {
    if(m_oi.GetXButton2()){
    this.IntakeMethod.set(.90);
    this.SingulatorMotor.set(.90);
    }
    else{
    this.IntakeMethod.set(0);
    this.SingulatorMotor.set(0);
  }
}

  public void tankDrive(double LeftSpeed, double RightSpeed) {
    M_DriveTrain.tankDrive(LeftSpeed, RightSpeed);
  }

  
    

  /**
   * Get left motor speed
   * 
   * @return Speed value, -1 to 1
   */
  public double getLeftMotorSpeed() {
    return LeftSide.get();
  }

  /**
   * Get right motor speed
   * 
   * @return Speed value, -1 to 1
   */
  public double getRightMotorSpeed() {
    return RightSide.get();
  }



  /**
   * Get left raw encoder value
   * 
   * @return raw encoder value for left side
   */
  public double getLeftEncoderValue() {
    return Left.getSelectedSensorPosition();
  }

  public void initDefaultCommand(TankDrive tankDrive) {
    setDefaultCommand(tankDrive);

  }

  /**
   * Get right raw encoder value
   * 
   * @return raw encoder value for right side
   */
  public double getRightEncoderValue() {
    return Right.getSelectedSensorPosition();
  }

  public double GetEncoderInches() {
    return encoderTicksToInches(Left.getSelectedSensorPosition());
  }

  /**
   * Get gyro angle in degrees
   * 
   * @return Gyro angle in degrees
   */
   public double getGyroAngleDegrees() {
     return gyro.getAngle();
   }

   public void ResetGyro(){
    gyro.reset();
   }


  /**
   * Sets left and right encoders. Generally, we don't want to call this except in
   * simulation.
   * 
   * @param newLeftValue  new left encoder value
   * @param newRightValue new right encoder value
   */
  public void setEncoders(double newLeftValue, double newRightValue) {
    Left.setSelectedSensorPosition(newLeftValue);
    Right.setSelectedSensorPosition(newRightValue);
  }

  public void TestingMotors()
  {
    //System.out.println(RightFollow.get());
    //\System.out.println(Left.get());
    Right.setSelectedSensorPosition(0);
    Left.setSelectedSensorPosition(0);

  }
  
public void ReadEncoder(){

System.out.println(Right.getSelectedSensorPosition());

}
  /**
   * Set a simulated gyro value. Does nothing if not simulating.
   * 
   * @param angleDegrees new angle in degrees
   */
   public void setSimulatedGyro(double angleDegrees) {
     if (simulatedGyro != null) {
       simulatedGyro.setAngle(angleDegrees);
     }
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("DistanceInInches", encoderTicksToInches(leftEncoder.getPosition()));
    //SmartDashboard.putNumber("Encoder Position", leftEncoder.getPosition());
  }

   @Override
   public void simulationPeriodic() {
     if (simulatedChassis != null) {
       simulatedChassis.periodic();
     }
     
   }
}
