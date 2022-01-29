package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.commands.TankDrive;
import frc.robot.simulation.SimpleSimulatedChassis;

public class DriveTrain extends SubsystemBase {
  XboxController controller = new XboxController(RobotMap.XboxControllerUsbPort);

  private CANSparkMax Right = new CANSparkMax(RobotMap.Motor_Right, MotorType.kBrushless);
  private CANSparkMax RightFollow = new CANSparkMax(RobotMap.Motor_RightFollow, MotorType.kBrushless);
  private CANSparkMax Left = new CANSparkMax(RobotMap.Motor_Left, MotorType.kBrushless);
  private CANSparkMax LeftFollow = new CANSparkMax(RobotMap.Motor_LeftFollow, MotorType.kBrushless);

  private MotorControllerGroup LeftSide = new MotorControllerGroup(Left, LeftFollow);
  private MotorControllerGroup RightSide = new MotorControllerGroup(Right, RightFollow);
  DifferentialDrive M_DriveTrain = new DifferentialDrive(LeftSide, RightSide);

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  private ADXRS450_Gyro gyro;

  // Additional state used for robot simulation
  private ADXRS450_GyroSim simulatedGyro; // might be null
  private SimpleSimulatedChassis simulatedChassis; // might be null

  public DriveTrain() {
    leftEncoder = Left.getEncoder();
    rightEncoder = Right.getEncoder();
    gyro = new ADXRS450_Gyro();

    leftEncoder.getPositionConversionFactor();
    leftEncoder.setPosition(0);

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

    // Possibility that GearRatio is actually 10.75
    final double GearRatio = (10.75);
    // Math to calculate the number of motor pulses based on our rotations

    final double PulsesPerInch = (2.0 * Math.PI) * 3 / GearRatio;
    // Math to calculate the current distance of the motor using the previous
    // equation
    return (ticks * PulsesPerInch);

  }

  public void LeftSide(double Speed) {
    Left.set(Speed);
    LeftFollow.set(Speed);
  }

  public void RightSide(double Speed) {
    Right.set(Speed);
    RightFollow.set(Speed);
  }

  // TODO: Not using M_DriveTrain here and in LeftSide and RightSide will
  // make the watchdog angry and cause the motors to go chop-chop-chop
  public void setMotors(double LeftSide, double RightSide) {
    this.LeftSide.set(LeftSide);
    this.RightSide.set(RightSide);
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
    return leftEncoder.getPosition();
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
    return rightEncoder.getPosition();
  }

  public double GetEncoderInches() {
    return encoderTicksToInches(leftEncoder.getPosition());
  }

  /**
   * Get gyro angle in degrees
   * 
   * @return Gyro angle in degrees
   */
  public double getGyroAngleDegrees() {
    return gyro.getAngle();
  }

  /**
   * Sets left and right encoders. Generally, we don't want to call this except in
   * simulation.
   * 
   * @param newLeftValue  new left encoder value
   * @param newRightValue new right encoder value
   */
  public void setEncoders(double newLeftValue, double newRightValue) {
    leftEncoder.setPosition(newLeftValue);
    rightEncoder.setPosition(newRightValue);
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
    SmartDashboard.putNumber("DistanceInInches", encoderTicksToInches(leftEncoder.getPosition()));
    SmartDashboard.putNumber("Encoder Position", leftEncoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    if (simulatedChassis != null) {
      simulatedChassis.periodic();
    }
  }
}
