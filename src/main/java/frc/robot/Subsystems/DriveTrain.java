package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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
import frc.robot.simulation.SimpleSimulatedChassis;

public class DriveTrain extends SubsystemBase {
  XboxController controller = new XboxController(RobotMap.XBOX_CONTROLLER_USB_PORT);

  private WPI_TalonFX m_right = new WPI_TalonFX(RobotMap.MOTOR_RIGHT);
  private WPI_TalonFX m_rightFollow = new WPI_TalonFX(RobotMap.MOTOR_RIGHT_FOLLOW);
  private WPI_TalonFX m_left = new WPI_TalonFX(RobotMap.MOTOR_LEFT);
  private WPI_TalonFX m_leftFollow = new WPI_TalonFX(RobotMap.MOTOR_LEFT_FOLLOW);
  public OI m_oi;
  
  private CANSparkMax m_intakMethod = new CANSparkMax(RobotMap.INTAKE_MOTOR, MotorType.kBrushless);

  private MotorControllerGroup m_leftSide = new MotorControllerGroup(m_left, m_leftFollow);
  private MotorControllerGroup m_rightSide = new MotorControllerGroup(m_right, m_rightFollow);
  CANSparkMax m_singulatorMotor = new CANSparkMax(RobotMap.SINGULATOR_MOTOR, MotorType.kBrushless);
  DifferentialDrive m_driveTrain = new DifferentialDrive(m_leftSide, m_rightSide);

  private ADXRS450_Gyro m_gyro;

  // Additional state used for robot simulation
  private ADXRS450_GyroSim m_simulatedGyro; // might be null
  private SimpleSimulatedChassis m_simulatedChassis; // might be null
  private double m_simulatedLeftEncoder = 0;
  private double m_simulatedRightEncoder = 0;

  public DriveTrain(OI oi) {
    m_left.setSelectedSensorPosition(0);
    // leftEncoder.setPosition(0);
    // rightEncoder.setPosition(0);
    m_gyro = new ADXRS450_Gyro();
    m_oi = oi;

    // leftEncoder.getPositionConversionFactor();

    m_left.setInverted(true);
    m_leftFollow.setInverted(true);

  }

  /**
   * Call only if the robot is running in simulation. Prepares simulated parts.
   */
  public void simulationInit() {
    m_simulatedGyro = new ADXRS450_GyroSim(m_gyro);
    m_simulatedGyro.setAngle(30);
    m_simulatedChassis = new SimpleSimulatedChassis(this);
  }

  public double encoderTicksToInches(double ticks) {

    // // Possibility that GearRatio is actually 10.75
    final double gearRatio = (18);
    // // Math to calculate the number of motor pulses based on our rotations

    final double pulsesPerInch = (2.0 * Math.PI) * 3 / gearRatio;
    // // Math to calculate the current distance of the motor using the previous
    // // equation
    return (ticks / 2061 * pulsesPerInch);
  }

  public void leftSide(double Speed) {
    m_left.set(Speed);
    m_leftFollow.set(Speed);
  }

  public void rightSide(double Speed) {
    m_right.set(Speed);
    m_rightFollow.set(Speed);
  }

  public void intakeSpeed(double Speed) {
    m_intakMethod.set(Speed);
    m_singulatorMotor.set(Speed);
  }

  public void setIntakeSpeed() {
    if (m_oi.getXButton2()) {
      this.m_intakMethod.set(.90);
      this.m_singulatorMotor.set(.90);
    } else {
      this.m_intakMethod.set(0);
      this.m_singulatorMotor.set(0);
    }
  }

  public void SetAutonomousIntake() {
    this.m_intakMethod.set(.90);
    this.m_singulatorMotor.set(.90);
  }

  public void StopIntake() {

    m_intakMethod.set(0);

  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_driveTrain.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * Get left motor speed
   * 
   * @return Speed value, -1 to 1
   */
  public double getLeftMotorSpeed() {
    return m_leftSide.get();
  }

  /**
   * Get right motor speed
   * 
   * @return Speed value, -1 to 1
   */
  public double getRightMotorSpeed() {
    return m_rightSide.get();
  }

  
 


  public void SlowerControls(){

    while(m_oi.getXButton())
      {
        m_driveTrain.tankDrive(-.5, .5);
      }
     

      while(m_oi.GetAButton())
      {
        m_driveTrain.tankDrive(-.5, -.5);
      }
      

      while(m_oi.getYButton()){
      m_driveTrain.tankDrive(.5, .5);
      }
     

     while(m_oi.getBButton()){
      m_driveTrain.tankDrive(.5, -.5);
      }
      
      }

    

  

  /**
   * Get left raw encoder value
   * 
   * @return raw encoder value for left side
   */
  public double getLeftEncoderValue() {
    if (m_simulatedChassis != null) {
      return m_simulatedLeftEncoder;
    } else {
      return m_left.getSelectedSensorPosition();
    }
  }

  /**
   * Get right raw encoder value
   * 
   * @return raw encoder value for right side
   */
  public double getRightEncoderValue() {
    if (m_simulatedChassis != null) {
      return m_simulatedRightEncoder;
    } else {
      return m_right.getSelectedSensorPosition();
    }
  }

  public double getEncoderInches() {
    return encoderTicksToInches(getLeftEncoderValue());
  }

  /**
   * Get gyro angle in degrees
   * 
   * @return Gyro angle in degrees
   */
  public double getGyroAngleDegrees() {
    return m_gyro.getAngle();
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  /**
   * Sets left and right encoders. Generally, we don't want to call this except in
   * simulation.
   * 
   * @param newLeftValue  new left encoder value
   * @param newRightValue new right encoder value
   */
  public void setSimulatedEncoders(double newLeftValue, double newRightValue) {
    m_simulatedLeftEncoder = newLeftValue;
    m_simulatedRightEncoder = newRightValue;
  }

  /**
   * Resets encoders to 0 value
   */
  public void resetEncoders() {
    m_right.setSelectedSensorPosition(0);
    m_left.setSelectedSensorPosition(0);
    m_simulatedLeftEncoder = 0;
    m_simulatedRightEncoder = 0;
  }

  /**
   * Set a simulated gyro value. Does nothing if not simulating.
   * 
   * @param angleDegrees new angle in degrees
   */
  public void setSimulatedGyro(double angleDegrees) {
    if (m_simulatedGyro != null) {
      m_simulatedGyro.setAngle(angleDegrees);
    }
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("DistanceInInches",
    // encoderTicksToInches(leftEncoder.getPosition()));
    // SmartDashboard.putNumber("Encoder Position", leftEncoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    if (m_simulatedChassis != null) {
      m_simulatedChassis.periodic();
    }

  }
}
