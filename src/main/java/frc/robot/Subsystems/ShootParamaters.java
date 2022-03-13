package frc.robot.Subsystems;

import java.lang.annotation.Target;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.RobotMap;

public class ShootParamaters extends SubsystemBase {
  /**
   *
   */
  private static final double TURRET_CENTER_X = 1.0;
  
  boolean m_LimelightHasValidTarget = false;
  public OI m_oi;
  public WPI_TalonFX m_shooterLeft = new WPI_TalonFX(RobotMap.SHOOTING_MOTOR_1);
  WPI_TalonFX m_shooterRight = new WPI_TalonFX(RobotMap.SHOOTING_MOTOR_2);
  CANSparkMax m_turretRotator = new CANSparkMax(RobotMap.TURRET_ROTATOR, MotorType.kBrushless);
  RelativeEncoder m_turretRotatorEncoder;
  PIDController m_shooterController = new PIDController(0.000075, 0.00023, 0);

  // public CANSparkMax TurretRotator = new CANSparkMax(RobotMap.TurretRotator,
  // MotorType.kBrushless);

public ShootParamaters(OI oi)
{
  m_oi = oi;
}
  public double updateLimelightTracking() {
    // double tv =
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    // double ty =
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    // double ta =
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    double kP = -0.1f;
    double minCommand = 0.03f;
    // double tx =
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    double steeringAdjust = tx;
    double heading = -tx;

    // Update_Limelight_Tracking();

    if (m_LimelightHasValidTarget) {

      if (tx >= TURRET_CENTER_X) {
        steeringAdjust = kP * heading - minCommand;
      }
      if (m_LimelightHasValidTarget) {

        if (tx < TURRET_CENTER_X) {
          steeringAdjust = kP * heading + minCommand;
          System.out.println(steeringAdjust);
        }
      }
    }
    return -steeringAdjust;

  }

  public void setTurretRotatorSpeed(double speed) {
    m_turretRotator.set(speed);
  }

  public void turretRotatorSpeed() {

    if (m_oi.getAButton2()) {
      updateTurretRotation();
    } else if(!m_oi.manualAimMode()) {
      setTurretRotatorSpeed(0);
    }

  }

  public void ManualTurretRotation(){

    m_turretRotator.set(m_oi.GetTurretRotationAxis());

  }

  public void updateTurretRotation() {
    setTurretRotatorSpeed(updateLimelightTracking() / 40);
  }

  public boolean isTurretCentered() {
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double buffer = .5;
    return (tx < 0 + buffer && tx > 0 - buffer);

  }

  public void shootSpeedLeft(double speed) {
    m_shooterLeft.setInverted(true);
    m_shooterLeft.set(speed);
  }

  public void shootSpeedRight(double speed) {
    m_shooterRight.setInverted(false);
    m_shooterRight.set(speed);
  }

  public void calculatedShootSpeed() {

    if (m_oi.getRightBumper2()) {
      setTargetSpeedToCalculatedSpeed();

    } else {
      setShooterTargetSpeed(0);
    }

  }

  /**
   * Get the target velocity for the shooter
   * 
   * @return Target encoder ticks per 100ms
   */
  public double computeShooterVelocity() {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    // acceration due to gravity
    final double g = 9.8;
    // ratio between linerar and angular velocity
    final double wheelToBall = 2;
    // Height for testing
    final double testHeight = 1.5494;
    // distance from the ground to the Target
    final double targetHeight = 2.64;
    // distance from ground to limelight
    final double limeToGround = .679;
    // distance from the limelight to the shooter
    final double limeToShooter = .2286;
    // angle of the ball shooter in degrees
    double shootangleD = 55;
    // angle between middle of limelight and target in degrees
    double limeangleD = ty + 27.5;
    // distance between ballshooter and target
    double distance = ((((targetHeight - limeToGround) / Math.tan(Math.toRadians((limeangleD)))) + limeToShooter) * 2);

    double distance2 = (distance / 2) * 3.281;



    // speed of the ball needed to reach the target
    double ballspeed = Math.sqrt(((distance + 7) * g) / Math.sin(2 * (Math.toRadians(shootangleD))));
    // final velocity of the ball
    double vf = wheelToBall * ballspeed;
    // speed of the wheel needed to accelerate the ball
    double wheelspeed = (vf) + 1 + (1 + .4) / ((2 * .5) + (671.31 / 270.00));
    // rpm of the wheel
    double rpm = (wheelspeed * 60) / (.0508 * 2 * Math.PI);

    // SmartDashboard.putNumber("Limelight angle", limeangleD);
    // SmartDashboard.putNumber("RPM", rpm);
    // SmartDashboard.putNumber("rpm percent", percent * 100);
    // SmartDashboard.putNumber("wheel speed", wheelspeed);
    // SmartDashboard.putNumber("ball speed", ballspeed);
    // SmartDashboard.putNumber("TY", Math.abs(ty));
     SmartDashboard.putNumber("distance to target", distance2);

    // TODO: boost output by 15% of max RPM (fudge factor)
    final double ENCODER_TICKS_PER_REVOLUTION = 2048;
    final double TENTHS_OF_A_SECOND_PER_MINUTE = 600;

    return (rpm * (ENCODER_TICKS_PER_REVOLUTION / TENTHS_OF_A_SECOND_PER_MINUTE));
    
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub

    double targetRpm = m_shooterController.getSetpoint();
    if (m_shooterController.getSetpoint() == 0) {
      shootSpeedRight(0);
      shootSpeedLeft(0);
      m_shooterController.reset();
    } else {
      double measuredRpm = m_shooterRight.getSelectedSensorVelocity();
      double outputValue = m_shooterController.calculate(measuredRpm);
      System.out.print(" t: ");
      System.out.print(targetRpm);
      System.out.print(" m: ");
      System.out.print(measuredRpm);
      System.out.print(" o: ");
      System.out.println(outputValue);
      outputValue = Math.max(-1, Math.min(1, outputValue));
      shootSpeedRight(outputValue);
      shootSpeedLeft(outputValue);
      
      if (measuredRpm <= targetRpm + 50 && measuredRpm >= targetRpm - 50) {

        SmartDashboard.putBoolean("ShootReady", true);
      } else {
        SmartDashboard.putBoolean("ShootReady", false);
      }
    }
    if(m_oi.manualAimMode()){
      ManualTurretRotation();
    }
  }

  public void setShooterTargetSpeed(double speed) {
    m_shooterController.setSetpoint(speed);
  }

  public void setTargetSpeedToCalculatedSpeed() {
    double targetRpm = computeShooterVelocity();
    setShooterTargetSpeed(targetRpm);
  }
 
  // public void TurretRotatorLimits()
  // {
  // if (TurretRotatorEncoder.getPosition() >= 150)
  // {
  // TurretRotator.set(0);
  // }

  // if(TurretRotatorEncoder.getPosition() <= -150)
  // {
  // TurretRotator.set(0);
  // }

  // }

  // public void EncoderLimitTesting()
  // {
  // TurretRotatorEncoder = TurretRotator.getEncoder();
  // System.out.println(TurretRotatorEncoder.getPosition());

  // }

}
