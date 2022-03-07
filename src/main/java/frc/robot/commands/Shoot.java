// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.ShootParamaters;
import frc.robot.Subsystems.SingulatorInformation;

public class Shoot extends CommandBase {

  PIDController m_shooterController = new PIDController(0.000075, 0.00023, 0);

  private double m_expireTime;
  private double m_timeout;
  public DriveTrain m_driveTrain;
  public ShootParamaters m_shoot;
  public SingulatorInformation m_singulator;

  /** Creates a new Shoot. */
  public Shoot(ShootParamaters shootParameters, DriveTrain drivetrain, SingulatorInformation singulatorInformation) {
    m_shoot = shootParameters;
    m_driveTrain = drivetrain;
    m_singulator = singulatorInformation;
    addRequirements(m_shoot);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void calculatedShootSpeed() {

    double targetRpm = computeShooterVelocity();
    m_shooterController.setSetpoint(targetRpm);
    double measuredRpm = m_shoot.m_shooterLeft.getSelectedSensorVelocity();
    double outputValue = m_shooterController.calculate(measuredRpm);

    outputValue = Math.max(-1, Math.min(1, outputValue));

    m_shoot.shootSpeedRight(outputValue);
    m_shoot.shootSpeedLeft(outputValue);

    if (measuredRpm <= targetRpm + 50 && measuredRpm >= targetRpm - 50) {
      SmartDashboard.putBoolean("ShootReady", true);
      m_driveTrain.setIntakeSpeed();
      m_singulator.setSingulatorSpeed();

    } else {
      SmartDashboard.putBoolean("ShootReady", false);
    }

  }

  public double computeShooterVelocity() {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    // acceration due to gravity
    final double G = 9.8;
    // ratio between linerar and angular velocity
    final double wheelToBall = 2;
    // Height for testing
    final double testHeight = 1.5494;
    // distance from the ground to the Target
    final double targetHeight = 2.64;
    // distance from ground to limelight
    final double limeToGround = .679;
    // distance from the limelight to the shooter
    final double limeToShooter = 0;
    // angle of the ball shooter in degrees
    double shootangleD = 60;
    // angle between middle of limelight and target in degrees
    double limeangleD = ty + 35;
    // distance between ballshooter and target
    double distance = (((testHeight - limeToGround) / Math.tan(Math.toRadians((limeangleD)))) + limeToShooter) * 2;

    // speed of the ball needed to reach the target
    double ballspeed = Math.sqrt((distance * G) / Math.sin(2 * (Math.toRadians(shootangleD))));
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
    // SmartDashboard.putNumber("distance to target", distance);

    // TODO: boost output by 15% of max RPM (fudge factor)
    final double ENCODER_TICKS_PER_REVOLUTION = 2048;
    final double TENTHS_OF_A_SECOND_PER_MINUTE = 600;

    return (rpm * (ENCODER_TICKS_PER_REVOLUTION / TENTHS_OF_A_SECOND_PER_MINUTE));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(computeShooterVelocity());

    calculatedShootSpeed();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double targetRpm = m_shoot.computeShooterVelocity();
    double measuredRpm = m_shoot.m_shooterLeft.getSelectedSensorVelocity();

    return measuredRpm <= targetRpm + 50 && measuredRpm >= targetRpm - 50;
  }

}
