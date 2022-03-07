// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Pneumatics;
import frc.robot.Subsystems.ShootParamaters;
import frc.robot.Subsystems.SingulatorInformation;

public class StopAll extends CommandBase {

  public Pneumatics m_pneumatics;
  public ShootParamaters m_shoot;
  public DriveTrain m_intakeMethod;
  public DriveTrain m_setIntakeSpeed;
  public DriveTrain m_driveTrain;
  public SingulatorInformation m_singulator;

  /** Creates a new StopAll. */
  public StopAll(DriveTrain drivetrain, Pneumatics pneumatics, ShootParamaters Shootparameters,
      SingulatorInformation singulatorInformation) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_singulator = singulatorInformation;
    m_shoot = Shootparameters;
    m_driveTrain = drivetrain;
    m_pneumatics = pneumatics;

    addRequirements(m_driveTrain, m_pneumatics, m_singulator, m_shoot);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    System.out.println("AllMotorsStopping");

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_shoot.setShooterTargetSpeed(0);
    m_driveTrain.tankDrive(0, 0);
    m_singulator.stopSingulator();
    m_driveTrain.StopIntake();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
