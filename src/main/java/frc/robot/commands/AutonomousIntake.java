// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Pneumatics;
import frc.robot.Subsystems.ShootParamaters;
import frc.robot.Subsystems.SingulatorInformation;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousIntake extends SequentialCommandGroup {
  /** Creates a new RunIntake. */
  public AutonomousIntake(DriveTrain drivetrain, Pneumatics pneumatics, ShootParamaters shootParamaters, SingulatorInformation singulatorInformation) { super(
  
    new RunIntake(drivetrain, pneumatics),
      new Wait(1),
        new StopAll(drivetrain, pneumatics, shootParamaters, singulatorInformation)
  );
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}
