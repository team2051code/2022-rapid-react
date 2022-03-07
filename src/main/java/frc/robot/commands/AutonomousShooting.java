package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Pneumatics;
import frc.robot.Subsystems.ShootParamaters;
import frc.robot.Subsystems.SingulatorInformation;

public class AutonomousShooting extends SequentialCommandGroup {
   public AutonomousShooting(
       DriveTrain drivetrain, 
       Pneumatics pneumatics, 
       ShootParamaters shootParamaters, 
       SingulatorInformation singulatorInformation){
       super(
            new AlignShooter(shootParamaters),
            new SpinUpShooter(shootParamaters),
            // Shoot balls in singulator
              new StartSingulator(singulatorInformation),
              new Wait(2),
              new StopAll(
                  drivetrain, 
                  pneumatics, 
                  shootParamaters, 
                  singulatorInformation)
       );
   } 
}
