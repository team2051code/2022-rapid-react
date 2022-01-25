package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class DriveJoystick extends CommandBase {
private DriveTrain M_DriveTrain;
private XboxController M_Controller;
private DifferentialDrive M_Drive;

public DriveJoystick(DriveTrain driveTrain) {

M_DriveTrain = driveTrain;
   
}




@Override public void initialize(){


}

@Override public void execute(){
M_DriveTrain.tankDrive(M_Controller.getRawAxis(1), M_Controller.getRawAxis(5));


}

@Override public void end(boolean interrupt) {

}
@Override public boolean isFinished() {
return false;



    }
}



 

 
