package frc.robot.commands2;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveJoystick extends CommandBase {
private XboxController M_Controller;
private MotorControllerGroup M_LeftSide;
private MotorControllerGroup M_RightSide;
private DifferentialDrive M_Drive;

public DriveJoystick(MotorControllerGroup LeftSide, MotorControllerGroup RightSide, DifferentialDrive Drive, XboxController controller) {

M_LeftSide = LeftSide;
M_RightSide = RightSide;
M_Drive = Drive;
M_Controller = controller;
    
}




@Override public void initialize(){


}

@Override public void execute(){
M_Drive.tankDrive(M_Controller.getRawAxis(1), M_Controller.getRawAxis(5));

}

@Override public void end(boolean interrupt) {
M_LeftSide.set(0);
M_RightSide.set(0);

}
@Override public boolean isFinished() {
return false;



}

private double encoderTicksPerInches(double ticks){

    //Possibility that GearRatio is actually 10.75
    final double GearRatio = (10.75);
    //Math to calculate the number of motor pulses based on our rotations
    
    final double PulsesPerInch = (2.0 * Math.PI) * 3 / GearRatio;
    //Math to calculate the current distance of the motor using the previous equation
     return(ticks * PulsesPerInch);

    }
}



 

 
