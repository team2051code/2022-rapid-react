package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;


public class Drive extends CommandBase {
private DriveTrain M_DriveTrain;
public Drive(DriveTrain DriveTrain) {
M_DriveTrain = DriveTrain;


 
    
}


@Override public void initialize(){
M_DriveTrain.TestingMotors();


}

@Override public void execute(){
    double DesiredDistance = (87.42);
    double DistanceInInches = M_DriveTrain.GetEncoderInches();
    

// if(DesiredDistance >= DistanceInInches)
// {
// M_DriveTrain.tankDrive(0, 0);
// DesiredDistance = 20;
// }

// if(DesiredDistance <= DistanceInInches)
// {
// M_DriveTrain.tankDrive(.5, .5);
// }

SmartDashboard.putNumber("DistanceTraveled", M_DriveTrain.GetEncoderInches());


//System.out.println(M_DriveTrain.getLeftEncoderValue());


}

@Override public void end(boolean interrupt) {

M_DriveTrain.tankDrive(0, 0);

}
@Override public boolean isFinished() {

//Addition of the distances to group them together
double DesiredDistance = (87.42);
//Sets the distance to zero
double DistanceInInches = M_DriveTrain.GetEncoderInches();






//While our desired distance is greater than our current distance keep running the drive motors
return (DesiredDistance <= DistanceInInches);


    }
}




 

 
