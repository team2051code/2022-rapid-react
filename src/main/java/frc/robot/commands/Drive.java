package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;


public class Drive extends CommandBase {
private DriveTrain M_DriveTrain;
public Drive(DriveTrain DriveTrain) {
M_DriveTrain = DriveTrain;


 
    
}


@Override public void initialize(){


}

@Override public void execute(){

M_DriveTrain.tankDrive(.5, .5);
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




 

 
