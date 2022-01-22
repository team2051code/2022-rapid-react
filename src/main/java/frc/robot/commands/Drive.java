package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Drive extends CommandBase {
private MotorControllerGroup M_LeftSide;
private MotorControllerGroup M_RightSide;
private RelativeEncoder M_encoder;


public Drive(MotorControllerGroup LeftSide, MotorControllerGroup RightSide, RelativeEncoder encoder) {

M_encoder = encoder;
M_LeftSide = LeftSide;
M_RightSide = RightSide;

 
    
}


@Override public void initialize(){



}

@Override public void execute(){
M_LeftSide.set(.1);
M_RightSide.set(.1);
System.out.println("OnCommand");


}

@Override public void end(boolean interrupt) {
M_LeftSide.set(0);
M_RightSide.set(0);

}
@Override public boolean isFinished() {

//Addition of the distances to group them together
double DesiredDistance = (12);
//Sets the distance to zero
double DistanceInInches = encoderTicksPerInches(M_encoder.getPosition());

//While our desired distance is greater than our current distance keep running the drive motors
return (DesiredDistance <= DistanceInInches);



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



 

 
