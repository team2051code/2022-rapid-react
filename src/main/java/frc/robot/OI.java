package frc.robot;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;



public class OI {
    private XboxController controller = new XboxController(RobotMap.XboxControllerPort);

    public double GetDriverRawAxis(int axis){
    return controller.getRawAxis(axis);
    }
    
    public boolean GetAButton(){
    return controller.getAButton();    
    }
    public boolean GetBButton(){
    return controller.getBButton();
    }
    }
    

