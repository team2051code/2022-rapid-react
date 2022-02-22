package frc.robot;
import javax.swing.plaf.basic.BasicBorders.ToggleButtonBorder;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;



public class OI {
    public boolean toggleOn = false;
    boolean togglePressed = false;

    private XboxController controller = new XboxController(RobotMap.XboxControllerPort);
    boolean ToggleReady = controller.getYButton();
    

    public double GetDriverRawAxis(int axis){
    return controller.getRawAxis(axis);
    }
    
    public boolean GetAButton(){
    return controller.getAButton();    
    }
    public boolean GetBButton(){
    return controller.getBButton();
    }
    public boolean GetXButton(){
    return controller.getXButton();
    }
    public boolean GetYButton(){
    return controller.getYButton();    
    }
    public boolean GetLeftBumper(){
    return controller.getLeftBumper();    
    }
    public boolean GetRightBumper(){
    return controller.getRightBumper();
    }

    public void UpdateToggle()
    {
      


      if(controller.getYButton()){
        if(!togglePressed){
            toggleOn = !toggleOn;
            togglePressed = true;
        }
         }else{

        togglePressed = false;

         }

        }

      }





    

    

    

