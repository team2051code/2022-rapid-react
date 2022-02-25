package frc.robot;
import javax.swing.plaf.basic.BasicBorders.ToggleButtonBorder;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;



public class OI {
    public boolean toggleOn = false;
    boolean togglePressed = false;

    private XboxController controller = new XboxController(RobotMap.XboxControllerPort);
    private XboxController controller2 = new XboxController(RobotMap.XboxControllerPort2);
    boolean ToggleReady = controller.getYButton();
    

    public double GetDriverRawAxis(int axis){
    return controller.getRawAxis(axis);
    }
    
    public boolean GetAButton2(){
    return controller2.getAButton();    
    }
    public boolean GetBButton(){
    return controller.getBButton();
    }
    public boolean GetXButton(){
    return controller2.getXButton();
    }
    public boolean GetYButton(){
    return controller.getYButton();    
    }
    public boolean GetLeftBumper(){
    return controller.getLeftBumper();    
    }
    
    public boolean GetLeftBumper2(){
      return controller2.getLeftBumper(); 
    }
    public boolean GetRightBumper(){
    return controller.getRightBumper();
    }
    
    public boolean GetRightBumper2(){
      return controller2.getRightBumper();
    }
    public double GetRightTrigger2(){
    return controller2.getRightTriggerAxis();
    }
    public boolean StickClick(){
    return controller.getRightStickButton();
    }
    public boolean GetXButton2(){
      return controller2.getXButton();
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





    

    

    

