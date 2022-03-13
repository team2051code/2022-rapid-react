package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OI extends SubsystemBase {
  public boolean m_toggleOn = false;
  boolean m_manualModeTogglePressed = false;

  private XboxController m_controller = new XboxController(RobotMap.XBOX_CONTROLLER_PORT);
  private XboxController m_controller2 = new XboxController(RobotMap.XBOX_CONTROLLER_PORT_2);
  boolean m_toggleReady = m_controller.getYButton();
  private boolean m_manualAimMode = false;

  public double getDriverRawAxis(int axis) {
    return m_controller.getRawAxis(axis);
  }

  public double GetTurretRotationAxis(){
    return m_controller.getRawAxis(RobotMap.Turning);
  }

  public boolean getAButton2() {
    return m_controller2.getAButton();
  }

  public boolean getBButton() {
    return m_controller.getBButton();
  }

  public boolean getXButton() {
    return m_controller.getXButton();
  }

  public boolean getYButton() {
    return m_controller.getYButton();
  }

  public boolean getLeftBumper() {
    return m_controller.getLeftBumper();
  }

  public boolean getLeftBumper2() {
    return m_controller2.getLeftBumper();
  }

  public boolean getRightBumper() {
    return m_controller.getRightBumper();
  }

  public boolean getRightBumper2() {
    return m_controller2.getRightBumper();
  }

  public double getRightTrigger2() {
    return m_controller2.getRightTriggerAxis();
  }

  public boolean stickClick() {
    return m_controller.getRightStickButton();
  }

  public boolean getXButton2() {
    return m_controller2.getXButton();
  }

  public boolean getBackButton2() {
    return m_controller2.getBackButton();
  }

  public boolean GetStartButton2(){
    return m_controller2.getStartButton();
  }
  public boolean GetAButton(){
    return m_controller.getAButton();
  }
  public boolean GetBButton2(){
    return m_controller2.getBButton();
  }
  


 
  
  public boolean manualAimMode(){
    return m_manualAimMode;

  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
   if(m_controller2.getLeftStickButtonPressed()){
     m_manualAimMode = !m_manualAimMode;
     
   }
   SmartDashboard.putBoolean("ManualAim?", m_manualAimMode);
  }
  

}
