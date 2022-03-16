package frc.robot.Subsystems;


import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.RobotMap;


public class Pneumatics extends SubsystemBase {
  /** Creates a new Pneumatics. */
  Debouncer m_debouncer = new Debouncer(0.09, DebounceType.kRising);
  public OI m_oi;
  DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(
    PneumaticsModuleType.REVPH, 
    RobotMap.GEARBOX_FORWARD_PCM_CHANNEL, 
    RobotMap.GEARBOX_REVERSE_PCM_CHANNEL);
  Solenoid m_SingleFirst = new Solenoid(PneumaticsModuleType.REVPH,
    RobotMap.INTAKE_DOWN_CHANNEL);
  Solenoid m_SingleSecond = new Solenoid(PneumaticsModuleType.REVPH,
    RobotMap.INTAKE_UP_CHANNEL);
  Compressor m_pcmCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  Timer m_timer = new Timer();
public Pneumatics(OI oi){

m_oi = oi;

}



  public void gearShift() {
    if (m_debouncer.calculate(m_oi.stickClick())) {
      m_doubleSolenoid.set(Value.kReverse);
      m_doubleSolenoid.toggle();
      
    }
    
  }

  public void AutonomousForward(){
    m_timer.start();

    if(m_timer.get() < 1){
    m_SingleFirst.set(true);
    m_SingleSecond.set(false);
  }
  else{
    m_SingleSecond.set(false);
    m_SingleFirst.set(false);
}
}
  
  


  public void forwards() {

    
    if(m_oi.getBackButton2())
    {
        m_timer.start();

        if(m_timer.get() < 1){
        m_SingleFirst.set(true);
        m_SingleSecond.set(false);
      }
      else{
        m_SingleSecond.set(true);
        m_SingleFirst.set(true);
    }
  }
      else{
        m_timer.stop();
        m_timer.reset();
        m_SingleSecond.set(true);
        m_SingleFirst.set(false);
    }
  }



    // if (m_oi.GetStartButton2()) {
    //   m_SingleFirst.set(true);
    // }
    //   else{
    //     if(m_oi.getBackButton2()){
    //       m_SingleSecond.set(true);
    //         }
    //         else{
    //           m_SingleFirst.set(false);
    //           m_SingleSecond.set(false);

    //         }
        //}
      



  // public void backwards() {
  // if (m_oi.GetStartButton2()) {
  // m_SingleSecond.set(true);
  // }
  // else{
  // m_SingleSecond.set(false);
  // }

  // }

  public void solenoidState() {
    Value state = m_doubleSolenoid.get();

    if (state == Value.kForward) {
      SmartDashboard.putBoolean("InLowGear", false);
    } else {
      SmartDashboard.putBoolean("InLowGear", true);

    }
  }
}
