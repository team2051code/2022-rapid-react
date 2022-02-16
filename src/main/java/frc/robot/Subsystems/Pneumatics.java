package frc.robot.Subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;

public class Pneumatics extends SubsystemBase {
  /** Creates a new Pneumatics. */
  public OI m_oi = new OI();
  DoubleSolenoid DoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
  Solenoid SingleSolenoid2 = new Solenoid(PneumaticsModuleType.REVPH, 0);
  Compressor pcmCompressor = new Compressor(1, PneumaticsModuleType.REVPH);

  public void GearShift(){
  if(m_oi.GetLeftTrigger())
  {
  DoubleSolenoid.set(Value.kReverse);
  DoubleSolenoid.toggle();
  }

  }
  

  



  public void forwards(){
    if(m_oi.GetXButton()){
    SingleSolenoid2.set(true);
  }
    else{
    SingleSolenoid2.set(false);  
    }
  }
 public void CompressorOnOrOff(){
  
  System.out.println(pcmCompressor.getPressure());

  if(pcmCompressor.getPressure() <= 60)
  {
    pcmCompressor.getPressure();
    pcmCompressor.enableDigital();
  }
  else
  {
  pcmCompressor.disable();
  }
 }
}
