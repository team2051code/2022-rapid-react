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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;

public class Pneumatics extends SubsystemBase {
  /** Creates a new Pneumatics. */
  Debouncer m_debouncer = new Debouncer(0.09, DebounceType.kRising);
  public OI m_oi = new OI();
  DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
  Solenoid m_singleSolenoid2 = new Solenoid(PneumaticsModuleType.REVPH, 0);
  Compressor m_pcmCompressor = new Compressor(1, PneumaticsModuleType.REVPH);

  public void gearShift() {
    if (m_debouncer.calculate(m_oi.stickClick())) {
      m_doubleSolenoid.set(Value.kReverse);
      m_doubleSolenoid.toggle();
    }
  }

  public void forwards() {
    if (m_oi.getBackButton2()) {
      m_singleSolenoid2.set(true);
    } else {
      m_singleSolenoid2.set(false);
    }

  }

  public void solenoidState() {
    Value state = m_doubleSolenoid.get();

    if (state == Value.kForward) {
      SmartDashboard.putBoolean("InLowGear", true);
    } else {
      SmartDashboard.putBoolean("InLowGear", false);

    }
  }
}
