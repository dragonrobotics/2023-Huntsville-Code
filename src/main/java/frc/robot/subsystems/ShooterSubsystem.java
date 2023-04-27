// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private CANSparkMax frontRotor = new CANSparkMax(14, MotorType.kBrushless);
  private CANSparkMax backRotor = new CANSparkMax(15, MotorType.kBrushless);

  public ShooterSubsystem() {
    backRotor.setInverted(true);
  }

  @Override
  public void periodic() {
  }

  public void setFrontRotorSpeed(double speed) {
    frontRotor.setVoltage(speed);
  }

  public void setBackRotorSpeed(double speed) {
    backRotor.setVoltage(-speed);
  }

  public void setRotorSpeed(double frontSpeed, double backSpeed) {
    frontRotor.setVoltage(frontSpeed);
    backRotor.setVoltage(backSpeed);
  }

  public void stop() {
    frontRotor.setVoltage(0);
    backRotor.setVoltage(0);
  }
}
