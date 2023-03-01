// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private MotorControllerGroup intakeMotors;

  /** Creates a new Intake. */
  public IntakeSubsystem(MotorControllerGroup motors) {
    intakeMotors = motors;
  }

  public void runMotors(double speed) {
    intakeMotors.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Motor Speed:", intakeMotors.get());
  }

  
}
