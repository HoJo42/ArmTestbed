// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.CancelablePrintJob;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  private WPI_PigeonIMU pigeon;
  private DutyCycleEncoder encoder;
  private CANSparkMax winchMotor;
  private CANSparkMax rotatorMotor;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(WPI_PigeonIMU pigeon, DutyCycleEncoder encoder, CANSparkMax winchMotor, CANSparkMax rotatorMotor) {
    this.pigeon = pigeon;
    this.encoder = encoder;
    this.winchMotor = winchMotor;
    this.rotatorMotor = rotatorMotor;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("encoder count: ", encoder.get());
    SmartDashboard.putNumber("Pigeon Yaw: ", pigeon.getYaw());
    SmartDashboard.putNumber("Pigeon Pitch: ", pigeon.getPitch());
    SmartDashboard.putNumber("Pigeon Roll: ", pigeon.getRoll());
    SmartDashboard.putNumber("Motor Speed:", rotatorMotor.get());

    // This method will be called once per scheduler run
  }

  public void setRotatorSpeed(double spd){
    if(encoder.get() > ArmConstants.LOWER_LIMIT && encoder.get() < ArmConstants.UPPER_LIMIT){
      rotatorMotor.set(MathUtil.clamp(spd, -0.5, 0.5));
    }else{
      rotatorMotor.set(0);
    }
    
  }
}
