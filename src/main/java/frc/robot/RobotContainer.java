// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmPID;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(ArmConstants.ENCODER_PORT);
  private final WPI_PigeonIMU armPigeon = new WPI_PigeonIMU(ArmConstants.PIGEON_PORT);
  private final CANSparkMax winchMotor = new CANSparkMax(ArmConstants.WINCH_PORT, MotorType.kBrushless);
  private final CANSparkMax rotateMotor = new CANSparkMax(ArmConstants.ROTATE_PORT, MotorType.kBrushless);
  private final DigitalInput limitSwitch = new DigitalInput(0);
  private final PIDController armPID = new PIDController(0.03, 0, 0);


  private ArmSubsystem m_ArmSubsystem = new ArmSubsystem(armPigeon, armEncoder, winchMotor, rotateMotor, limitSwitch);
  private ArmPID m_ArmPID = new ArmPID(armPID, m_ArmSubsystem, armPigeon);

  private final CANSparkMax clawMotor = new CANSparkMax(ClawConstants.INTAKE_MOTOR_PORT, MotorType.kBrushless);
  private final CANSparkMax clawMotorOther = new CANSparkMax(ClawConstants.INTAKE_MOTOR_PORT_OTHER, MotorType.kBrushless);
  private MotorControllerGroup clawMotors;

  private IntakeSubsystem m_IntakeSubsystem;
  private RunIntake m_RunIntake;

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    rotateMotor.setIdleMode(IdleMode.kBrake);
    clawMotorOther.setInverted(true);
    clawMotors = new MotorControllerGroup(clawMotor, clawMotorOther);
    m_IntakeSubsystem = new IntakeSubsystem(clawMotors);
    m_RunIntake = new RunIntake(m_IntakeSubsystem);
    // Configure the trigger bindings
    configureBindings();
    setDefaultCommands();
  }

  private void setDefaultCommands(){
    // m_ArmSubsystem.setDefaultCommand(m_ArmPID);
    // m_IntakeSubsystem.setDefaultCommand(m_RunIntake);
  }
  // new RunCommand(() ->{
  //   armPID.setSetpoint(0);
  //   double currentOutput = armPID.calculate(armPigeon.getRoll());
  //   SmartDashboard.putNumber("Current Output:", currentOutput);
  //   m_ArmSubsystem.setRotatorSpeed(currentOutput);
  // }, m_ArmSubsystem));//

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(new InstantCommand(() -> {
      winchMotor.setVoltage(3);
    }));
    m_driverController.b().whileFalse(new InstantCommand(() -> {
      winchMotor.setVoltage(0);
    }));
    m_driverController.a().whileTrue(new RunCommand(() -> {
      if(!limitSwitch.get()) {
        winchMotor.setVoltage(-2);
      } else {
        winchMotor.setVoltage(0);
        winchMotor.getEncoder().setPosition(0);
      }
      
    }));
    m_driverController.a().whileFalse(new InstantCommand(() -> {
      winchMotor.setVoltage(0);
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  public void coast(){
    rotateMotor.setIdleMode(IdleMode.kCoast);
  }
}
