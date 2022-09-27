// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
  public WPI_TalonFX shooterLeft = new WPI_TalonFX(Constants.SHOOTER_LEFT_CAN_ID);
  public WPI_TalonFX shooterRight = new WPI_TalonFX(Constants.SHOOTER_RIGHT_CAN_ID);

  public WPI_TalonFX frontStorageRoller = new WPI_TalonFX(Constants.FRONT_STORAGE_ROLLER_CAN_ID);
  public CANSparkMax backStorageRoller = new CANSparkMax(Constants.BACK_STORAGE_ROLLER_CAN_ID, MotorType.kBrushless);
  
  public DigitalInput storageLimitSwitch = new DigitalInput(3);

  /** Creates a new Shooter. */
  public Shooter() {
    configMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("LimitSwitch", storageLimitSwitch.get());
    SmartDashboard.putNumber("Shooter Speed", shooterLeft.getSelectedSensorVelocity());
  }

  public void spinShooter() {
    shooterLeft.set(-.55); //-.55ish was okay for tarmac shots; -.6 for past tarmac line 
  }
  public void shooterRestingSpeed() {
    shooterLeft.set(-.25);
  }
  public void stopShootProcess() {
    stop();
    stopFrontStorage();
    stopBackStorage();
    RobotContainer.hood.stop();
  }
  public void ShootProcess() {
    spinShooter();
    spinFrontStorage(-.75);
    spinBackStorage(.5);
  }
  public void stop() {
    shooterLeft.set(0);
    //shooterRight.set(0);
  }

  public void spinShooterClosedLoop(double velocity) {
    shooterLeft.set(ControlMode.Velocity, velocity);
    spinStorage(-.75, 1);
  }

  public void spinFrontStorage(double frontSpeed) {
    frontStorageRoller.set(-.75);
  }

  public void stopFrontStorage() {
    frontStorageRoller.set(0);
  }

  public void spinBackStorage(double speed) {
    backStorageRoller.set(speed);
  }

  public void stopBackStorage() {
    backStorageRoller.set(0);
  }

  public void spinStorage(double frontSpeed, double backSpeed) {
    spinFrontStorage(frontSpeed);
    spinBackStorage(backSpeed);
  }



  public void configMotors() {   
      shooterLeft.configFactoryDefault();
      shooterRight.configFactoryDefault();
      frontStorageRoller.configFactoryDefault();
      shooterRight.follow(shooterLeft);
      //shooterLeft.setInverted(true);
      shooterLeft.setNeutralMode(NeutralMode.Coast);
      shooterRight.setNeutralMode(NeutralMode.Coast);
      frontStorageRoller.setNeutralMode(NeutralMode.Brake);

      double kP = 0.4;
      double kI = 0;
      double kD = 0.15;
      double kF = 0.055;
      shooterLeft.config_kP(0, kP);
      shooterLeft.config_kF(0, kF);
      shooterLeft.config_kI(0, kI);
      shooterLeft.config_kD(0, kD);
  
      //shooterLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 20, 1));
      //shooterRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 20, 1));
  
      shooterLeft.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
      shooterLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
      shooterLeft.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 255);
      shooterLeft.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
      shooterLeft.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255);
      //shooterLeft.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10000);
      //shooterLeft.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 10000);
  

      shooterRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
      shooterRight.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 255);
      shooterRight.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
      shooterRight.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255);
      shooterRight.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 255);
      //shooterRight.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 10000);
      //shooterRight.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 10000);
  
      backStorageRoller.setIdleMode(CANSparkMax.IdleMode.kBrake);

  }

}
