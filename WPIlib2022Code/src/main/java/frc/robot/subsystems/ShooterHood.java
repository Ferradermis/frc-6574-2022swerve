// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterHood extends SubsystemBase {
  public WPI_TalonFX hood = new WPI_TalonFX(Constants.SHOOTER_HOOD_CAN_ID, "rio");
  /** Creates a new ShooterHood. */
  public ShooterHood() {
    resetHood();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Hood Pos.", hood.getSelectedSensorPosition());
  }

  public void configMotors() {   
    hood.configFactoryDefault();
    hood.setNeutralMode(NeutralMode.Brake);
    //TalonFXInvertType kInvertType = TalonFXInvertType.CounterClockwise;
    //hood.setInverted(kInvertType);

    double kP = 1;
    double kI = 0.25;
    double kD = 0;
    double kF = 0.5; //was .05
    hood.config_kP(0, kP);
    hood.config_kF(0, kF);
    hood.config_kI(0, kI);
    hood.config_kD(0, kD);
  }

  public void setPosition(double desiredPosition) {
    hood.set(ControlMode.Position, desiredPosition);
  }

  public void stop() {
    hood.set(0);
  }

  public void stopZero() {
    hood.set(0);
    resetHood();
  }

  public void resetHood() {
    hood.setSelectedSensorPosition(0);
  }

  public void spinHoodOpenLoop() {
    hood.set(.15);
  }

  public double getHoodCurrent() {
    double hoodCurrent = hood.getSupplyCurrent();
    return hoodCurrent;
  }

  public void resetHoodAndZero() {
    
  }
} 
