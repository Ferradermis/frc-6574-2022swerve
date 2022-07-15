package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class IntakeProcess extends CommandBase {
  /** Creates a new IntakeProcess. */
  public IntakeProcess() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.intake.deployIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.oi.driver_leftBumper.get()) {
      RobotContainer.intake.spin(-1);
    } else {
     RobotContainer.intake.spin(Constants.INTAKE_SPIN_SPEED);
     RobotContainer.intake.spinOmnis(Constants.OMNIS_SPIN_SPEED);
     if (!RobotContainer.shooter.storageLimitSwitch.get()) {
      RobotContainer.shooter.spinStorage(); //Constants.INTAKE_SPIN_SPEED
     }
     else {
       RobotContainer.shooter.spinFrontStorage();
     } //Constants.OMNIS_SPIN_SPEED
    }
 //Constants.FEEDER_OUTER_SPEED
     //if (no color sensor sensing) {
     //RobotContainer.shooter.spinInner(Constants.FEEDER_INNER_SPEED);
     //} else {
      //RobotContainer.shooter.stopInner();
     //}
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.stop();
    RobotContainer.intake.retractIntake();
    RobotContainer.intake.stopOmnis();
    RobotContainer.shooter.stopBackStorage();
    RobotContainer.shooter.stopFrontStorage();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}