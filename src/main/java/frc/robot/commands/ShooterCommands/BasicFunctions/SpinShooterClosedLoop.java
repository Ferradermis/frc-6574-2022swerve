// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands.BasicFunctions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class SpinShooterClosedLoop extends CommandBase {

  boolean interrupted = false;
  /** Creates a new SpinShooterClosedLoop. */
  public SpinShooterClosedLoop() {
    addRequirements(RobotContainer.shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.shooter.spinShooterClosedLoop(Shooter.shooterSpeed);

    /*if (RobotContainer.shooter.shooterReady(Shooter.shooterSpeed)) {
      interrupted = true;
    } 
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.shooter.shooterReady(Shooter.shooterSpeed);
    //return interrupted && !RobotContainer.oi.operator_rightTrigger.get();
    }
}
