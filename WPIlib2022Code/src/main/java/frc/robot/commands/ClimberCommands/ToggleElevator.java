// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ToggleElevator extends CommandBase {
  /** Creates a new ToggleElevator. */
  public ToggleElevator() {
  addRequirements(RobotContainer.climber);  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.climber.raiseClimberElevator();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.lowerClimberElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
