// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;


public class LimelightAlign extends CommandBase {
  private Swerve s_Swerve;
  //private Translation2d translation;

  public static double turnKP = .33;
  public static double simpleFF = .05; //CHANGE ASAP, GET DECENT VALUE
  public static double threshold = .75; //REDUCE IF POSSIBLE
  public static double offset = 0;

  /** Creates a new LimelightAlign. */
  public LimelightAlign(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    addRequirements(RobotContainer.limelight);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double angleX = Limelight.getAngleX();

    double xAxis = (angleX * turnKP);
    
    SmartDashboard.putNumber("Angle Error Limelight", angleX * turnKP);
    if (Limelight.hasTarget()) {
      
      if (angleX > threshold) {
        s_Swerve.rotate(xAxis, true, true);
      }
      else if (angleX < -threshold) {
        s_Swerve.rotate(xAxis, true, true);
      }
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Limelight.aimedAtTarget();
  }
}
