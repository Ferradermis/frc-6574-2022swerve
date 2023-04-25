// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeProcessEnd;
import frc.robot.commands.ShooterCommands.closedLoopShooterCycle;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAutoDriveCommands extends SequentialCommandGroup {
  double timeoutSeconds = 1.75;
  private Swerve s_Swerve;
  private boolean openLoop = true;
  /** Creates a new TwoBallAutoDriveCommands. */
  public TwoBallAutoDriveCommands(Swerve s_Swerve) {

    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    addRequirements(RobotContainer.limelight); {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new WaitCommand(1.5),
    new AutoSwerve(s_Swerve, -.25, 0, 0, false, openLoop),
    new WaitCommand(timeoutSeconds)
    );
  }
}
}
