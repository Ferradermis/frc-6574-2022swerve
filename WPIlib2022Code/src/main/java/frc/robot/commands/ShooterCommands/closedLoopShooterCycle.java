// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.HoodCommands.SetHoodToPosition;
import frc.robot.commands.ShooterCommands.BasicFunctions.SpinShooterClosedLoop;
import frc.robot.commands.StorageCommands.SpinFrontStorageBackward;
import frc.robot.commands.StorageCommands.SpinStorageForShooting;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class closedLoopShooterCycle extends SequentialCommandGroup {
  /** Creates a new closedLoopShooterCycle. */
  public closedLoopShooterCycle() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetHoodToPosition(-7850), //-7800 for auto and tarmac shots, -2000 for close up
      new SpinShooterClosedLoop(),
      new SpinFrontStorageBackward(),
      new WaitCommand (.25),
      new SpinStorageForShooting()
      );
  }
}
