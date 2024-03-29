// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.HoodCommands.SetHoodToPosition;
import frc.robot.commands.ShooterCommands.BasicFunctions.SpinShooter;
import frc.robot.commands.StorageCommands.SpinStorageForShooting;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BasicShooterCycle extends SequentialCommandGroup {
  /** Creates a new BasicShooterCycle. */
  public BasicShooterCycle() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new SetHoodToPosition(-7800), //-7800 for auto and tarmac shots, -2000 for close up
    new WaitCommand (.25),
    new SpinShooter(),
    new WaitCommand(.55), //replace with shooterReady logic
    new SpinStorageForShooting()
    );
  }
}
