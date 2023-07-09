// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.Drivetrain

class AutonomousDistance(drivetrain: Drivetrain) : SequentialCommandGroup() {
    /**
     * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
     * turn around and drive back.
     *
     * @param drivetrain The drivetrain subsystem on which this command will run
     */
    init {
        addCommands(
            DriveDistance(-0.5, 10.0, drivetrain),
            TurnDegrees(-0.5, 180.0, drivetrain),
            DriveDistance(-0.5, 10.0, drivetrain),
            TurnDegrees(0.5, 180.0, drivetrain)
        )
    }
}