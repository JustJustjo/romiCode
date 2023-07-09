// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Drivetrain

class DriveDistance(private val speed: Double, private val distance: Double, private val drive: Drivetrain) : CommandBase() {
    /**
     * Creates a new DriveDistance. This command will drive your robot for a desired distance at
     * a desired speed.
     *
     * @param speed  The speed at which the robot will drive
     * @param inches The number of inches the robot will drive
     * @param drive  The drivetrain subsystem on which this command will run
     */
    init {
        addRequirements(drive)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
        drive.romiArcadeDrive(0.0, 0.0)
        drive.resetEncoders()
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        drive.romiArcadeDrive(speed, 0.0)
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
        drive.romiArcadeDrive(0.0, 0.0)
    }

    // Returns true when the command should end.
//    override fun isFinished(): Boolean {
//        // Compare distance travelled from start to desired distance
//        return Math.abs(drive.averageDistanceInch) >= distance
//    }
}