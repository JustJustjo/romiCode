// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Drivetrain

class TurnDegrees(private val speed: Double, private val degrees: Double, private val drive: Drivetrain) :
    CommandBase() {
    /**
     * Creates a new TurnDegrees. This command will turn your robot for a desired rotation (in
     * degrees) and rotational speed.
     *
     * @param speed   The speed which the robot will drive. Negative is in reverse.
     * @param degrees Degrees to turn. Leverages encoders to compare distance.
     * @param drive   The drive subsystem on which this command will run
     */
    init {
        addRequirements(drive)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
        // Set motors to stop, read encoder values for starting point
        drive.romiArcadeDrive(0.0, 0.0)
        drive.resetEncoders()
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        drive.romiArcadeDrive(0.0, speed)
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
        drive.romiArcadeDrive(0.0, 0.0)
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        /* Need to convert distance travelled to degrees. The Standard
       Romi Chassis found here, https://www.pololu.com/category/203/romi-chassis-kits,
       has a wheel placement diameter (149 mm) - width of the wheel (8 mm) = 141 mm
       or 5.551 inches. We then take into consideration the width of the tires.
    */
        val inchPerDegree = Math.PI * 5.551 / 360
        // Compare distance travelled from start to distance based on degree turn
        return averageTurningDistance >= inchPerDegree * degrees
    }

    private val averageTurningDistance: Double
        private get() {
            val leftDistance = Math.abs(drive.inputs.leftEncoderDistanceInch)
            val rightDistance = Math.abs(drive.inputs.rightEncoderDistanceInch)
            return (leftDistance + rightDistance) / 2.0
        }
}