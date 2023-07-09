// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.BuiltInAccelerometer
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.IO.DriveIO
import frc.robot.sensors.RomiGyro

class Drivetrain(val io: DriveIO) : SubsystemBase(), DriveIO {
    val inputs = DriveIO.DriveIOInputsAutoLogged()
    // The Romi has the left and right motors set to
    // PWM channels 0 and 1 respectively
    // The Romi has onboard encoders that are hardcoded
    // to use DIO pins 4/5 and 6/7 for the left and right

    val table = NetworkTableInstance.getDefault().getTable("Drivetrain")

    val headingEntry = table.getEntry("heading")
    val odometryEntry = table.getEntry("Odometry")

    val currentRotation = Rotation2d(inputs.gyroAngle * Math.PI/180.0)
    val odometry = DifferentialDriveOdometry(currentRotation, inputs.leftEncoderDistanceInch.inchesToMeters(), inputs.rightEncoderDistanceInch.inchesToMeters())

    /** Creates a new Drivetrain.  */
    init {
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.

        // Use inches as unit for encoder distances
        resetEncoders()
    }
    override fun periodic() {
        io.updateInputs(inputs)
        // This method will be called once per scheduler run
        headingEntry.setDouble(inputs.gyroAngle)
        odometry.update(Rotation2d(inputs.gyroAngle * Math.PI/180.0), inputs.leftEncoderDistanceInch.inchesToMeters(), inputs.rightEncoderDistanceInch.inchesToMeters())
        odometryEntry.setDoubleArray(doubleArrayOf(odometry.poseMeters.x, odometry.poseMeters.y, odometry.poseMeters.rotation.degrees))
    }

    fun romiArcadeDrive(xAxisSpeed: Double, zAxisRotate: Double) {
        io.arcadeDrive(xAxisSpeed, zAxisRotate)
    }

    fun resetEncoders() {
        io.reset()
    }

//    val leftEncoderCount: Int
//        get() = leftEncoder.get()
//    val rightEncoderCount: Int
//        get() = rightEncoder.get()
//    val leftDistanceInch: Double
//        get() = leftEncoder.distance
//    val rightDistanceInch: Double
//        get() = rightEncoder.distance
//    val averageDistanceInch: Double
//        get() = (leftDistanceInch + rightDistanceInch) / 2.0



    /** Reset the gyro.  */
    fun resetGyro() {
        io.gyroReset()
    }
    fun resetOdom() {
        odometry.resetPosition(Rotation2d(inputs.gyroAngle * Math.PI/180.0), 0.0, 0.0, Pose2d(0.0, 0.0, Rotation2d(0.0)))
    }


    fun Double.inchesToMeters(): Double {
        return this/39.37
    }

    companion object {
        private const val COUNTS_PER_REVOLUTION = 1440.0
        private const val WHEEL_DIAMETER_INCH = 2.75591 // 70 mm
        private const val WHEEL_CIRCUMFERENCE_METERS = 0.07 * Math.PI
    }

}