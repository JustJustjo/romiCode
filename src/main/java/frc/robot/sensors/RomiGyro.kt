// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.sensors

import edu.wpi.first.hal.SimDevice
import edu.wpi.first.hal.SimDouble
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLog
import kotlin.math.absoluteValue

class RomiGyro: SubsystemBase() {
    private var simRateX: SimDouble? = null
    private var simRateY: SimDouble? = null
    private var simRateZ: SimDouble? = null
    private var simAngleX: SimDouble? = null
    private var simAngleY: SimDouble? = null
    private var simAngleZ: SimDouble? = null
    private var angleXOffset = 0.0
    private var angleYOffset = 0.0
    private var angleZOffset = 0.0

    val table = NetworkTableInstance.getDefault().getTable("Gyro")
    val angleZOffsetEntry = table.getEntry("Angle Z Offset")
    val angleZRawEntry = table.getEntry("Angle Z Raw")
    val angleZEntry = table.getEntry("Angle Z")

    /** Create a new RomiGyro.  */
    init {
        val gyroSimDevice = SimDevice.create("Gyro:RomiGyro")
        if (gyroSimDevice != null) {
            gyroSimDevice.createBoolean("init", SimDevice.Direction.kOutput, true)
            simRateX = gyroSimDevice.createDouble("rate_x", SimDevice.Direction.kInput, 0.0)
            simRateY = gyroSimDevice.createDouble("rate_y", SimDevice.Direction.kInput, 0.0)
            simRateZ = gyroSimDevice.createDouble("rate_z", SimDevice.Direction.kInput, 0.0)
            simAngleX = gyroSimDevice.createDouble("angle_x", SimDevice.Direction.kInput, 0.0)
            simAngleY = gyroSimDevice.createDouble("angle_y", SimDevice.Direction.kInput, 0.0)
            simAngleZ = gyroSimDevice.createDouble("angle_z", SimDevice.Direction.kInput, 0.0)
        } else {
            simRateX = null
            simRateY = null
            simRateZ = null
            simAngleX = null
            simAngleY = null
            simAngleZ = null
        }
    }
    override fun periodic() {
        angleZRawEntry.setDouble(rawAngleZ)
        angleZOffsetEntry.setDouble(angleZOffset)
        angleZEntry.setDouble(angleZ)
    }

    /**
     * Get the rate of turn in degrees-per-second around the X-axis.
     *
     * @return rate of turn in degrees-per-second
     */
    val rateX: Double
        get() = simRateX?.get() ?: 0.0

    /**
     * Get the rate of turn in degrees-per-second around the Y-axis.
     *
     * @return rate of turn in degrees-per-second
     */
    val rateY: Double
        get() = simRateY?.get() ?: 0.0

    /**
     * Get the rate of turn in degrees-per-second around the Z-axis.
     *
     * @return rate of turn in degrees-per-second
     */
    val rateZ: Double
        get() = simRateZ?.get() ?: 0.0

    /**
     * Get the currently reported angle around the X-axis.
     *
     * @return current angle around X-axis in degrees
     */
    val angleX: Double
        get() = if (simAngleX != null) {
            simAngleX!!.get() - angleXOffset
        } else 0.0

    /**
     * Get the currently reported angle around the X-axis.
     *
     * @return current angle around Y-axis in degrees
     */
    val angleY: Double
        get() = if (simAngleY != null) {
            simAngleY!!.get() - angleYOffset
        } else 0.0

    /**
     * Get the currently reported angle around the Z-axis.
     *
     * @return current angle around Z-axis in degrees
     */
    val rawAngleZ: Double
        get() = simAngleZ!!.get()
    val angleZ: Double
        get() = if (simAngleZ != null) {
            var rawGyroAngle = simAngleZ!!.get()
            if (((rawGyroAngle - angleZOffset) - prevAngleZ).absoluteValue < 0.06) {
                angleZOffset += ((rawGyroAngle - angleZOffset) - prevAngleZ)
                prevAngleZ = rawGyroAngle - angleZOffset
                -(rawGyroAngle - angleZOffset)//made it negative to work with Advantage Scope
            } else {
                prevAngleZ = rawGyroAngle - angleZOffset
                -(rawGyroAngle - angleZOffset)
            }
        } else 0.0
    var prevAngleZ = angleZ


    /** Reset the gyro angles to 0.  */
    fun reset() {
        if (simAngleX != null) {
            angleXOffset = simAngleX!!.get()
            angleYOffset = simAngleY!!.get()
            angleZOffset = simAngleZ!!.get()
            println("im getting called")
        }
    }
}