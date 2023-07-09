// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.IO.DriveRealIO
import frc.robot.IO.DriveSimIO
import frc.robot.subsystems.Drivetrain
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
class Robot : LoggedRobot() {
    private var autonomousCommand: Command? = null
    private var robotContainer: RobotContainer? = null
    var drivetrain: Drivetrain = if (false) Drivetrain(DriveRealIO()) else Drivetrain(DriveSimIO())

    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    override fun robotInit() {
        Logger.getInstance().recordMetadata("ProjectName", "MyProject") // Set a metadata value


        if (/*isReal()*/true) {
            Logger.getInstance().addDataReceiver(WPILOGWriter("/home")) // Log to a USB stick
            Logger.getInstance().addDataReceiver(NT4Publisher()) // Publish data to NetworkTables
            PowerDistribution(1, PowerDistribution.ModuleType.kRev) // Enables power distribution logging





        } else {
            setUseTiming(false) // Run as fast as possible
            val logPath = LogFileUtil.findReplayLog() // Pull the replay log from AdvantageScope (or prompt the user)
            Logger.getInstance().setReplaySource(WPILOGReader(logPath)) // Read replay log
            Logger.getInstance().addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))) // Save outputs to a new log

        }

 Logger.getInstance().disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page

 Logger.getInstance().disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
        Logger.getInstance()
            .start() // Start logging! No more data receivers, replay sources, or metadata values may be added.

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = RobotContainer()
    }

    /**
     * This method is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     *
     * This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    override fun robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
//        NTClient.something.set(Drivetrain.leftEncoder.get().toDouble())

        CommandScheduler.getInstance().run()
    }

    /** This method is called once each time the robot enters Disabled mode.  */
    override fun disabledInit() {}
    override fun disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your [RobotContainer] class.  */
    override fun autonomousInit() {
        // Get selected routine from the SmartDashboard
        autonomousCommand = robotContainer!!.autonomousCommand

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand!!.schedule()
        }
    }

    /** This method is called periodically during autonomous.  */
    override fun autonomousPeriodic() {}
    override fun teleopInit() {
        // This makes sure that the autonomous stops running which will
        // use the default command which is ArcadeDrive. If you want the autonomous
        // to continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand!!.cancel()
        }
    }

    /** This method is called periodically during operator control.  */
    override fun teleopPeriodic() {
        drivetrain?.romiArcadeDrive(-OI.driverController.leftStickY, -OI.driverController.rightStickX)
        if (OI.driverController.a) {
            println("resetting drive")
            drivetrain?.resetEncoders()
            drivetrain?.resetGyro()
            drivetrain?.resetOdom()
        }
    }
    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
    }

    /** This method is called periodically during test mode.  */
    override fun testPeriodic() {}
}