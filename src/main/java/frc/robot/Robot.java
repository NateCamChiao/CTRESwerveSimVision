// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private final Vision vision;
    private final CommandSwerveDrivetrain swerve;
    private CommandJoystick controller = new CommandJoystick(0);

    public Robot() {
        vision = new Vision();
        swerve = new CommandSwerveDrivetrain(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft,
                TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
        vision.setSubsystemSuppliers(() -> swerve.getState().Pose.getRotation(), () -> swerve.getVisionlessDrivetrainPose(),
                swerve::addVisionMeasurement);

        double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        double MaxAngularRate = 3 / 2 * Math.PI;// 3/4 of a rotation per second max angular velocity
        SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 20% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        swerve.setDefaultCommand(
                // Drivetrain will execute this command periodically
                swerve.applyRequest(() -> drive.withVelocityX(-controller.getRawAxis(1) * MaxSpeed) // Drive forward with
                                                                                                 // negative Y (forward)
                        .withVelocityY(-controller.getRawAxis(0) * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-controller.getRawAxis(2) * MaxAngularRate * .5) // Drive counterclockwise
                                                                                           // with negative X (left)
                ));
        this.vision.disableCameras(true);

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        this.vision.disableCameras(true);
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
        this.vision.disableCameras(false); // re-enable
    }

    @Override
    public void autonomousInit() {

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
