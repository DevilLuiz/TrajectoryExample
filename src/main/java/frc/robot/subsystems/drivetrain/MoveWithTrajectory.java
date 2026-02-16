// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveWithTrajectory extends Command {
  private DriveTrain driveTrain;
  private Timer timer = new Timer();
  private Trajectory trajectory;
  private int targetId;

  public MoveWithTrajectory(DriveTrain driveTrain, int targetId) {
    this.driveTrain = driveTrain;
    this.targetId = targetId;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // gets the alliance color and adds an offset to the target id if the alliance is blue, since the blue and red targets are mirrored across the field. Also sets the trajectory to be reversed if the target is a station (ids 1-2 and 12-13)
    var allianceColor = DriverStation.getAlliance().orElse(Alliance.Red);
    boolean station = targetId <= 2;

    if(allianceColor == Alliance.Blue)
      targetId += 11;

    // gets the target pose from the AprilTagFieldLayout and the robot's current pose
    Pose2d targetPose = AprilTagFieldLayout
    .loadField(AprilTagFields.k2025ReefscapeAndyMark)
    .getTagPose(targetId).get().toPose2d();
    Pose2d robotPose = driveTrain.getPose();

    // generates a trajectory from the robot's current pose to the target pose with a max velocity of 3 m/s and a max acceleration of 2 m/s^2, and sets it to be reversed if the target is a station
    TrajectoryConfig config = new TrajectoryConfig(3, 2);
    config.setReversed(station);

    trajectory = TrajectoryGenerator.generateTrajectory(
      robotPose,
      List.of(),
      targetPose,
      config
    );

    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drives the robot with speeds relative to the trajectory at the current time
    driveTrain.driveWithRelativeSpeeds(trajectory, timer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    driveTrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // ends the command when the timer exceeds the total time of the trajectory
    return timer.get() >= trajectory.getTotalTimeSeconds();
  }
}
