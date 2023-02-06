package frc.robot.System;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.component.DriveBase;

public class NewAutoEngine {

  static int currentStep = 0;
  static int trajectoryAmount; //set the amount of the trajectory
  static String[] trajectoryJSON = {};
  //input the json files of the trajectory

  static Trajectory[] trajectory = new Trajectory[trajectoryAmount];

  public static Timer timer = new Timer();
  protected static SendableChooser<String> chooser;
  protected static String autoSelected;
  protected static final String DoNothing = "Do Nothing";

  public static void init() {
    chooser = new SendableChooser<String>();
    chooserSetting();
    for (int i = 0; i < trajectoryAmount; i++) {
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON[i]);
        // Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectorySIM[i]);//for simulation
        trajectory[i] = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON[i] + "\n" + ex.getMessage(),
            ex.getStackTrace());
        // DriverStation.reportError("Unable to open trajectory: " + trajectorySIM, ex.getStackTrace());//for simulation
      }

      var pose = trajectory[i].getInitialPose();

      DriveBase.setODOPose(pose);
    }
  }

  public static void start() {
    currentStep = 0;

    DriveBase.resetEnc();
    DriveBase.resetGyro();
    DriveBase.resetPIDs();
    autoSelected = chooser.getSelected();

    timer.reset();
    timer.start();
  }

  public static void loop() {
    DriveBase.updateODO();
    DriveBase.putDashboard();
    SmartDashboard.putNumber("Time", timer.get());
    switch (autoSelected) { //choose the trajectory
      case DoNothing:
      DriveBase.directControl(0, 0);
        break;
    }
  }

  private static void chooserSetting() {
    chooser.setDefaultOption("Do Nothing", DoNothing);
    SmartDashboard.putData("Auto Choice", chooser);
  }

  
}