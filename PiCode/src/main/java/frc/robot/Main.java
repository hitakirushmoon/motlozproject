package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Main {
  
  public static int resolutionWidth = 640;
  public static void main(String[] args) {

    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
    NetworkTable RPi = NetworkTableInstance.getDefault().getTable("/Raspberry Pi");
    NetworkTableEntry widthResoEntry = RPi.getEntry("reso");

    System.out.println("Setting up NetworkTables client for team " + 6520);
    ntinst.startClientTeam(6520);


    UsbCamera cam = CameraServer.getInstance().startAutomaticCapture();
		cam.setResolution(resolutionWidth, resolutionWidth * 9/16);
    cam.setExposureManual(10);
    
    // Vision vision = new Vision();
    Processing proc = new Processing();

    while (true){ 
      // vision.process();
      widthResoEntry.setDouble(resolutionWidth);
      proc.process();
    }

  }

}
