package frc.robot.subsystems;

import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.LineSegmentDetector;
// import org.usfirst.frc.team6520.robot.RobotMap;
import org.opencv.imgproc.Moments;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class SS_VisionVer2 extends Subsystem {

    static double H = 130, S = 37, V = 55, HE = 40, SE = 20, VE = 30;
    static Mat mat = new Mat();
    public Thread m_visionThread;
    Preferences prefs = Preferences.getInstance();
    static final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(9, 9));

    public void vision() {

        m_visionThread = new Thread(() -> {
            int H = prefs.getInt("H", 130); // màu đỏ
            int S = prefs.getInt("S", 37); // màu lục
            int V = prefs.getInt("V", 55); // màu xanh lam
            int HE = prefs.getInt("HE", 40); // khoảng của đỏ
            int SE = prefs.getInt("SE", 20); // khoảng của lục
            int VE = prefs.getInt("VE", 30); // khoảng của xanh
            int focalLength = prefs.getInt("focalLength", 30); // khoảng của xanh

            SmartDashboard.putNumber("H", H);
            SmartDashboard.putNumber("S", S);
            SmartDashboard.putNumber("V", V);
            SmartDashboard.putNumber("HE", HE);
            SmartDashboard.putNumber("SE", SE);
            SmartDashboard.putNumber("VE", VE);

            SmartDashboard.putNumber("focalLength", focalLength);

            prefs.putDouble("start", SmartDashboard.getNumber("start", -2));
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
            camera.setResolution(640, 360);
            camera.setExposureManual(10);
            camera.setFPS(30);

            // khởi tạo CvSink là một vật thể host video từ cam
            CvSink cvSink = CameraServer.getInstance().getVideo();
            // khởi tạo vật thể để output processed feed đến smartdashboard
            CvSource outputStream = CameraServer.getInstance().putVideo("Blob", 640, 360);
            CvSource outputStream1 = CameraServer.getInstance().putVideo("Inflated", 640, 360);
            CvSource outputStream2 = CameraServer.getInstance().putVideo("Deflated", 640, 360);
            CvSource outputStream3 = CameraServer.getInstance().putVideo("Edge", 640, 360);
            // CvSource outputStream2 = CameraServer.getInstance().putVideo("Blob", 640,
            // 360);

            LinkedList<MatOfPoint> contours = new LinkedList<MatOfPoint>();

            while (!Thread.interrupted()) {
                contours.removeAll(contours);
                prefs.putInt("H", H);
                prefs.putInt("S", S);
                prefs.putInt("V", V);
                prefs.putInt("HE", HE);
                prefs.putInt("SE", SE);
                prefs.putInt("VE", VE);
                prefs.putInt("focalLength", focalLength);

                H = (int) SmartDashboard.getNumber("H", 130);
                S = (int) SmartDashboard.getNumber("S", 37);
                V = (int) SmartDashboard.getNumber("V", 55);
                HE = (int) SmartDashboard.getNumber("HE", 40);
                SE = (int) SmartDashboard.getNumber("SE", 20);
                VE = (int) SmartDashboard.getNumber("VE", 30);
                focalLength = (int) SmartDashboard.getNumber("focalLength", 30);
                if (cvSink.grabFrame(mat) == 0) {
                    outputStream.notifyError(cvSink.getError());
                    continue;
                }

                Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);
                Core.inRange(mat, new Scalar((H - HE) / 2, (S - SE) * 255 / 100, (V - VE) * 255 / 100),
                        new Scalar((H + HE) / 2, (S + SE) * 255 / 100, (V + VE) * 255 / 100), mat);
                outputStream.putFrame(mat);
                Imgproc.dilate(mat, mat, kernel);
                outputStream1.putFrame(mat);
                Imgproc.erode(mat, mat, kernel);
                outputStream2.putFrame(mat);
                Imgproc.Canny(mat, mat, 50, 150);
                outputStream3.putFrame(mat);
                // LineSegmentDetector a = Imgproc.createLineSegmentDetector();
                // Mat lines = new Mat();
                // a.detect(mat, lines);
                // SmartDashboard.putString("lines", lines);
                // a.drawSegments(mat, lines);

            }
        });
        m_visionThread.setDaemon(true);
        m_visionThread.start();

    }

    protected void initDefaultCommand() {

    }

}