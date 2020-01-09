package frc.robot;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;


import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Vision {

	public Thread m_visionThread;
	
	public void vision() {
		m_visionThread = new Thread(() -> {
		});
		m_visionThread.setDaemon(true);
		m_visionThread.start();
	}

	NetworkTable RPi = NetworkTableInstance.getDefault().getTable("/Raspberry Pi");


	CvSink cvSink = CameraServer.getInstance().getVideo();
	
	// CvSource outputStream = CameraServer.getInstance().putVideo("RPi", 160, 120);
	CvSource outputStream = CameraServer.getInstance().putVideo("Blob", 640, 360);
	CvSource outputStream1 = CameraServer.getInstance().putVideo("Inflated", 640, 360);
	CvSource outputStream2 = CameraServer.getInstance().putVideo("Deflated", 640, 360);
	CvSource outputStream3 = CameraServer.getInstance().putVideo("Edge", 640, 360);

	Mat mat = new Mat();
    static final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(9, 9));

	LinkedList<MatOfPoint> contours = new LinkedList<MatOfPoint>();
	
	Mat hierarchy = new Mat();

	double H = 130, S = 37, V = 55, HE = 40, SE = 20, VE = 30;



	
	public int process(){

		

		contours.removeAll(contours);

		if (cvSink.grabFrame(mat) == 0) {
			outputStream.notifyError(cvSink.getError());
			return 0;
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


		outputStream.putFrame(mat);

		return 1;
		}
	}
	