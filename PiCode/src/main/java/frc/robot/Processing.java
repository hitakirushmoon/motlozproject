package frc.robot;

import java.util.ArrayList;
import java.util.Collections;
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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Processing {
    NetworkTable RPi = NetworkTableInstance.getDefault().getTable("/Raspberry Pi");
	NetworkTableEntry xEntry = RPi.getEntry("X");
    NetworkTableEntry yEntry = RPi.getEntry("Y");
    NetworkTableEntry distanceEntry = RPi.getEntry("distance");
    NetworkTableEntry modeEntry = RPi.getEntry("mode");
    
    CvSink cvSink = CameraServer.getInstance().getVideo();
	CvSource outputStream = CameraServer.getInstance().putVideo("RPi", 160, 120);
	Mat mat = new Mat();
    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
    
    List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    List<RotatedRect> rotated = new ArrayList<RotatedRect>();
    Mat hierarchy = new Mat();

	// double[] hue = {85.61150787545623, 202.78156996587032};
    // double[] sat = {168.16545049492404, 255.0};
    // double[] val = {117.71582442221882, 141.86006825938568};

    double[] hue = {25, 125, 25.89928366297441, 45.87030326140209};
    double[] sat = {50, 350, 197.21221928116228, 255.0};
    double[] val = {125, 275, 168.16548001851967, 255.0};

    int screenWidth = Main.resolutionWidth;
    int midScreen = screenWidth/2;

    int acceptableSize = 100 * screenWidth/640;

    int centerX = 0, centerY = 0;

    final double heightCvtConst = 7000 * screenWidth * 9/16 / 640;
    double distFinal = 0;

    public void process(){
        if (modeEntry.getString("target").equals("cargo")){
            ballTrack();
        } else if (modeEntry.getString("target").equals("target")){
            targetTracking();
        }
    }

    public void resetNumbers(){
        centerX = -1;
        centerY = -1;
        contours.removeAll(contours);
        rotated.removeAll(rotated);

    }

    public void targetTracking(){

        resetNumbers();

        if (cvSink.grabFrame(mat) == 0) {
			outputStream.notifyError(cvSink.getError());
			return;
        }
        
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);
		Core.inRange(mat, new Scalar(hue[0], sat[0], val[0]), new Scalar(hue[1], sat[1], val[1]), mat);
		Imgproc.dilate(mat, mat, kernel);
        Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        
        for (int i = 0; i < contours.size(); i++){

            MatOfPoint contour = contours.get(i);
            Imgproc.putText(mat, "" + (int) contour.size().area(), new Point(Imgproc.boundingRect(contour).x, Imgproc.boundingRect(contour).y + 100), 0, 0.5, new Scalar(255, 255, 255));
           
            if (contour.size().area() < acceptableSize){
                contours.remove(i);
            } else {

                MatOfPoint2f dst = new MatOfPoint2f();  
                contour.convertTo(dst, CvType.CV_32F);
                RotatedRect rotRect = Imgproc.minAreaRect(dst);
                // Imgproc.putText(mat, "" + (int) rotRect.angle, new Point(rotRect.center.x, rotRect.center.y - 100), 0, 0.5, new Scalar(255, 255, 255));
                rotated.add(rotRect);
            }
        }


        for (int i = 0; i < rotated.size() - 1; i++){
            RotatedRect rot1 = rotated.get(i);
            RotatedRect rot2 = rotated.get(i + 1);
            
            if (rot1.angle < rot2.angle && rot1.angle != 0 && rot2.angle != 0){
                int avgX = (int) ((rot1.center.x + rot2.center.x) / 2);
                int avgY = (int) ((rot1.center.y + rot2.center.y) / 2);
                int avgH = (int) ((rot1.size.height + rot2.size.height) / 2);
                int avgW = (int) ((rot1.size.width + rot2.size.width) / 2);

                if (Math.abs(midScreen - avgX) < Math.abs(midScreen - centerX)){
                    centerX = avgX;
                    centerY = avgY; 
                }

                double dist = (6.5 * screenWidth) / (2 * avgH * Math.tan(70.42/2)) * 2.54; //need fixin
                distFinal = dist + (dist - heightCvtConst/avgH) * 2.45;
            }
        }

        if (centerX != -1){
            Point midPoint = new Point(centerX, centerY);
            // Imgproc.putText(mat, "" + distFinal, midPoint, 0 , 0.5 , new Scalar(255,255,255));
            drawCrosshair(mat, centerX, centerY);    
        }
       
        xEntry.setNumber(centerX);
		yEntry.setNumber(centerY);
        distanceEntry.setNumber(distFinal);        

        outputStream.putFrame(mat);
    }

    public void drawCrosshair(Mat mat, int centerX, int centerY){
        // Imgproc.rectangle(mat, new Point(centerX - 100, centerY - 50), new Point(centerX + 100, centerY + 50), new Scalar(255,255,255));
        Imgproc.line(mat, new Point(centerX - 100, centerY), new Point(centerX + 100, centerY), new Scalar(255,255,255));
        Imgproc.line(mat, new Point(centerX, centerY - 100), new Point(centerX, centerY + 100), new Scalar(255,255,255));
        Imgproc.rectangle(mat, new Point(centerX - 10, centerY - 10), new Point(centerX + 10, centerY + 10), new Scalar(255,255,255));
    }


    public void ballTrack(){
        resetNumbers();
        
        if (cvSink.grabFrame(mat) == 0) {
			outputStream.notifyError(cvSink.getError());
			return;
        }
        
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);
		Core.inRange(mat, new Scalar(hue[2], sat[2], val[2]), new Scalar(hue[3], sat[3], val[3]), mat);
		Imgproc.dilate(mat, mat, kernel);
        Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        
        for (int i = 0; i < contours.size(); i++){
            if (contours.get(i).size().area() < 20){
                contours.remove(i);
                continue;
            }
            Rect rect = Imgproc.boundingRect(contours.get(i));
            if (Math.abs(midScreen - rect.x) < Math.abs(midScreen - centerX)){
                centerX = rect.x;
            }

            double dist = (6.5 * screenWidth) / (2 * rect.height * Math.tan(70.42/2)) * 2.54;
            distFinal = dist + (dist - heightCvtConst/rect.height) * 2.45;
        }

        xEntry.setNumber(centerX);
		yEntry.setNumber(centerY);
        distanceEntry.setNumber(distFinal);        

        outputStream.putFrame(mat);
    }
}
