package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "OpenCV bluebackstage")

public class opencvbluebackstage extends LinearOpMode {

    double cX = 0;
    double cY = 0;
    double width = 0;
    String detectedside="none";
    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 1920/3; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 1080/3; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels
    private DcMotorEx slideright =null;
    private DcMotorEx innerintake = null;
    private Servo midflip = null;
    private Servo rightflip = null;
    private Servo leftflip = null;
    private Servo minileft = null;
    private Servo miniright = null;
    private Servo rightlinkage = null;
    private PIDController controller;
    public static double p = 0.0065, i = 0, d = 0;
    public static double f = -0.1;
    public static int target = 0;
    private final double ticks = 384.5;
    public void outputpixel(){
        innerintake.setPower(0.8);
        sleep(1000);
        innerintake.setPower(0);
    }

    public void simpleslides() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slideright = hardwareMap.get(DcMotorEx.class, "rightslide");

        controller.setPID(p, i, d);
        int slidepos = slideright.getCurrentPosition();
        double pid = controller.calculate(slidepos, target);
        double ff = f;

        double power = pid + ff;
        target=-1000;
        while (!(slideright.getCurrentPosition()>(target+500))){
            //nothing
        }

    }

    @Override
    public void runOpMode() {
        rightlinkage=hardwareMap.get(Servo.class,"rightlinkage");
        innerintake=hardwareMap.get(DcMotorEx.class,"innerintake");
        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence left0 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0.00)))
                .forward(18)
                .splineToLinearHeading(new Pose2d(25,+6,Math.toRadians(40)),Math.toRadians(0))
                .build();

        TrajectorySequence left1 = drive.trajectorySequenceBuilder(left0.end())
                //.lineToSplineHeading(new Pose2d(20,30))
                .lineTo(new Vector2d(12,0))
                .splineToLinearHeading(new Pose2d(20,38,Math.toRadians(270)),Math.toRadians(0))

                .build();


        TrajectorySequence middle0 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0.00)))
                .forward(30)
                .build();

        TrajectorySequence middle1 = drive.trajectorySequenceBuilder(middle0.end())
                //.lineToSplineHeading(new Pose2d(20,30))
                .lineTo(new Vector2d(12,0))
                .splineToLinearHeading(new Pose2d(26,38,Math.toRadians(270)),Math.toRadians(0))

                .build();
        TrajectorySequence right0 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(18)
                .splineToLinearHeading(new Pose2d(25,-6,Math.toRadians(320)),Math.toRadians(0))
                                .build();
        TrajectorySequence right1 = drive.trajectorySequenceBuilder(right0.end())
                .lineTo(new Vector2d(12,0))
                .splineToLinearHeading(new Pose2d(32,38,Math.toRadians(270)),Math.toRadians(0))
                .build();
        Trajectory parking = drive.trajectoryBuilder(right1.end())
                        .strafeTo(new Vector2d(0,42))
                                .build();
        telemetry.addData("Detect Side:",detectedside);
        telemetry.update();
        waitForStart();
        rightlinkage.setPosition(0.4);
        if (opModeIsActive()) {

            controlHubCam.stopStreaming();
            if (detectedside=="middle"){
                drive.followTrajectorySequence(middle0);
                sleep(500);
                outputpixel();
                drive.followTrajectorySequence(middle1);
                detectedside="done";
            }
            if (detectedside=="right"){
                drive.followTrajectorySequence(right0);
                sleep(500);
                outputpixel();
                drive.followTrajectorySequence(right1);
                detectedside="done";

            }
            if (detectedside=="left"){
                drive.followTrajectorySequence(left0);
                sleep(500);
                outputpixel();
                drive.followTrajectorySequence(left1);
            }

            drive.followTrajectory(parking);
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Distance in Inch", (getDistance(width)));
            telemetry.update();

            // The OpenCV pipeline automatically processes frames and handles detection
        }

        // Release resources
        controlHubCam.stopStreaming();
    }

    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
    class YellowBlobDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                String mylabel = "Area: " + String.format("%2f",Imgproc.contourArea(largestContour));
                Imgproc.putText(input, mylabel, new Point(cX-30,cY+100),Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0,255,0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);
                if (Imgproc.contourArea(largestContour)>3500){
                    if (cX>350){
                        detectedside="right";
                    }
                    else{
                        detectedside="middle";

                    }
                }
                else {
                    detectedside="left";
                }
                telemetry.addData("Detect Side:",detectedside);
                telemetry.update();
            }
            Imgproc.putText(input, detectedside, new Point(cX-100,cY+100),Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0,255,0), 2);
            telemetry.addData("Detect Side:",detectedside);
            telemetry.update();
            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerYellow = new Scalar(0, 100, 50);
            Scalar upperYellow = new Scalar(40, 255, 255);


            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }


}
