package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="agooberdrive", group="Linear OpMode")
public class gooberdrive extends LinearOpMode {
    double linkagetarget=0;
    Boolean apressed=false;
    String ejectstate="none";
    private ElapsedTime runtime = new ElapsedTime();
    private IMU imu = null;
    private DcMotorEx leftFront = null;
    private DcMotorEx leftRear = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightRear = null;
    private DcMotorEx slideright =null;
    private DcMotorEx slideleft =null;
    private DcMotorEx outerintake=null;
    private DcMotorEx innerintake = null;
    private Servo midflip = null;
    private Servo rightflip = null;
    private Servo leftflip = null;
    private Servo rightlinkage = null;
    private Servo leftlinkage = null;
    private Servo minileft = null;
    private Servo miniright = null;
    double axial;
    double lateral;
    double yaw;
    double heading;
    double targetheading;
    double error;
    Boolean starturn;
    double righttarget=1;
    double lefttarget=0;
    double midtarget = 0.57;
    private PIDController controller;

    public static double p = 0.0065, i = 0, d = 0;
    public static double f = -0.1;

    public static int target = 0;

    private final double ticks = 384.5;

    private DcMotorEx rightslide;

    double timeToFlip = 0.01;
    Boolean toflip = false;
    String flip = "none";



    String turndirection;
    double turnpower;
    boolean dpadpressed=false;




    public void simpleslides(){
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rightslide = hardwareMap.get(DcMotorEx.class, "rightslide");

        controller.setPID(p, i, d);
        int slidepos = rightslide.getCurrentPosition();
        double pid = controller.calculate(slidepos, target);
        double ff = f;

        double power = pid + ff;

        slideright .setPower(-power);

        if (gamepad2.y) {
            target = -1750;
            rightflip.setPosition(0.48);
            leftflip.setPosition(0.52);
            midflip.setPosition(0.57);

        }

        if (gamepad2.b) {
            target = -1200;
            rightflip.setPosition(0.48);
            leftflip.setPosition(0.52);
            midflip.setPosition(0.57);
        }

        if (gamepad2.a) {
            target = -1000;
            rightflip.setPosition(0.48);
            leftflip.setPosition(0.52);
            midflip.setPosition(0.57);

        }
        if (gamepad2.x) {
            target = 0;
        }

        if (gamepad2.left_bumper){
            rightflip.setPosition(0.48);
            leftflip.setPosition(0.52);
            midflip.setPosition(0.57);
        }
        if (gamepad2.right_bumper){
            rightflip.setPosition(0.94);
            leftflip.setPosition(0.06);
            midflip.setPosition(0.55);
        }
    }
    public void dpadmovement(){
        if (gamepad1.dpad_down){
            axial=-0.2;
            lateral=0;
            yaw=0;
            dpadpressed=true;
        }
        if (gamepad1.dpad_right){
            lateral=0.3;
            axial=0;
            yaw=0;
            dpadpressed=true;
        }
        if (gamepad1.dpad_left){
            lateral=-0.3;
            axial=0;
            yaw=0;
            dpadpressed=true;
        }
        if (gamepad1.dpad_up){
            axial=0.2;
            lateral=0;
            yaw=0;
            dpadpressed=true;
        }
        if ( (!gamepad1.dpad_right)&&(!gamepad1.dpad_left)&&(!gamepad1.dpad_up)&&(!gamepad1.dpad_down)      ){
            dpadpressed=false;
        }
        telemetry.addData("Dpad Pressed",dpadpressed);

    }
    public void changetarget(){
        if (gamepad1.a &&(righttarget<1)){
            righttarget=righttarget+0.01;
        }
        if (gamepad1.b&&(righttarget>0)){
            righttarget=righttarget-0.01;
        }
        lefttarget=1-righttarget;

        rightflip.setPosition(righttarget);
        leftflip.setPosition(lefttarget);
        midflip.setPosition(midtarget);
    }





    public void updateheading() {
        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if (heading < 0) {
            heading = 360 + heading;

        }
        telemetry.addData("Heading",heading);

    }
    public void updatetargetheading()  {
        updateheading();
        if (gamepad1.left_bumper){
            targetheading=   ((Math.floor((heading/90)+0.015))  +1) * 90   ;
            starturn=true;
            turndirection="left";
        }
        if (gamepad1.right_bumper){
            targetheading=   ((Math.floor((heading/90)-0.1))  ) * 90   ;
            starturn=true;
            turndirection="right";
        }
        if (targetheading==360){
            targetheading=0;
        }
        if (targetheading<0){
            targetheading=360+targetheading;
        }
        if (targetheading>360){
            targetheading=targetheading-360;
        }
        telemetry.addData("Target Heading:",targetheading);
        //make divisor 90 for only cardinal directions
        //change offset to 0.6? for only diagonals
        // maybe have diagonals only bound when another button is pressed
        //.1 offset as of right now only gives 4.5 angles of leeway, increase to ~0.22 if using 8 locks and .11 if using only cardinal

    }

    public void turn(double power,String direction){
        if (direction=="right"){
            leftFront.setPower(power);
            leftRear.setPower(power);
            rightFront.setPower(-power);
            rightRear.setPower(-power);
        }
        if (direction=="left"){
            leftFront.setPower(-power);
            leftRear.setPower(-power);
            rightFront.setPower(power);
            rightRear.setPower(power);
        }
    }
    public void updateerror(){
        updateheading();
        error=Math.abs(heading-targetheading);
    }

    public void imuturn(){

        updatetargetheading();

        if (starturn){
            updateheading();


            if (targetheading!=0){
                error=Math.abs(heading-targetheading);

                while(error>1){
                    if (error>40){
                        turnpower=0.6;
                    }
                    else if (error>30){
                        turnpower=0.4;
                    }
                    else if (error>15){
                        turnpower=0.2;
                    }

                    turn(turnpower,turndirection);
                    updateheading();
                    error=Math.abs(heading-targetheading);
                }
                starturn=false;
            }
            if (targetheading==0){
                error=Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                while(error>1){
                    if (error>40){
                        turnpower=0.6;
                    }
                    else if (error>30){
                        turnpower=0.4;
                    }
                    else if (error>15){
                        turnpower=0.2;
                    }
                    turn(turnpower,turndirection);
                    updateheading();
                    error=Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                }
                starturn=false;
            }

        }

        //Now for homing:
        //Path 1: lock driver control, if bumper still pressed advance target with higher area of leeway (.6 probably)
        //Path 2: integrate into driver control, lock right stick while turning and use some big brain stuff to spline if possible.

    }
    public void flipperautomation(){

    }


    public void runOpMode() {

        imu=hardwareMap.get(IMU.class,"imu");
        leftFront=hardwareMap.get(DcMotorEx.class,"leftFront");
        leftRear=hardwareMap.get(DcMotorEx.class,"leftRear");
        rightFront=hardwareMap.get(DcMotorEx.class,"rightFront");
        rightRear=hardwareMap.get(DcMotorEx.class,"rightRear");

        outerintake=hardwareMap.get(DcMotorEx.class,"outerintake");
        slideleft=hardwareMap.get(DcMotorEx.class,"leftSlide");
        slideright=hardwareMap.get(DcMotorEx.class,"rightslide");
        innerintake=hardwareMap.get(DcMotorEx.class,"innerintake");
        rightflip=hardwareMap.get(Servo.class,"rightflip");
        leftflip=hardwareMap.get(Servo.class,"leftflip");
        midflip=hardwareMap.get(Servo.class,"midflip");
        leftlinkage=hardwareMap.get(Servo.class,"leftlinkage");
        rightlinkage=hardwareMap.get(Servo.class,"rightlinkage");

        miniright=hardwareMap.get(Servo.class,"miniright");
        minileft=hardwareMap.get(Servo.class,"minileft");


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();
        starturn=false;



        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            double max;
            double deadzone = 0.15;
            double modifier = 1;

            if (gamepad1.left_stick_y>deadzone){
                axial=-(gamepad1.left_stick_y*gamepad1.left_stick_y*modifier);
            }
            if (gamepad1.left_stick_y<-deadzone){
                axial=(gamepad1.left_stick_y*gamepad1.left_stick_y*modifier);
            }
            if ((java.lang.Math.abs(gamepad1.left_stick_y)<deadzone)&&!dpadpressed){
                axial=0.0;
            }

            if ((java.lang.Math.abs(gamepad1.left_stick_x)<deadzone)&&!dpadpressed){
                lateral=0.0;
            }

            if ((java.lang.Math.abs(gamepad1.right_stick_x)<deadzone)&&!dpadpressed){
                yaw=0.0;
            }


            if (gamepad1.left_stick_x>deadzone){
                lateral=(gamepad1.left_stick_x*gamepad1.left_stick_x*modifier);
            }
            if (gamepad1.left_stick_x<-deadzone){
                lateral=-(gamepad1.left_stick_x*gamepad1.left_stick_x*modifier);
            }


            if (gamepad1.right_stick_x>deadzone){
                yaw=(gamepad1.right_stick_x*gamepad1.right_stick_x*modifier);
            }
            if (gamepad1.right_stick_x<-deadzone){
                yaw=-(gamepad1.right_stick_x*gamepad1.right_stick_x*modifier);
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            if (gamepad1.y){
                imu.resetYaw();
            }

            leftFront.setPower(leftFrontPower);
            telemetry.addData("leftFrontPower",leftFrontPower);
            leftRear.setPower(leftBackPower);
            telemetry.addData("rightFront power:",rightFrontPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightBackPower);
            telemetry.addData("TIME",getRuntime());



            //updatetargetheading();
            //imuturn();
            //slides();
            //realslides();
            //analogencoderreading();
            //changetarget();
            simpleslides();
            //dpadmovement();
            // leftlinkage.setPosition(linkagetarget);
            if (gamepad1.right_trigger>0){
                linkagetarget=linkagetarget+0.05;
                while(gamepad1.right_trigger>0){
                    linkagetarget=linkagetarget;
                    telemetry.addData("Left Linkage Target",linkagetarget);

                }
            }
            if (gamepad1.left_trigger>0){
                linkagetarget=linkagetarget-0.05;
                while(gamepad1.left_trigger>0){
                    linkagetarget=linkagetarget;
                    telemetry.addData("Left Linkage Target",linkagetarget);
                }
            }

            updateheading();
            if (gamepad1.left_bumper){
                targetheading=   ((Math.floor((heading/90)+0.015))  +1) * 90   ;
                starturn=true;
                turndirection="left";
            }
            if (gamepad1.right_bumper){
                targetheading=   ((Math.floor((heading/90)-0.1))  ) * 90   ;
                starturn=true;
                turndirection="right";
            }
            if (targetheading==360){
                targetheading=0;
            }
            if (targetheading<0){
                targetheading=360+targetheading;
            }
            if (targetheading>360){
                targetheading=targetheading-360;
            }

            if (starturn){
                updateheading();


                if (targetheading!=0){
                    error=Math.abs(heading-targetheading);

                    while(error>1){
                        if (error>40){
                            turnpower=0.6;
                        }
                        else if (error>30){
                            turnpower=0.4;
                        }
                        else if (error>15){
                            turnpower=0.2;
                        }

                        turn(turnpower,turndirection);
                        updateheading();
                        error=Math.abs(heading-targetheading);
                    }
                    starturn=false;
                }
                if (targetheading==0){
                    error=Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                    while(error>1){
                        if (error>40){
                            turnpower=0.6;
                        }
                        else if (error>30){
                            turnpower=0.4;
                        }
                        else if (error>15){
                            turnpower=0.2;
                        }
                        turn(turnpower,turndirection);
                        updateheading();
                        error=Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                    }
                    starturn=false;
                }

            }
            if (gamepad1.dpad_down){
                axial=-0.2;
                lateral=0;
                yaw=0;
                dpadpressed=true;
            }
            if (gamepad1.dpad_right){
                lateral=0.3;
                axial=0;
                yaw=0;
                dpadpressed=true;
            }
            if (gamepad1.dpad_left){
                lateral=-0.3;
                axial=0;
                yaw=0;
                dpadpressed=true;
            }
            if (gamepad1.dpad_up){
                axial=0.2;
                lateral=0;
                yaw=0;
                dpadpressed=true;
            }
            if ( (!gamepad1.dpad_right)&&(!gamepad1.dpad_left)&&(!gamepad1.dpad_up)&&(!gamepad1.dpad_down)      ){
                dpadpressed=false;
            }

            if (gamepad2.dpad_up){
                innerintake.setPower(-1);
                outerintake.setPower(-0.4);
            }

            if (gamepad2.dpad_down){
                innerintake.setPower(1);
                outerintake.setPower(0.4);
            }

            if (gamepad2.dpad_left){
                innerintake.setPower(0);
                outerintake.setPower(0);
            }
            /*
            if (gamepad1.y){
                minileft.setPosition(0.3);
                miniright.setPosition(0.2);

            }

            if (gamepad1.a){
                minileft.setPosition(0.05);
                miniright.setPosition(0.05);
            }
            */
            if (gamepad1.a&&!apressed){
                if (minileft.getPosition()==0.05){
                    minileft.setPosition(0.05);
                    //miniright.setPosition(0.05);
                    flip="clamp";
                    timeToFlip=getRuntime()+0;
                }
                else{
                    minileft.setPosition(0.3);
                    //miniright.setPosition(0.2);
                    flip="unclamp";
                    timeToFlip=getRuntime()+0.25;
                }

                apressed=true;
                toflip=true;
            }
            if ((toflip)&&(getRuntime()>timeToFlip)){
                if (flip=="clamp;"){
                    miniright.setPosition(0.2);
                }
                else{
                    miniright.setPosition(0.05);
                }
                toflip=false;
            }
            if (!gamepad1.a){
                apressed=false;
            }
            rightlinkage.setPosition(linkagetarget);
            telemetry.addData("linkage target",linkagetarget);
            telemetry.addData("Axial",axial);
            telemetry.addData("Lateral",lateral);
            telemetry.addData("yaw",yaw);
            telemetry.update();

        }
    }

}
