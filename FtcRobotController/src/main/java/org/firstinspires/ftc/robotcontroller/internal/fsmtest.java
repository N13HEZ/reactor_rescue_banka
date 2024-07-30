package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp

public class fsmtest extends LinearOpMode {
    private DcMotor rightslide;
    private Servo clamp;
    private Servo flip;
    public void runOpMode() throws InterruptedException {
        flip =hardwareMap.get(Servo.class,"flip");
        rightslide=hardwareMap.get(DcMotor.class,"rightslide");
        clamp=hardwareMap.get(Servo.class,"clamp");
        rightslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightslide.setTargetPosition(0);
        rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        String state = "waiting";

        waitForStart();
        while(opModeIsActive()){

            if (gamepad1.y){
                if (state=="waiting"){
                    rightslide.setTargetPosition(1000);
                    state="extending";
                }
            }
            if (state=="extending"){
                if (rightslide.getCurrentPosition()==1000){
                    state="extended";
                }
            }
            if (state=="extended"){
                flip.setPosition(0);
                state = "flipped";
            }
            if(state == "flipped"){
                clamp.setPosition(0);
                state = "released";
            }
            if (state == "released") {
                flip.setPosition(1);
                state = "unflipped";
            }
            if (state == "unflipped"){
                rightslide.setTargetPosition(0);
                state = "descending";
            }
            if(state == "descending"){
                if(rightslide.getCurrentPosition() == 0){
                    state = "descended";
                }
            }
        }
    }
}

