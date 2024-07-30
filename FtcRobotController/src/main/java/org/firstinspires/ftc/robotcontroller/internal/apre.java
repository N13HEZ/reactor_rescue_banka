package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="ape", group="Linear OpMode")
public class apre extends LinearOpMode {
    private DcMotorEx leftslide;
    private DcMotorEx rightslide;
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()){
            leftslide=hardwareMap.get(DcMotorEx.class,"leftSlide");
            rightslide=hardwareMap.get(DcMotorEx.class,"rightslide");
            leftslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (gamepad1.a){
                leftslide.setPower(0.3);
                rightslide.setPower(-0.3);

            }
            if (gamepad1.b){
                leftslide.setPower(-0.3);
                rightslide.setPower(0.3);

            }
            if (gamepad1.y){
                leftslide.setPower(0);
                rightslide.setPower(0);
            }
        }

    }
}
