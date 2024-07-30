package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Config
public class pidfSlides extends OpMode {
    private PIDController controller;

    public static double p = 0.0065, i = 0, d = 0;
    public static double f = -0.1;

    public static int target = 0;

    private final double ticks = 384.5;

    private DcMotorEx rightslide;


    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rightslide = hardwareMap.get(DcMotorEx.class, "rightslide");
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int slidepos = rightslide.getCurrentPosition();
        double pid = controller.calculate(slidepos, target);
        double ff = f;

        double power = pid + ff;

        rightslide.setPower(-power);

        telemetry.addData("pos", slidepos);
        telemetry.addData("target", target);
        telemetry.addData("ff", ff);
        telemetry.update();

        if (gamepad2.y) {
            target = -1800;
        }

        if (gamepad2.b) {
            target = -1200;
        }

        if (gamepad2.a) {
            target = -1000;

        }
        if (gamepad2.x) {
            target = 0;
        }

    }
}
