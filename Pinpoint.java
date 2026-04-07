package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.gobilda.pinpoint.GoBildaPinpointDriver;

public class Pinpoint {
    private GoBildaPinpointDriver driver;

    public Pinpoint(HardwareMap hardwareMap) {
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    public void update() {
        driver.update();
    }

    public double getX() {
        return driver.getPosX();
    }

    public double getY() {
        return driver.getPosY();
    }

    public double getHeading() {
        return driver.getHeading();
    }
}
