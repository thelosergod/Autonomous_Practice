package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.gobilda.pinpoint.GoBildaPinpointDriver;

public class Pinpoint {

    private GoBildaPinpointDriver driver;

    public Pinpoint(HardwareMap hardwareMap) {
        // initialize the GoBilda Pinpoint driver from hardwareMap
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    // update the Pinpoint readings
    public void update() {
        driver.update();
    }

    // get robot position (in inches)
    public double getX() {
        return driver.getPosX();
    }

    public double getY() {
        return driver.getPosY();
    }

    // get heading in radians
    public double getHeading() {
        return driver.getHeading();
    }
}
