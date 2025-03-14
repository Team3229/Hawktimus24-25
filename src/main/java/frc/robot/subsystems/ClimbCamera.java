package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

public class ClimbCamera {
    
    UsbCamera camera;

    public ClimbCamera() {
        camera = CameraServer.startAutomaticCapture("Climb Camera", 0);

        try {
            camera.setResolution(320, 240);
            camera.setFPS(20);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}
