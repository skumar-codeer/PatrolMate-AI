PatrolMate AI — existing saved project code

FILES
- PatrolMate_Arduino_UNO_Autonomous.ino: saved Arduino UNO autonomous obstacle-avoidance sketch (L298N, front/rear HC-SR04, two servos, SoftwareSerial UART).
- ultrasoniccode_saved.ino: separately saved ultrasonic/motor test sketch. This may be the version used during individual sensor testing; do not assume it is the current uploaded sketch.
- arduniocodeworking_saved.ino: separately saved working motor-control sketch.
- PatrolMate_ESP32S3_Wireless.ino: saved ESP32-S3 Wi-Fi web server + UART command bridge. It embeds the same dashboard HTML in the firmware.
- PatrolMate_Wireless_Dashboard.html: standalone copy of the wireless control dashboard. When using the ESP32-S3's built-in web server, open the ESP32-S3 IP address; the embedded dashboard is served from that firmware.
- ESP32CAM_CameraWebServer/: ESP32-CAM camera web-server sketch and supporting headers/source files saved from the user's library.

IMPORTANT NOTES
1. These are the saved source files retrieved from the project library, not a claim that each has just been compiled or hardware-tested.
2. The saved Arduino autonomous sketch prints distances to USB Serial but its saved version does not forward the sensor telemetry line back to ESP32-S3. The ESP32-S3 dashboard can only show distances if the Arduino firmware sends them over UART in the format expected by the bridge.
3. The saved autonomous sketch does not contain the link-watchdog patch. Keep motor power disconnected or wheels raised while checking and integrating. Verify STOP, motor PWM zero, and a link-loss watchdog before movement tests.
4. ESP32-S3 UART wiring in the saved sketch: GPIO43 TX -> Arduino D12 RX; Arduino D13 TX -> ESP32-S3 GPIO44 RX through a 5V-to-3.3V level shifter; common GND. The saved Arduino sketch uses SoftwareSerial(12, 13) at 9600 baud.
5. The camera stream URL previously used in the project was http://10.217.18.110:81/stream, but the camera's IP may change. Use the current camera IP/stream endpoint.
6. The ESP32-CAM folder contains the camera source plus its required supporting files. Keep the folder contents together in the Arduino IDE project directory; select the correct camera board configuration for your actual module.
7. Wi-Fi credentials are placeholders in the saved ESP32-S3 source. Enter your own local credentials before uploading. Never commit real passwords/tokens to a public GitHub repository.
8. No GitHub repository URL was present in the retrieved project context, so this package does not generate a QR code to a guessed repository. Once the repository URL is known, make the poster QR encode that exact URL.
