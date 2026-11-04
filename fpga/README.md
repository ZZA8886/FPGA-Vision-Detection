# FPGA implementation

This directory contains the Gowin FPGA project for the Navbot controller.

The design combines mecanum motion control, encoder odometry, vision UART control, QR route execution, OV5647 MIPI capture, DDR3 frame buffering, HDMI output and IMU660RA yaw feedback. The IMU uses the BMI270 register configuration supplied by SeekFree and communicates over the board's `imu_scl`/`imu_sda` pins.

Open `navbot_camera.gprj` in Gowin IDE. Keep the relative paths in the project file unchanged when moving the repository.
