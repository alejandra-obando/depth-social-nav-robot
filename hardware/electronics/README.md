# Electronics

Wiring diagrams, power distribution and pinout of the boards and sensors.

- [`usb_cable_d415.md`](usb_cable_d415.md) – which cable to buy to connect the D415 to the Pi 4.

## USB connections on the Pi 4

| Device | Pi 4 port | Why |
|--------|-----------|-----|
| Intel RealSense D415 | **Blue (USB 3.0)** | The depth stream needs USB 3 speed. |
| Logitech C270 | Black (USB 2.0) | It is a USB 2.0 camera; leaves the blue port free. |
| LiDAR / Dynamixel adapter | Black (USB 2.0) | Low bandwidth. |
