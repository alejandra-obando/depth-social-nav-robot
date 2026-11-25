# Cable for the Intel RealSense D415

## The problem

The D415 came with a **USB-C to USB-C** cable, but the **Raspberry Pi 4 has no USB-C data port**
(its USB-C port is only for power). Its data ports are USB-A: two blue (USB 3.0) and two black (USB 2.0).

So we need a cable **USB-C (male) on the camera side → USB-A (male) on the Pi side**, and it **must carry USB 3 data**.

## Why USB 3 matters

The D415 is a USB 3.1 Gen 1 camera (5 Gbps). On a USB 2.0 cable (480 Mbps) it still turns on, but the RealSense driver
falls back to USB 2 mode: lower resolution and frame rate, and often depth and colour cannot run together.
**Many cheap USB-C to USB-A cables are USB 2.0 "charging" cables** — that is the main trap.

## Checklist for a listing (Mercado Libre or any store)

The cable is good if **all** of these are true:

- [ ] Ends: **USB-C male** and **USB-A male**.
- [ ] The listing says **USB 3.0, USB 3.1 Gen 1, USB 3.2 Gen 1 or 5 Gbps** (or 10 Gbps, also fine).
- [ ] The USB-A end is **blue inside** or has the **"SS" (SuperSpeed)** logo — check the photos.
- [ ] Length **between 0.3 m and 1 m**. The camera is on the robot, so a short cable is enough and easier to route.
      Avoid more than 2 m (and extension cables).
- [ ] The USB-C plug is not very thick, so it fits the camera's port fully.

**Red flags — do not buy if the listing:**

- says **"USB 2.0"** or **"480 Mbps"**;
- only talks about **"carga rápida" / fast charging** and gives no data speed;
- gives no speed at all (ask the seller before buying);
- is a "charging only" or "solo carga" cable.

**Nice to have:** a right-angle (L-shaped) USB-C plug, braided cable, brand with a datasheet (Anker, Ugreen, Belkin, Baseus…).

### Alternative

An adapter **USB-C female → USB-A male, USB 3.0** would let us use the USB-C cable that came with the camera.
It works, but it adds a loose joint on a moving robot; the single cable is better.

## How to check it once we have it

1. Plug the D415 into a **blue** port of the Pi.
2. Run `lsusb -t`: the camera line must say **`5000M`** (USB 3). If it says `480M`, it is running as USB 2.
3. With librealsense installed, `rs-enumerate-devices` must show **`Usb Type Descriptor : 3.2`** (not `2.1`).
4. Try depth and colour together at 640 × 480, 30 fps in `realsense-viewer` or the ROS 2 node.
5. Check the Pi has enough power with both cameras on: `vcgencmd get_throttled` must return `0x0`.
