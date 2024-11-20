# Raspberry Pi 4 vs Raspberry Pi 5 Comparison

The following table outlines the key hardware differences between Raspberry Pi 4 Model B 8GB and Raspberry Pi 5 8GB across critical features that are <b>relevant</b> to our project:

| **Feature**             | **Raspberry Pi 4  Model B (8GB)**                                    | **Raspberry Pi 5 (8GB)**                                   | **Key Observations**                                                                                             |
|--------------------------|------------------------------------------------------|-----------------------------------------------------|-----------------------------------------------------------------------------------------------------------------|
| **Processor**           | Broadcom BCM2711 (Cortex-A72, Quad-core, 1.5 GHz)    | Broadcom BCM2712 (Cortex-A76, Quad-core, 2.4 GHz)   | - Up to **2x faster performance** for compute-intensive tasks.           |
| **RAM**                 | 8GB LPDDR4                              | 8GB LPDDR4X                                  | - LPDDR4X provides **higher bandwidth**, enhancing multitasking and data processing.                            |
| **GPU**                 | Broadcom VideoCore VI                                | Broadcom VideoCore VII                              | - Enhanced hardware-accelerated video encoding and 3D graphics.                                                 |
| **Storage Options**     | MicroSD only                                         | MicroSD and PCIe (NVMe SSD support)                 | - NVMe SSDs provide **much faster I/O speeds**, ideal for large datasets or applications.                       |
| **MicroSD R/W Speed**   | Limited by older controller                           | Improved SD controller with faster speeds           | - Up to **50% faster read/write speeds**, improving boot times and file handling.                               |
| **Camera Handling**     | `Picamera` library, limited features                  | `Picamera2` using `libcamera` stack                | - Lower latency and better support for high-resolution video streams.                                           |
| **Power Management**    | Basic power regulation                               | Advanced PMIC (Power Management IC)                 | - Improved stability when using multiple peripherals.                                                           |
| **Multitasking**        | Limited with multiple peripherals                    | Handles multitasking with ease                     | - Seamless operation, even with multiple peripherals (e.g., camera, servo, sensors).                            |

---

## Key Improvements in Raspberry Pi 5 (Both hardware and software)
- **Performance**: Upgraded CPU, GPU, RAM and microSD reader deliver significant performance boosts.
- **Camera Capabilities**: Improved camera handling with the new `Picamera2` library and `libcamera` stack, offering higher image quality and lower latency.
- **Operating System Optimization**: Raspberry Pi OS for Pi 5 is better optimized for 64-bit computing, enabling smoother multitasking, improved memory handling, and compatibility with newer software features.