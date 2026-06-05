# Read-only ROS/Odin Test Materials

Purpose: run image-topic subscription and OpenVINO inference timing only.

Allowed:
- Subscribe to camera/image topic.
- Record inference latency, processing rate, CPU/iGPU load, dropped frames, and memory use.
- Compare CPU, GPU, and AUTO OpenVINO device choices if available.

Forbidden:
- Do not publish flight-control commands.
- Do not send waypoints.
- Do not switch flight modes.
- Do not arm or unlock the aircraft.
- Do not start any ring traversal motion.
