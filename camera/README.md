# Pothole Detector

## Raspberry Pi performance / GPU

This detector uses OpenCV DNN (YOLOv4-tiny). On Raspberry Pi, "GPU" acceleration is not guaranteed:

- Raspberry Pi does not support NVIDIA CUDA.
- OpenCV can run DNN on OpenCL if your OpenCV build has OpenCL enabled and a working OpenCL driver is present.
- If OpenCL is not available, the script falls back to CPU.

For real acceleration on Raspberry Pi, the usual path is an external accelerator (e.g. Coral USB Edge TPU, Intel NCS2) plus a model/runtime that supports it (typically TensorFlow Lite / OpenVINO).

## Running

CPU (default):

	./run_webcam.sh --headless --device /dev/video0

Try GPU/OpenCL when available:

	./run_webcam.sh --headless --device /dev/video0 --dnn-target auto
	./run_webcam.sh --headless --device /dev/video0 --dnn-target opencl
	./run_webcam.sh --headless --device /dev/video0 --dnn-target opencl-fp16

Set OpenCV threads (can help on Pi):

	./run_webcam.sh --headless --device /dev/video0 --opencv-threads 4
