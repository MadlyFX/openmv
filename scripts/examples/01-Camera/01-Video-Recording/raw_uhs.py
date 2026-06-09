# This work is licensed under the MIT license.
# Copyright (c) 2013-2026 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# UHS Raw Video Recording Example
#
# This example requires an OpenMV N6 board, UHS-capable firmware, and a fast
# UHS-I microSD card mounted at /sdcard.
#
# RAW output has no container or header. Frames are stored consecutively as
# 8-bit Bayer images:
#
#     frame size = width * height bytes
#     frame N offset = N * frame size
#
# Use csi.GRAYSCALE instead of csi.BAYER for 8-bit grayscale output.

import csi
import machine
import video

FPS = 30
SECONDS = 10
FRAME_COUNT = FPS * SECONDS
PATH = "/sdcard/capture.raw"
WRITE_BUFFER = 1024 * 1024

csi0 = csi.CSI()
csi0.reset()
csi0.pixformat(csi.BAYER)
csi0.framesize(csi.HD)
csi0.framerate(FPS)
csi0.snapshot(time=2000)  # Allow exposure and white balance to settle.

width = csi0.width()
height = csi0.height()
frame_bytes = width * height  # Bayer and grayscale are one byte per pixel.
reserve_bytes = frame_bytes * FRAME_COUNT

print("Recording %d frames at %dx%d..." % (FRAME_COUNT, width, height))
print("Preallocating %d MiB..." % ((reserve_bytes + 1048575) // 1048576))

led = machine.LED("LED_RED")
recorder = video.Recorder(
    csi0,
    PATH,
    codec=video.RAW,
    fps=FPS,
    write_buffer=WRITE_BUFFER,
    preallocate=reserve_bytes,
)

try:
    led.on()
    status = recorder.record(frames=FRAME_COUNT)
finally:
    # close() flushes pending writes and trims an interrupted recording to its
    # actual byte length.
    recorder.close()
    led.off()

print("Saved:", PATH)
print("Frames:", status["frames"])
print("Dropped:", status["dropped"])
print("Bytes:", status["bytes"])
print("Actual FPS:", status["actual_fps"])
