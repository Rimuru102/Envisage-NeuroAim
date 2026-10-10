"""Make demo.mp4: the sample hand slides right at 0.4 m/s on a 2 m plane (60 fps, 3 s)."""
import cv2, numpy as np
im = cv2.imread("assets/hand.jpg")[560:800, 0:300]          # crop around the hand
FPS, Z, FX, V = 60, 2.0, 615.0, 0.4
w = cv2.VideoWriter("demo.mp4", cv2.VideoWriter_fourcc(*"mp4v"), FPS, (848, 480))
for i in range(3 * FPS):
    x = int(424 + FX * (-0.5 + V * i / FPS) / Z - im.shape[1] / 2)  # world x from -0.5 m
    c = np.full((480, 848, 3), 90, np.uint8)
    c[120:120 + im.shape[0], x:x + im.shape[1]] = im
    w.write(c)
w.release()
