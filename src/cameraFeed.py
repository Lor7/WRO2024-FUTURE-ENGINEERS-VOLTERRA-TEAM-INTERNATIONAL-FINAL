from picamera2 import Picamera2
import time
import cv2
import numpy as np
import pickle
import zmq
from threading import Thread

def stop():
    while True:    
        try:
            message = recv_socket.recv_string()
            if message == "STOP":
                exit(0)
        except:
            pass

# Set up the ZeroMQ context and socket
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("ipc:///tmp/frame")  # Bind to a TCP port

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(raw = {"size":(1280,960)}, main ={"format":'RGB888', "size": (640, 480)}))
picam2.set_controls({"FrameRate": 15.0})
picam2.start()
while True:
    frame = picam2.capture_array()
    _, buffer = cv2.imencode('.jpg', frame)
    frame_data = pickle.dumps(buffer)
    socket.send(frame_data)
    #cv2.imshow("Output", frame)
    if cv2.waitKey(1) == ord('q') & 0xFF:
        cv2.destroyAllWindows()
        break
    
picam2.stop()
picam2.close()
socket.close()
context.term()
