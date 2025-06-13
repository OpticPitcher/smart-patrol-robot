from fastapi import FastAPI, HTTPException
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
from turtlebot_wrapper import TurtleBot3Wrapper
import uvicorn
from typing import Optional
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np
import threading
import time

app = FastAPI(title="TurtleBot3 Control Server")
robot = TurtleBot3Wrapper()

# Global variables for video streaming
bridge = CvBridge()
current_frame = None
frame_lock = threading.Lock()

def image_callback(msg):
    """Callback function to process incoming camera images"""
    global current_frame
    try:
        # Convert ROS image message to OpenCV format
        cv_image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        with frame_lock:
            current_frame = cv_image
    except Exception as e:
        rospy.logerr(f"Error processing image: {str(e)}")

# Subscribe to the camera topic
rospy.Subscriber("/camera/image/compressed", CompressedImage, image_callback)

def generate_frames():
    """Generator function to yield video frames"""
    while True:
        with frame_lock:
            if current_frame is not None:
                # Encode frame as JPEG
                ret, buffer = cv2.imencode('.jpg', current_frame)
                if ret:
                    frame_bytes = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        time.sleep(0.033)  # ~30 FPS

class MovementCommand(BaseModel):
    duration: Optional[float] = None
    speed: Optional[float] = 0.2  # Default speed in m/s

@app.get("/")
async def root():
    return {"status": "TurtleBot3 Control Server is running"}

@app.get("/video")
async def video_feed():
    """Stream video feed from the robot's camera"""
    return StreamingResponse(
        generate_frames(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@app.post("/move-forward")
async def move_forward(command: MovementCommand):
    try:
        robot.move(linear_vel=command.speed, duration=command.duration)
        return {"status": "Moving forward", "speed": command.speed}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/move-backward")
async def move_backward(command: MovementCommand):
    try:
        robot.move(linear_vel=-command.speed, duration=command.duration)
        return {"status": "Moving backward", "speed": -command.speed}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/turn-left")
async def turn_left(command: MovementCommand):
    try:
        robot.move(angular_vel=command.speed, duration=command.duration)
        return {"status": "Turning left", "speed": command.speed}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/turn-right")
async def turn_right(command: MovementCommand):
    try:
        robot.move(angular_vel=-command.speed, duration=command.duration)
        return {"status": "Turning right", "speed": -command.speed}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/stop")
async def stop():
    try:
        robot.stop()
        return {"status": "Stopped"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/status")
async def get_status():
    try:
        return robot.get_movement_state()
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000) 