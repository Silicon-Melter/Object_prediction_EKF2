from ultralytics import YOLO
import pyrealsense2 as rs
import cv2
import numpy as np

# โหลดโมเดล YOLO
model = YOLO('tunkkkuylek.pt').to("cuda")  # หรือ "cpu" ถ้าไม่มี GPU

# หาค่า id ของ sports ball
names_lower = {k: v.lower() for k, v in model.names.items()}
ball_id = next((k for k, v in names_lower.items() if v == 'sports ball'), 0)
if ball_id is None:
    raise ValueError('"sports ball" class not found in model.names!')

print(f"Filtering on class id {ball_id}: {model.names[ball_id]}")

# ตั้งค่ากล้อง RealSense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # ดึงภาพจากกล้องมาเป็น array
        frame = np.asanyarray(color_frame.get_data())

        # ใช้ YOLO ตรวจจับเฉพาะ sports ball
        results = model.predict(source=frame, conf=0.5, classes=[ball_id], verbose=False)

        # วาดกรอบและแสดง
        annotated = results[0].plot()
        cv2.imshow("YOLOv11 RealSense Sports Ball", annotated)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
