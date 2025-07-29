import cv2
import os

# Path to your video file
video_path = "cap\WIN_20250729_20_51_03_Pro.mp4"  # <-- change this to your file name
cap = cv2.VideoCapture(video_path)

# Check if video opened successfully
if not cap.isOpened():
    print("Error: Cannot open video file.")
    exit()

# Create folder to save frames
save_folder = "video_frames"
os.makedirs(save_folder, exist_ok=True)

frame_count = 0

print("[INFO] Starting frame extraction...")

while True:
    ret, frame = cap.read()
    if not ret:
        break  # End of video

    # Save frame as image
    filename = os.path.join(save_folder, f"frame_{frame_count:05d}.jpg")
    cv2.imwrite(filename, frame)
    print(f"[SAVED] {filename}")
    frame_count += 1

# Cleanup
cap.release()
print(f"[INFO] Done. {frame_count} frames saved to '{save_folder}' folder.")
