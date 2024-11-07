import cv2

# List of camera stream URLs
stream_urls = [
    'http://192.168.0.10:81/stream',
    'http://192.168.0.14:81/stream',
]

# Initialize VideoCapture objects for each stream
caps = [cv2.VideoCapture(url, cv2.CAP_FFMPEG) for url in stream_urls]

# Check if all streams opened successfully
for i, cap in enumerate(caps):
    if not cap.isOpened():
        print(f"Cannot open stream {stream_urls[i]}")
        caps[i] = None  # Mark as None if unable to open

while True:
    for i, cap in enumerate(caps):
        # Skip this stream if it couldn't be opened
        if cap is None:
            continue

        # Read frame from each camera
        ret, frame = cap.read()
        if not ret:
            print(f"Failed to grab frame from stream {i}")
            continue

        # Display the frame
        window_name = f'ESP32-CAM Stream {i + 1}'
        cv2.imshow(window_name, frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release all VideoCapture objects and close windows
for cap in caps:
    if cap:
        cap.release()
cv2.destroyAllWindows()