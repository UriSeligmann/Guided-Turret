import cv2
import numpy as np
from scipy.spatial import distance as dist

# Initialize the model (MobileNet-SSD)
prototxt = "deploy.prototxt"
model = "mobilenet_iter_73000.caffemodel"

# Check if model files exist
import os
if not os.path.isfile(prototxt):
    print(f"Prototxt file not found at {prototxt}")
    exit()
if not os.path.isfile(model):
    print(f"Model file not found at {model}")
    exit()

net = cv2.dnn.readNetFromCaffe(prototxt, model)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

# Labels of the classes in the model
CLASSES = ["background", "person"]

# Initialize the camera
camera = cv2.VideoCapture('http://192.168.1.102:81/stream', cv2.CAP_FFMPEG)
camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
if not camera.isOpened():
    print("Cannot open camera")
    exit()

print("Camera opened successfully")

# For assigning IDs to people
next_person_id = 0
tracked_persons = {}  # {person_id: (centroid_x, centroid_y, heading_x, heading_y)}
missing_frames = {}   # {person_id: number_of_missing_frames}
max_missing_frames = 10  # Threshold to remove lost tracks

frame_count = 0
detection_interval = 5  # Run detection every 5 frames

# Prediction update coefficient (how much we trust the heading prediction)
prediction_weight = 0.5  

def calculate_heading(centroid_old, centroid_new):
    """Calculate the movement direction (heading) vector."""
    return (centroid_new[0] - centroid_old[0], centroid_new[1] - centroid_old[1])

try:
    while True:
        ret, frame = camera.read()
        if not ret:
            print("Failed to grab frame")
            break

        frame_count += 1
        h, w = frame.shape[:2]

        if frame_count % detection_interval == 1:
            # Run detection
            blob = cv2.dnn.blobFromImage(
                cv2.resize(frame, (300, 300)),
                0.007843,
                (300, 300),
                127.5
            )
            net.setInput(blob)
            detections = net.forward()

            current_frame_persons = []  # [(centroid, bbox)]

            for i in range(detections.shape[2]):
                confidence = detections[0, 0, i, 2]
                if confidence > 0.8:  # Increased confidence threshold
                    idx = int(detections[0, 0, i, 1])
                    if idx == 15:
                        box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                        startX, startY, endX, endY = box.astype("int")
                        startX = max(0, startX)
                        startY = max(0, startY)
                        endX = min(w, endX)
                        endY = min(h, endY)

                        centroid = ((startX + endX) // 2, (startY + endY) // 2)
                        current_frame_persons.append((centroid, (startX, startY, endX, endY)))

            # Match current persons with tracked persons
            updated_tracked_persons = {}
            updated_missing_frames = {}

            if tracked_persons and current_frame_persons:
                object_ids = list(tracked_persons.keys())
                object_centroids = [v[:2] for v in tracked_persons.values()]

                input_centroids = [p[0] for p in current_frame_persons]

                D = dist.cdist(np.array(object_centroids), np.array(input_centroids))
                rows = D.min(axis=1).argsort()
                cols = D.argmin(axis=1)[rows]

                assigned_rows = set()
                assigned_cols = set()

                for (row, col) in zip(rows, cols):
                    if row in assigned_rows or col in assigned_cols:
                        continue
                    if D[row, col] > 50:  # Distance threshold
                        continue

                    person_id = object_ids[row]
                    centroid, bbox = current_frame_persons[col]
                    
                    # Update heading based on current and previous centroid
                    heading = calculate_heading(tracked_persons[person_id][:2], centroid)
                    updated_tracked_persons[person_id] = (*centroid, *heading)
                    updated_missing_frames[person_id] = 0  # Reset missing frames count

                    # Draw bounding box, ID, and heading on the frame
                    startX, startY, endX, endY = bbox
                    cv2.putText(frame, f"ID {person_id}", (startX, startY - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)

                    # Draw the heading direction vector
                    cv2.arrowedLine(frame, centroid, 
                                    (centroid[0] + int(heading[0] * 10), 
                                     centroid[1] + int(heading[1] * 10)),
                                    (255, 0, 0), 2)

                    assigned_rows.add(row)
                    assigned_cols.add(col)

                # Persons that have not been assigned
                for row in range(D.shape[0]):
                    if row not in assigned_rows:
                        person_id = object_ids[row]
                        missing_frames[person_id] = missing_frames.get(person_id, 0) + 1
                        if missing_frames[person_id] < max_missing_frames:
                            updated_tracked_persons[person_id] = tracked_persons[person_id]
                            updated_missing_frames[person_id] = missing_frames[person_id]
                        # Else, we drop this person_id from tracking

                # New detections
                for i in range(len(current_frame_persons)):
                    if i not in assigned_cols:
                        centroid, bbox = current_frame_persons[i]
                        updated_tracked_persons[next_person_id] = (*centroid, 0, 0)  # Initial heading (0,0)
                        updated_missing_frames[next_person_id] = 0

                        startX, startY, endX, endY = bbox
                        cv2.putText(frame, f"ID {next_person_id}", (startX, startY - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)

                        next_person_id += 1
            else:
                if current_frame_persons:
                    # First frame detections or no tracked persons
                    for centroid, bbox in current_frame_persons:
                        tracked_persons[next_person_id] = (*centroid, 0, 0)  # Initial heading (0,0)
                        missing_frames[next_person_id] = 0

                        startX, startY, endX, endY = bbox
                        cv2.putText(frame, f"ID {next_person_id}", (startX, startY - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)

                        next_person_id += 1

                    updated_tracked_persons = tracked_persons.copy()
                    updated_missing_frames = missing_frames.copy()
                else:
                    # No detections; increment missing frames for all tracked persons
                    for person_id in tracked_persons.keys():
                        missing_frames[person_id] = missing_frames.get(person_id, 0) + 1
                        if missing_frames[person_id] < max_missing_frames:
                            updated_tracked_persons[person_id] = tracked_persons[person_id]
                            updated_missing_frames[person_id] = missing_frames[person_id]
                        # Else, we drop this person_id from tracking

            tracked_persons = updated_tracked_persons.copy()
            missing_frames = updated_missing_frames.copy()
        else:
            # For frames without detection, just display tracked persons
            for person_id, (centroid_x, centroid_y, heading_x, heading_y) in tracked_persons.items():
                # Predict next position based on heading
                predicted_centroid = (int(centroid_x + heading_x * prediction_weight),
                                      int(centroid_y + heading_y * prediction_weight))

                # Draw predicted position and the ID
                cv2.circle(frame, predicted_centroid, 4, (0, 255, 0), -1)
                cv2.putText(frame, f"ID {person_id}", (predicted_centroid[0] - 10, predicted_centroid[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Draw arrow to indicate direction
                cv2.arrowedLine(frame, (centroid_x, centroid_y), 
                                (centroid_x + int(heading_x * 10), centroid_y + int(heading_y * 10)),
                                (255, 0, 0), 2)

        # Display the frame
        cv2.imshow('Frame', frame)

        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Exit key pressed")
            break

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    camera.release()
    cv2.destroyAllWindows()