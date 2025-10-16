import cv2
from ultralytics import YOLO

## Display video feed flag for debugging.
display_video_feed = True

## Load trained model.
model = YOLO("best.pt")

## Open the selected camera.
video_camera = cv2.VideoCapture(6)

## Get the default frame width and height.
frame_width = int(video_camera.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(video_camera.get(cv2.CAP_PROP_FRAME_HEIGHT))

center_of_camera = frame_width / 2

## Since it is not really possible to center the robot perfectly,
## we must allow some room (threshold) around the center to be included.
## Center threshold in pixels.
center_threshold = 25

confidence_percentage = 0.65

if display_video_feed:

    while True:

        ret, frame = video_camera.read()
        if (not ret):
            break

        results = model.track(frame, stream=True, verbose=False)

        for result in results:
            class_names = result.names

            for box in result.boxes:
                if (box.conf[0] > confidence_percentage):
                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    cls = int(box.cls[0])
                    class_name = class_names[cls]
                    conf = float(box.conf[0])
                    color = (0, 0, 255) # Blue
                    
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

                    cv2.putText(frame, f"{class_name} {conf:.2f}",
                                (x1, max(y1 - 10, 20)), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, color, 2)

        ## The variable frame is the actual image after drawing the bounding boxes around the object.
        ## We only need to know the location coordinates of the horizontal center of the object
        ## so we can turn the robot to face in that direction.
        cv2.imshow("Camera Feed", frame)

        # Press 'q' to exit the loop
        if (cv2.waitKey(1) == ord('q')):
            break

else:

    ## THIS IS WHAT WILL RUN DURING AUTONOMOUS OPERATION.
    while True:

        ret, frame = video_camera.read()
        if (not ret):
            break

        results = model.track(frame, stream=True, verbose=False)

        for result in results:
            class_names = result.names

            for box in result.boxes:
                if (box.conf[0] > confidence_percentage):

                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    horizontal_center_of_cone = (x2 + x1) / 2
                    
                    if (horizontal_center_of_cone < center_of_camera - center_threshold):
                        ## TURN RIGHT.
                        pass
                    elif (horizontal_center_of_cone > center_of_camera + center_threshold):
                        ## TURN LEFT.
                        pass
                    elif (horizontal_center_of_cone > center_of_camera - center_threshold) and \
                         (horizontal_center_of_cone < center_of_camera + center_threshold):
                        ## STOP TURNING, YOU ARE CENTERED ON THE CONE.
                        pass

        # Press 'q' to exit the loop
        if (cv2.waitKey(1) == ord('q')):
            break


video_camera.release()
