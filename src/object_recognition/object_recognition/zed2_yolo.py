import pyzed.sl as sl
import cv2
import torch
import numpy as np


class ZED2ObjectDetection:
    def __init__(self):
        # Initialize ZED camera
        self.zed = sl.Camera()

        # Create camera parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD1080
        init_params.camera_fps = 30
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE

        # Open the camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"Error opening camera: {err}")
            exit(1)

        # Load YOLO model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

        # Configure runtime parameters
        self.runtime_params = sl.RuntimeParameters()

        # Get camera information
        camera_info = self.zed.get_camera_information()
        self.image_width = camera_info.camera_configuration.resolution.width
        self.image_height = camera_info.camera_configuration.resolution.height

        # Create image mat
        self.image_zed = sl.Mat()

    def detect_objects(self):
        while True:
            if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                # Retrieve the left image
                self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT)

                # Convert ZED image to OpenCV format
                image_ocv = self.image_zed.get_data()

                # Convert from RGBA to RGB
                image_rgb = cv2.cvtColor(image_ocv, cv2.COLOR_RGBA2RGB)

                # Perform YOLO detection
                results = self.model(image_rgb)

                # Draw detections
                for detection in results.xyxy[0]:  # Changed from results.pred to results.xyxy
                    x1, y1, x2, y2, conf, cls = detection.cpu().numpy()
                    if conf > 0.5:  # Confidence threshold
                        label = f"{results.names[int(cls)]}: {conf:.2f}"

                        # Convert coordinates to integers
                        x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

                        # Draw bounding box
                        cv2.rectangle(image_rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)

                        # Add label
                        cv2.putText(image_rgb, label, (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Display the image
                cv2.imshow("ZED Object Detection", image_rgb)

                # Break loop with 'q'
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    def close(self):
        self.zed.close()
        cv2.destroyAllWindows()


def main():
    try:
        detector = ZED2ObjectDetection()
        detector.detect_objects()
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        if 'detector' in locals():
            detector.close()


if __name__ == "__main__":
    main()