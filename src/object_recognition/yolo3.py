import cv2
import numpy as np
import time

class ObjectDetector:
    def __init__(self):
        # Initialize YOLO
        self.net = cv2.dnn.readNet(
            "yolov3.weights",  
            "yolov3.cfg"
        )
        # self.net = cv2.dnn.readNet(
        #    "yolov3-tiny.weights",
        #     "yolov3-tiny.cfg"
        #     )
        
        # Load classes
        with open("coco.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
        
        # Set up network parameters
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))
        print(self.output_layers)
        # Detection parameters
        self.conf_threshold = 0.3
        self.nms_threshold = 0.5

    def detect_objects(self, frame):
        height, width, _ = frame.shape
        
        # Prepare image for YOLO
        blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        
        # Get detections
        outs = self.net.forward(self.output_layers)
        
        # Information for drawing
        class_ids = []
        confidences = []
        boxes = []

        # Process detections
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                
                if confidence > self.conf_threshold:
                    # Object detected
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    # Rectangle coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Apply Non-Max Suppression
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_threshold, self.nms_threshold)
        
        # Draw boxes
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(self.classes[class_ids[i]])
                confidence = confidences[i]
                color = self.colors[class_ids[i]]
                
                # Draw box
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                
                # Draw label
                cv2.putText(frame, f"{label} {confidence:.2f}", 
                           (x, y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 
                           0.5, color, 2)
        
        return frame

def main():
    # Initialize camera
    cap = cv2.VideoCapture(0)  # ZED camera index
    detector = ObjectDetector()
    
    # Set camera parameters (adjust for ZED camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    # FPS calculation
    fps_start_time = 0
    fps = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        # Calculate FPS
        fps_end_time = time.time()
        time_diff = fps_end_time - fps_start_time
        if time_diff > 1.0:
            fps = 1.0 / time_diff
            fps_start_time = fps_end_time
            
        # Process frame
        processed_frame = detector.detect_objects(frame)
        
        # Display FPS
        cv2.putText(processed_frame, f"FPS: {fps:.2f}", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                    1, (0, 255, 0), 2)
        
        # Display frame
        cv2.imshow("ZED Camera - Object Detection", processed_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()