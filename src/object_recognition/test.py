import cv2

cap = cv2.VideoCapture(0)  # Replace 0 with the correct device ID for your ZED camera

while True:
    ret, frame = cap.read()
    if not ret:
        break
    cv2.imshow("ZED Camera", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
