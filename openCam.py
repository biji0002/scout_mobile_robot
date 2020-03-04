import cv2

#print("Before URL")
cv2.namedWindow('output',cv2.WINDOW_NORMAL)
cv2.resizeWindow('output',720,480)
cap = cv2.VideoCapture('rtsp://10.7.5.105/h264major')

#print("After URL")
	
while True:

    #print('About to start the Read command')
    ret, frame = cap.read()
    frame = cv2.resize(frame,(360,240))
    #print('About to show frame of Video.')
    cv2.imshow("output",frame)
    cv2.waitKey(10)
    #print('Running..')
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
