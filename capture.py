import cv2
import numpy

cap=cv2.VideoCapture(0)
i=0
while(True):
    ret,frame=cap.read()
    frame=cv2.flip(frame,1)
    cv2.imshow('frame',frame)
    
    key=cv2.waitKey(1)
    if key&0xFF==32:
        cv2.imwrite(f'./captures/{i}.png',frame)
        i+=1
    elif key&0xFF==ord('q'):
        break
    else:
        pass
cap.release()
cv2.destroyAllWindows()