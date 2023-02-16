import cv2
from  deepface import DeepFace
import numpy as np
cap=cv2.VideoCapture(0)
backends = [
  'opencv', 
  'ssd', 
  'dlib', 
  'mtcnn', 
  'retinaface', 
  'mediapipe'
]
p=0
face_objs = DeepFace.extract_faces(img_path = np.zeros((240,480,3)).astype(np.uint8), 
            target_size = (224, 224), 
            detector_backend = backends[1],enforce_detection=False) 
x,y,w,h=0,0,0,0
face=np.zeros((h,w)).astype(np.uint8)
while(True):
    ret,frame=cap.read()
    if p==0:
        face_objs = DeepFace.extract_faces(img_path = frame, 
            target_size = (224, 224), 
            detector_backend = backends[1],enforce_detection=False) 
        x=face_objs[0]['facial_area']['x']
        y=face_objs[0]['facial_area']['y']
        w=face_objs[0]['facial_area']['w']
        h=face_objs[0]['facial_area']['h']
        # face=face_objs[0]['face']
    # print(face_objs[0]['facial_area'])
    
    cv2.rectangle(frame,(x,y),(x+w,y+h),color=(255,0,0))
    # frame[y:y+h+1,x:x+w+1,:]+=face.astype(np.uint8)
    p=(p+1)%5
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()