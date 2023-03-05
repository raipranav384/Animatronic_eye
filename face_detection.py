import cv2
from  deepface import DeepFace
import numpy as np

# url=0
url="http://192.168.20.46:81/stream"
cap=cv2.VideoCapture(url)
backends = [
  'opencv', 
  'ssd', 
  'dlib', 
  'mtcnn', 
  'retinaface', 
  'mediapipe'
]
p=0
speed=2
face_objs = DeepFace.extract_faces(img_path = np.zeros((240,480,3)).astype(np.uint8), 
            target_size = (224, 224), 
            detector_backend = backends[1],enforce_detection=False) 
x,y,w,h=0,0,0,0
face=np.zeros((h,w)).astype(np.uint8)

K=np.load('cam_int.npy')
K_inv=np.linalg.inv(K)
X=np.zeros((3,1))
mid_point=np.zeros((3,1))
# print('Intrinsic_shape:',K.shape)
f = open("angles.txt", "w+")
while(True):
    ret,frame=cap.read()
    if ret is not True or frame is None:
        continue
    frame=cv2.flip(frame,1)
    if p==0:
        face_objs = DeepFace.extract_faces(img_path = frame, 
            target_size = (224, 224), 
            detector_backend = backends[1],enforce_detection=False,align=False) 
        # print(len(face_objs))
        if len(face_objs)!=0:
            x=face_objs[0]['facial_area']['x']
            y=face_objs[0]['facial_area']['y']
            w=face_objs[0]['facial_area']['w']
            h=face_objs[0]['facial_area']['h']
            mid_point=np.array([x+w//2,y+h//2,1]).T
            X=K_inv@mid_point
            angles=np.arctan(X/X[-1])
            f.seek(0)
            f.write(f"{angles[0]:1.3f},{angles[1]:1.3f}\r")
            f.flush()
    cv2.rectangle(frame,(x,y),(x+w,y+h),color=(255,0,0))
    cv2.circle(frame,center=(mid_point[0],mid_point[1]),radius=10,color=(255,0,0),thickness=10)
    # frame[y:y+h+1,x:x+w+1,:]+=face.astype(np.uint8)
    p=(p+1)%speed
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

f.close()
cap.release()
cv2.destroyAllWindows()