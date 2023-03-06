import cv2
from  deepface import DeepFace
import numpy as np
import serial
import argparse
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
parser = argparse.ArgumentParser(description ='Process some integers.')
parser.add_argument("--uno", default=False, action="store_true")
args=parser.parse_args()

# Arduino code
limits={'c':np.array([50,52]),
        'br':np.array([10,90]),
        'tr':np.array([65,90]),
        'tl':np.array([70,10]),
        'bl':np.array([0,0])}
limits2={'c':np.array([12,12]),'l':np.array([10,10]),'u':np.array([65,90])}
min_max_lims=np.array([[-9.3,9.8],[-0.3,0.7]])
print(min_max_lims)
if args.uno:
    arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=.1)
    
def write_data(angles):
    
    string=f"{angles[1]},{angles[0]}\n"
    arduino.write(bytes(string,'utf-8'))
    arduino.flush()
    

def process_angle(angles,limits):
    out=angles.copy()
    out[1]*=-1
    # angles+=limits['c']
    # idxs=angles<0
    # angles[idxs]=180-angles[idxs]
    # angles[0]=limits2['l'][0] if angles[0]<limits2['l'][0] else angles[0]
    # angles[1]=limits2['l'][1] if angles[1]<limits2['l'][1] else angles[1]

    # angles[0]=limits2['u'][0] if angles[0]>limits2['u'][0] else angles[0]
    # angles[1]=limits2['u'][1] if angles[1]>limits2['u'][1] else angles[1]
    out=(65-10)*(out-min_max_lims[:,0])/(min_max_lims[:,1]-min_max_lims[:,0])
    out+=limits['c']
    return out

p=0
speed=5
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
ret,frame=cap.read()
wt,ht=frame.shape[1],frame.shape[0]
# X=K_inv@mid_point
# angles_cent=(180*np.arctan(X/X[-1])/np.pi)[:-1]

while(True):
    ret,frame=cap.read()
    frame2=frame.copy()
    frame=cv2.flip(frame,1)
    if ret is not True or frame is None:
        continue
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
            angles=(180*np.arctan(X/X[-1])/np.pi)[:-1]
            raw_angle=angles.copy()
            angles=process_angle(angles,limits2)
            if args.uno:
                print('written:',angles.astype(np.int32))
                write_data(angles.astype(np.int32))
            print(raw_angle)
            # print(angles)
            f.seek(0)
            f.write(f"{angles[0]:1.3f},{angles[1]:1.3f}\r")
            f.flush()
    cv2.rectangle(frame,(x,y),(x+w,y+h),color=(255,0,0))
    cv2.circle(frame,center=(mid_point[0],mid_point[1]),radius=10,color=(255,0,0),thickness=10)
    # frame[y:y+h+1,x:x+w+1,:]+=face.astype(np.uint8)
    p=(p+1)%speed
    # frame=cv2.flip(frame,1)
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

f.close()
cap.release()
cv2.destroyAllWindows()