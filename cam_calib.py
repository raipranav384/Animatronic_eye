import numpy as np
import cv2 
import glob

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# url=0
url="http://192.168.20.46:81/stream"
cap=cv2.VideoCapture(url)
i=0
speed=12
size=(7,9)
objp = np.zeros((size[0]*size[1],3), np.float32)
objp[:,:2] = np.mgrid[0:size[1],0:size[0]].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

gray=np.zeros((320,320,3)).astype(np.uint8)
while(True):
    ret,frame=cap.read()
    i+=1
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    ret, corners = cv2.findChessboardCorners(gray, size, None)
    # If found, add object points, image points (after refining them)
    # print(fname,ret)
    if ret == True and i%speed==0:
        print("Found!")
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        cv2.drawChessboardCorners(frame,size, corners2, ret)
        # cv2.imshow('img', frame)
        cv2.waitKey(500)
    
    key=cv2.waitKey(1)
    # key=cv2.waitKey(1)
    # if key&0xFF==32:
    #     cv2.imwrite(f'./captures/{i}.png',frame)
    #     i+=1
    # frame=cv2.flip(frame,1)
    cv2.imshow('frame',frame)
    if key&0xFF==ord('q'):
        break
    else:
        pass
cap.release()
cv2.destroyAllWindows()

# imgs=glob.glob('./captures/*')
# for frame_pth in imgs:
#     frame=cv2.imread(frame_pth)
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
#     ret, corners = cv2.findChessboardCorners(gray, size, flags=cv2.CALIB_CB_ADAPTIVE_THRESH)
#     # If found, add object points, image points (after refining them)
#     # print(fname,ret)
#     if ret == True:
#         print("Found!")
#         objpoints.append(objp)
#         corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
#         imgpoints.append(corners2)
#         # Draw and display the corners
#         cv2.drawChessboardCorners(frame,size, corners2, ret)
#         # cv2.imshow('img', frame)
#         cv2.waitKey(500)
# cv2.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error
print( "total error: {}".format(mean_error/len(objpoints)) )

np.save('cam_int.npy',mtx)
np.save('cam_dist.npy',dist)