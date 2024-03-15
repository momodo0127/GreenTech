import time
import cv2
import numpy as np 
from ArucoDetection_definitions import *
import robot_arm

start_time = time.time()

square_points=[[10,cv2.CAP_PROP_FRAME_HEIGHT-10], [cv2.CAP_PROP_FRAME_WIDTH-10,cv2.CAP_PROP_FRAME_HEIGHT-10], [cv2.CAP_PROP_FRAME_WIDTH-10, 10], [10,10]] #initial square

init_loc_1=[10,400]
init_loc_2=[400,400]
init_loc_3=[400,10]
init_loc_4=[10,10]

current_square_points=[init_loc_1,init_loc_2,init_loc_3,init_loc_4]
current_center_Corner=[[0,0]]

marker_location_hold=True

def main():
   

    desired_aruco_dictionary1 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    this_aruco_parameters1 =  cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(desired_aruco_dictionary1, this_aruco_parameters1)
 
    desired_aruco_dictionary2 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    this_aruco_parameters2 =  cv2.aruco.DetectorParameters()
    detector2 = cv2.aruco.ArucoDetector(desired_aruco_dictionary2, this_aruco_parameters2)
    
   
    cap = cv2.VideoCapture(0)
    
    square_points=current_square_points


    while(True):

        current_time=time.time()
        delay=0 

        ret, frame = cap.read()  
        
        markers, ids, rejectedCandidates = detector.detectMarkers(frame)
        marker_foam, ids_foam, rejectedCandidates_2 = detector2.detectMarkers(frame)

        frame_clean=frame.copy()

        left_corners,corner_ids=getMarkerCoordinates(markers,ids,0)
 
        if marker_location_hold==True:
            if corner_ids is not None:
                count=0
                for id in corner_ids:
                    
                    if id>4:
                        break  
                    id = int(id)

                    current_square_points[id-1]=left_corners[count]
                    count=count+1
            left_corners=current_square_points            
            corner_ids=[1,2,3,4]      

        
        if (start_time+delay*1)<current_time and (start_time+delay*2)>current_time:   
            cv2.aruco.drawDetectedMarkers(frame, markers) 
        if (start_time+delay*2)<current_time:    
            draw_corners(frame,left_corners)
        if (start_time+delay*3)<current_time:
            draw_numbers(frame,left_corners,corner_ids)
        if (start_time+delay*4)<current_time:    
            show_spec(frame,left_corners)
       
        frame_with_square,squareFound=draw_field(frame,left_corners,corner_ids)
        
            
        if (start_time+delay*6)<current_time:
            if squareFound:
                square_points=left_corners
            img_wrapped=four_point_transform(frame_clean, np.array(square_points))
            h, w, c = img_wrapped.shape
            
            left_corner_foam,corner_id_foam=getMarkerCoordinates(marker_foam,ids_foam,0)
            centerCorner=getMarkerCenter_foam(marker_foam)
     
            if marker_location_hold==True:
                if corner_id_foam is not None:
                    
                    current_center_Corner[0]=centerCorner[0]
                centerCorner[0]=current_center_Corner[0]              
            
            draw_corners(img_wrapped,centerCorner)
            img_wrapped=cv2.line(img_wrapped,(centerCorner[0][0],0), (centerCorner[0][0],h), (0,0,255), 2)
            img_wrapped=cv2.line(img_wrapped,(0,(centerCorner[0][1])), (w,(centerCorner[0][1])), (0,0,255), 2)

            draw_numbers(img_wrapped,left_corner_foam,corner_id_foam)
            cv2.imshow('img_wrapped',img_wrapped)

        cv2.imshow('frame_with_square',frame_with_square)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            robot_arm.arduino.write(b'H0,90,20,90,90,73,20\n') 
            break        
         
        if key == ord('p'):
            x_coordinate=int((centerCorner[0][1]/h)*600)-100
            y_coordinate=int((centerCorner[0][0]/w)*300)
            print("Optical position: ",x_coordinate,", ",y_coordinate)
            x_coordinate_comp,y_coordinate_comp=robot_arm.camera_compensation(x_coordinate,y_coordinate)
            print("Position after compensation: ",x_coordinate_comp,", ",y_coordinate_comp)
            robot_arm.pick_up(x_coordinate,y_coordinate)
            print("Foam placed!")
            
    cap.release()
    cv2.destroyAllWindows()
    return centerCorner 
   

if __name__ == '__main__':
    robot_arm.home()
    foam_center=main() 


