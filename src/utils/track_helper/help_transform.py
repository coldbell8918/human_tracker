import math
import numpy as np
### tf function -> lookup tranform funtion is exist , if u use it. target frame ,geometry_msgs/PoseStemped

def tf_transform(robot_x,robot_y,robot_th,x,y):
    d=math.sqrt(pow(x,2)+pow(y,2))
    a=math.atan(y/x)
    n_x=math.cos(a + robot_th)
    n_y=math.sin(a + robot_th)
    return round(robot_x+d*n_x,3), round(robot_y+d*n_y,3)

def data_transform(data,cam_lidar):
    merge=list()
    if cam_lidar=='cam':
        axis,number,clss=data.axis,data.number,data.clss

        for i in range(len(clss)):
            merge.append(clss[i])
            merge.append(number[i])
            y=round(pow(pow(axis[2*i+1]+0.1,2)-pow(axis[2*i],2),0.5),3)
            merge.append((round(-axis[2*i],3),y,round(axis[2*i+1]+0.1,3)))
   
    elif cam_lidar=='lidar':
        crowd=data.people
        
        for i in range(len(crowd)):
            merge.append(0)
            merge.append(crowd[i].id)
            x=round(crowd[i].pose.position.x,3)
            z=round(crowd[i].pose.position.y,3)
            y=round(pow(pow(z,2)+pow(x,2),0.5),3)
            merge.append((x,y,z)) #### axis???? x,y,z???
    merge=np.array(merge,dtype=object)

    return np.reshape(merge,(int(len(merge)/3),3))  ### merge [[0,1,(x,y,z)]]
