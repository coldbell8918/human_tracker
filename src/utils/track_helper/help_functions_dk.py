import numpy as np

def matching_target(list_,target,length):
    """
    Description: 
    """
    id_list=[list_[i][1] for i in range(len(list_))]
    check=np.where(np.array(id_list) == target)[0]
    
    if length=='len':
        return len(check)
    elif length=='index':
        return check

def matching_data(lidar,camera,const):
    return np.where(np.linalg.norm(np.array(lidar)-np.array(camera), 2)<=const)[0]
    
def min_dist_cal(x):
    return np.where(np.array(x)-min(x)==0)[0][0] 

def track_function(persons):
    list_ = persons.persons
    axis_z_list=[list_[i].pose.x for i in range(len(list_))]
    min_dist=min_dist_cal(axis_z_list)
    # target= list_[min_dist].index
    return min_dist

def goal_pub(goal_msg,save_last_goal):
    goal_msg.pose.position.x=save_last_goal[0]
    goal_msg.pose.position.y=save_last_goal[1]
    return goal_msg