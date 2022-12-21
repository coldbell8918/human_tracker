import numpy as np
import cv2

def orb_match(target_img,image_list):
    """
        Description: orb launching
        Args:
            target_img : array_img data
            image_list : array_imgs data 
        
        return : counted match numbers, matrix distances 
    """
    orb = cv2.ORB_create()
    matcher = cv2.BFMatcher()

    target_img= cv2.cvtColor(target_img,cv2.COLOR_BGR2GRAY)
    image_list=[cv2.cvtColor(image_list[i], cv2.COLOR_BGR2GRAY) for i in range(len(image_list))]

    target_keypoints, target_descriptors = orb.detectAndCompute(target_img,None)
    
    match_count=[]
    distances=[]

    for image in image_list:
        image_keypoints, image_descriptors = orb.detectAndCompute(image,None)
        matches = matcher.match(target_descriptors,image_descriptors)
        match_count.append(len(matches))

        dis=0
        for i in image:
            dis+=image[i].distance
        distances.append(dis) 

    return match_count,distances
