import numpy as np
from scipy import interpolate

def get_smoothed_path(current_pose, segmented_path, max_vel):

    offset_start_smoothing = 15
    offset_end_smoothing = 20

    found_start_local_path = False
    found_end_local_path = False
    
    min_dist = float('inf')
    closest_idx = -1
    for idx in range(len(segmented_path)):
        dist_path_point =  np.sqrt( (current_pose[0] - segmented_path[idx][0])**2 + 
                                    (current_pose[1] - segmented_path[idx][1])**2 )
        if dist_path_point < min_dist:
            min_dist = dist_path_point
            closest_idx = idx
   
    idx = 0
    while not found_start_local_path:
        dist_path_point =  np.sqrt( (segmented_path[closest_idx][0] - segmented_path[idx][0])**2 + 
                                    (segmented_path[closest_idx][1] - segmented_path[idx][1])**2 )
        if dist_path_point < offset_start_smoothing:
            found_start_local_path=True
            idx_start_local_path = idx

        idx+=1

    while not found_end_local_path:
        if len(segmented_path)<=idx-1:
            found_end_local_path=True
            idx_end_local_path = idx
        else:
            dist_path_point =  np.sqrt( (segmented_path[closest_idx][0] - segmented_path[idx][0])**2 + 
                                        (segmented_path[closest_idx][1] - segmented_path[idx][1])**2 )
            if dist_path_point >= offset_end_smoothing:
                found_end_local_path=True
                idx_end_local_path = idx
            
        idx+=1

    segmented_local_path = segmented_path[idx_start_local_path:idx_end_local_path]

    path_length = 0.0
    for i in range(len(segmented_local_path)-1):
        path_length += np.sqrt((segmented_local_path[i+1][0] - segmented_local_path[i][0])**2 + 
                               (segmented_local_path[i+1][1] - segmented_local_path[i][1])**2)
    num_points = int(2*path_length/(max_vel-2)*10)

    order_curve = min(max(len(segmented_local_path)-1, 1), 4)
    x_coords = [points[0] for points in segmented_local_path]
    y_coords = [points[1] for points in segmented_local_path]

    tck, *rest = interpolate.splprep([x_coords, y_coords], k=order_curve)   
    t = np.linspace(0, 1, num_points)
    interpolated_path = interpolate.splev(t, tck)
    new_x_coords, new_y_coords = interpolated_path

    smoothed_path = list(zip(new_x_coords, new_y_coords))

    for i in range(len(smoothed_path)):
        smoothed_path.append(smoothed_path[-1])
    smoothed_path = np.asarray(smoothed_path)
    idx_smoothed = find_next_point_idx(smoothed_path, current_pose)
    smoothed_path = smoothed_path[idx_smoothed:]
    

    return smoothed_path

def find_next_point_idx(smoothed_path, start):

        # find next point of the smoothed path to follow 
        point_found = False
        i = 0
        while not point_found:
                     
            m_coeff = (smoothed_path[i,1] - smoothed_path[i+1,1])/(smoothed_path[i,0] - smoothed_path[i+1,0])
            perp_m_coeff = -1/m_coeff
            perp_b_coeff = -perp_m_coeff*(smoothed_path[i,0]) + smoothed_path[i,1]

            if (smoothed_path[i+1,1] - perp_m_coeff*smoothed_path[i+1,0] - perp_b_coeff)>0:
                if (start[1] - perp_m_coeff*start[0] - perp_b_coeff)<-1:
                    point_found = True
            else:
                if (start[1] - perp_m_coeff*start[0] - perp_b_coeff)>1:
                    point_found = True

            i += 1

        return i