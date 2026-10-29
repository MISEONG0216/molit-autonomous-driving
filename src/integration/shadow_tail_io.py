"""Short-lookahead recovery for a latched LiDAR path near its endpoint.

This does not create a new detour or override LiDAR STOP_REQUIRED; it only
finds a still-forward point on an already verified existing local path.
Never authorizes tracking a target behind the vehicle.
"""
import math
import numpy as np


def shadow_tail_steering(path, ego_x, ego_y, yaw, vehicle_length,
                         minimum_forward_m=0.45, max_steer_rad=0.25):
    xy=np.asarray(path,dtype=float)
    if xy.ndim!=2 or len(xy)<2 or xy.shape[1]<2:
        return None
    xy=xy[:,:2]
    if not np.isfinite(xy).all() or not all(map(math.isfinite,(
            ego_x,ego_y,yaw,vehicle_length))):
        return None
    delta=xy-np.array([ego_x,ego_y])
    dist2=np.sum(delta*delta,axis=1)
    nearest=int(np.argmin(dist2))
    c,s=math.cos(yaw),math.sin(yaw)
    dx=delta[:,0]*c+delta[:,1]*s
    dy=-delta[:,0]*s+delta[:,1]*c
    candidates=[]
    for i in range(nearest,len(xy)):
        if dx[i] >= minimum_forward_m and math.isfinite(dx[i]):
            candidates.append(i)
    if not candidates:
        return None
    # Prefer a point as near as possible to the usual 2.5m lookahead.
    # If nothing is that far ahead, use the last forward endpoint.
    target=next((i for i in candidates if math.hypot(dx[i],dy[i])>=2.5),
                candidates[-1])
    d2=dx[target]**2+dy[target]**2
    if d2<minimum_forward_m**2:
        return None
    # Curvature for an arbitrary near-endpoint lookahead (not fixed 2.5m).
    steer=math.atan2(2*vehicle_length*dy[target],d2)
    if not math.isfinite(steer) or abs(steer)>max_steer_rad:
        return None
    return {"steering":float(steer),"target_index":int(target),
            "nearest_index":int(nearest),
            "lookahead_m":math.sqrt(d2)}
