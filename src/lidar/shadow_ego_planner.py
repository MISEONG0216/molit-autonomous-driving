"""Vehicle-frame LiDAR wall-aware local path, no GPS/global WP dependence.

Input point cloud axes: +x forward, +y left, +z up (lidar_to_vehicle).
Samples left/right static detour curves and rejects candidates touching ANY
observed LiDAR obstacle or tunnel wall, incl. wide clusters. A clear corridor
must have sufficient observed side-wall returns and safe swept footprint.
This planner never sends EgoCtrlCmd.
"""
import math
import numpy as np


def _distance_points_to_polyline(points, path):
    result = np.full(len(points), np.inf)
    for a,b in zip(path[:-1],path[1:]):
        v=b-a
        t=np.clip(np.sum((points-a)*v,axis=1)/max(np.dot(v,v),1e-9),0,1)
        result=np.minimum(result,np.linalg.norm(points-(a+t[:,None]*v),axis=1))
    return result


def _path(side, cfg):
    x=np.arange(0.,float(cfg.get("horizon_m",15.0))+.01,.5)
    enter=float(cfg.get("shift_end_m",7.0))
    return_start=float(cfg.get("return_start_m",10.0))
    return_end=float(cfg.get("horizon_m",15.0))
    d=float(cfg.get("offset_m",1.6))*side
    enter_t=np.clip((x-1.5)/max(.1,enter-1.5),0,1)
    exit_t=np.clip((x-return_start)/max(.1,return_end-return_start),0,1)
    sm=lambda t:t*t*(3-2*t)
    y=d*sm(enter_t)*(1-sm(exit_t))
    return np.column_stack((x,y))


def choose_local_detour(points, cfg, active_side=None):
    """Return dict status CLEAR/AVOIDING/STOP_REQUIRED and vehicle-local path.

    Does not assume unknown LiDAR space is clear. Doesn't plan around dynamic
    tracks: caller must stop if original tracker reports dynamic risk.
    """
    pts=np.asarray(points,dtype=float)
    fail={"status":"STOP_REQUIRED","side":None,"path":[],"reason":"UNVERIFIED"}
    if pts.ndim!=2 or pts.shape[1]<3 or len(pts)<int(cfg.get("min_points",80)):
        return dict(fail,reason="SPARSE_SCAN")
    pts=pts[np.isfinite(pts).all(axis=1)]
    zlo,zhi=float(cfg.get("z_min",-1.45)),float(cfg.get("z_max",1.25))
    pts=pts[(pts[:,2]>zlo)&(pts[:,2]<zhi)]
    horizon=float(cfg.get("horizon_m",15.0))
    pts=pts[(pts[:,0]>=1.)&(pts[:,0]<=horizon+1.)]
    if len(pts)<int(cfg.get("min_points",80)):
        return dict(fail,reason="SPARSE_FORWARD_SCAN")
    c=float(cfg.get("vehicle_half_width_m",.9))+float(cfg.get("margin_m",.45))
    # First distinguish a clear straight-ahead lane from a true obstruction.
    # Wall visibility on BOTH sides is necessary for a lateral DETOUR, not
    # for continuing the already camera-guided lane at low speed.
    immediate=pts[(pts[:,0]<=float(cfg.get("emergency_forward_m",2.5)))&
                  (np.abs(pts[:,1])<=c)]
    if len(immediate)>=int(cfg.get("obstacle_point_threshold",5)):
        return dict(fail,reason="IMMEDIATE_OBSTACLE")
    obstacle = pts[(pts[:,0]>=float(cfg.get("obstacle_min_forward_m",3.0)))&
                   (pts[:,0]<=float(cfg.get("obstacle_max_forward_m",12.0)))&
                   (np.abs(pts[:,1])<=c)]
    obstacle_present = len(obstacle)>=int(cfg.get("obstacle_point_threshold",5))
    if not obstacle_present and active_side not in (1,-1):
        return {"status":"CLEAR","side":None,"path":[],"reason":"EGO_LANE_CLEAR"}

    # Only after detecting a blockage should an uncertain wall observation
    # prohibit initiating a sideways maneuver.
    left=pts[(pts[:,1]>=c)&(pts[:,1]<=7.)&(pts[:,0]>=2.)]
    right=pts[(pts[:,1]<=-c)&(pts[:,1]>=-7.)&(pts[:,0]>=2.)]
    n=int(cfg.get("min_side_points",8))
    if len(left)<n or len(right)<n:
        return dict(fail,reason="SIDE_WALL_NOT_OBSERVED")
    # Candidate geometry must adapt to the observed obstacle footprint:
    # do not return to lane before the rear edge of the detected object.
    obstacle_front=float(np.min(obstacle[:,0]))
    obstacle_rear=float(np.max(obstacle[:,0]))
    half=float(cfg.get("vehicle_half_width_m",.9))
    margin=float(cfg.get("margin_m",.45))
    min_offset=float(cfg.get("minimum_offset_m",.8))
    max_offset=float(cfg.get("maximum_offset_m",2.6))
    step=float(cfg.get("offset_step_m",.2))
    candidates=[]
    for side in (1,-1):
        # Lateral clearance needed to fit the whole vehicle beside the
        # obstacle, not merely to move the camera center past its centroid.
        required=(float(np.max(obstacle[:,1]))+half+margin if side==1
                  else -float(np.min(obstacle[:,1]))+half+margin)
        offsets=np.arange(max(min_offset,math.ceil(max(0,required)/step)*step),
                          max_offset+1e-6,step)
        for offset in offsets:
            cfg2=dict(cfg)
            cfg2["offset_m"]=float(offset)
            # Finish the lateral shift BEFORE reaching the box.
            cfg2["shift_end_m"]=min(
                float(cfg.get("shift_end_m",7.0)),
                obstacle_front-float(cfg.get("obstacle_before_margin_m",1.5))
            )
            if cfg2["shift_end_m"]<=2.5:
                continue
            cfg2["return_start_m"]=max(
                float(cfg.get("return_start_m",10.0)),
                obstacle_rear+float(cfg.get("obstacle_after_margin_m",2.5))
            )
            cfg2["horizon_m"]=max(
                float(cfg.get("horizon_m",15.0)),
                cfg2["return_start_m"]+float(cfg.get("return_length_m",5.0))
            )
            if cfg2["horizon_m"]>float(cfg.get("max_horizon_m",24.0)):
                continue
            p=_path(side,cfg2)
            # Do not certify a late return through an unobserved region.
            # The current LiDAR scan must reach near the path endpoint.
            if float(np.max(pts[:,0])) < cfg2["horizon_m"]-float(
                    cfg.get("max_unobserved_tail_m",2.0)):
                continue
            hit=np.count_nonzero(_distance_points_to_polyline(pts[:,:2],p)<c)
            if hit<=int(cfg.get("allowed_points",0)):
                clearance=float(np.min(_distance_points_to_polyline(
                    pts[:,:2],p)))-c
                candidates.append((clearance,side,float(offset),p))
    if not candidates:
        return dict(fail,reason="NO_FREE_CANDIDATE")
    # Prefer the candidate with more minimum clearance. When nearly
    # identical prefer less deviation from camera lane. Never bias LEFT.
    best=max(candidates,key=lambda t:(round(t[0],2),-t[2]))
    clearance,side,offset,p=best
    return {"status":"AVOIDING","side":side,"path":p.tolist(),
            "offset_m":offset,"min_clearance_m":clearance,
            "reason":"EGO_VARIABLE_CLEARANCE"}
