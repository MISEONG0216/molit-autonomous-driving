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
    left=pts[(pts[:,1]>=c)&(pts[:,1]<=7.)&(pts[:,0]>=2.)]
    right=pts[(pts[:,1]<=-c)&(pts[:,1]>=-7.)&(pts[:,0]>=2.)]
    n=int(cfg.get("min_side_points",8))
    if len(left)<n or len(right)<n:
        return dict(fail,reason="SIDE_WALL_NOT_OBSERVED")
    # Check a near-forward obstacle before a detour; do not plan unsafe
    # transverse motion when a blockage is already too near.
    immediate=pts[(pts[:,0]<=float(cfg.get("emergency_forward_m",2.5)))&
                  (np.abs(pts[:,1])<=c)]
    if len(immediate)>=int(cfg.get("obstacle_point_threshold",5)):
        return dict(fail,reason="IMMEDIATE_OBSTACLE")
    candidates=[]
    for side in (1,-1):
        p=_path(side,cfg)
        hit=np.count_nonzero(_distance_points_to_polyline(pts[:,:2],p)<c)
        if hit<=int(cfg.get("allowed_points",0)):
            candidates.append((side,p))
    if not candidates:
        return dict(fail,reason="NO_FREE_SIDE")
    if active_side in (1,-1):
        for side,p in candidates:
            if side==active_side:
                return {"status":"AVOIDING","side":side,"path":p.tolist(),"reason":"LOCKED_SIDE_CLEAR"}
    # Prefer LEFT if both are geometrically clear; avoid arbitrary flip.
    side,p=candidates[0]
    return {"status":"AVOIDING","side":side,"path":p.tolist(),"reason":"EGO_LIDAR_CLEAR"}
