"""Timestamped route projection and user-visible reasons, independent of ROS/Qt."""
import time
import numpy as np
from .nav_math import apply_transform, make_ribbon, project_ground, snap_route_to_ground, route_range_mask


def grounded_route(node,stamp):
    node.ar_ground_source = ''
    mesh_route = getattr(node, '_meshnav_active', False)
    route = node.path + [0., 0., node.ground_render_offset_m] if mesh_route else node.path
    if node.ground_frame and node.ground_frame!=node.base_frame:
        matrix=node.lookup_matrix(node.ground_frame,node.path_frame,stamp)
        if matrix is not None:
            points=apply_transform(route,matrix)
            if not mesh_route:
                points=snap_route_to_ground(points,0.,node.ground_render_offset_m)
            body=node.lookup_matrix(node.ground_frame,node.base_frame,stamp)
            if body is None:return None
            delta=points-body[:3,3]
            yaw=np.arctan2(body[1,0],body[0,0]);c,s=np.cos(yaw),np.sin(yaw)
            local=np.column_stack((c*delta[:,0]+s*delta[:,1],-s*delta[:,0]+c*delta[:,1],delta[:,2]))
            node.ar_ground_source = node.ground_frame
            return points,node.ground_frame,route_range_mask(local,node.style.route_near_clip_m,node.style.route_far_clip_m)
    # A handheld body tilts; its XY plane cannot stand in for the floor. Use
    # the gravity-aligned map and the measured body height for the flat-floor
    # fallback, then let the image-time camera transform account for tilt.
    frame=getattr(node,'map_frame','map')
    body=node.lookup_matrix(frame,node.base_frame,stamp)
    path=node.lookup_matrix(frame,node.path_frame,stamp)
    if body is None or path is None:return None
    points=apply_transform(route,path)
    if not mesh_route:
        points=snap_route_to_ground(points,body[2,3]-node.ground_height_m,node.ground_render_offset_m)
    delta=points-body[:3,3]
    yaw=np.arctan2(body[1,0],body[0,0]);c,s=np.cos(yaw),np.sin(yaw)
    local=np.column_stack((c*delta[:,0]+s*delta[:,1],-s*delta[:,0]+c*delta[:,1],delta[:,2]))
    node.ar_ground_source = 'mesh' if mesh_route else 'configured_height'
    return points,frame,route_range_mask(local,node.style.route_near_clip_m,node.style.route_far_clip_m)


def project_camera(points, intrinsic, distortion=(), model='plumb_bob'):
    points=np.asarray(points,float);K=np.asarray(intrinsic,float)
    d=np.asarray(distortion,float)
    if (K.shape!=(3,3) or not np.isfinite(K).all() or K[0,0]<=0 or K[1,1]<=0 or
            d.ndim!=1 or not np.isfinite(d).all() or
            model not in ('plumb_bob','rational_polynomial') or len(d) not in (0,4,5,8)):
        raise ValueError('相机标定格式不受支持，请检查相机配置')
    coefficients=np.zeros(8);coefficients[:len(d)]=d
    k1,k2,p1,p2,k3,k4,k5,k6=coefficients
    valid=np.isfinite(points).all(axis=1)&(points[:,2]>.05)
    z=np.where(valid,points[:,2],1.)
    x,y=points[:,0]/z,points[:,1]/z
    with np.errstate(over='ignore',invalid='ignore',divide='ignore'):
        r2=x*x+y*y
        denominator=1+k4*r2+k5*r2*r2+k6*r2*r2*r2
        radial=(1+k1*r2+k2*r2*r2+k3*r2*r2*r2)/denominator
        xd=x*radial+2*p1*x*y+p2*(r2+2*x*x)
        yd=y*radial+p1*(r2+2*y*y)+2*p2*x*y
        pixels=np.column_stack((K[0,0]*xd+K[0,1]*yd+K[0,2],K[1,1]*yd+K[1,2]))
    valid &= np.isfinite(pixels).all(axis=1)&(np.abs(denominator)>1e-9)
    pixels[~valid]=0
    return pixels,valid


def route_intersects_image(pixels, valid, image_shape):
    """Clip segments, including ones crossing the image with both ends outside."""
    a,b=np.asarray(pixels[:-1],float),np.asarray(pixels[1:],float)
    possible=np.asarray(valid[:-1]&valid[1:],bool).copy()
    lo=np.zeros(len(a));hi=np.ones(len(a));delta=b-a
    for axis,limit in ((0,image_shape[1]-1),(1,image_shape[0]-1)):
        moving=np.abs(delta[:,axis])>1e-12
        possible &= moving|((a[:,axis]>=0)&(a[:,axis]<=limit))
        safe=np.where(moving,delta[:,axis],1.)
        first=-a[:,axis]/safe;last=(limit-a[:,axis])/safe
        lo=np.where(moving,np.maximum(lo,np.minimum(first,last)),lo)
        hi=np.where(moving,np.minimum(hi,np.maximum(first,last)),hi)
    return bool(np.any(possible&(lo<=hi)))


def projected_route(node,image_shape):
    def unavailable(code,message):
        node.ar_status=dict(code=code,message=message);return None
    now=node.get_clock().now().nanoseconds
    if node.require_localizer_quality and not node.localization_tools.health.valid(now):
        return unavailable('localization','等待地图定位恢复，暂不显示相机路线')
    if not node.navigation_active:
        return unavailable('navigation','导航运行后显示相机路线')
    if node.camera_error:
        return unavailable('camera_format',node.camera_error)
    if node.camera_calibration_error:
        return unavailable('calibration',node.camera_calibration_error)
    if node.path_error:
        return unavailable('route_invalid',node.path_error)
    stamp=getattr(node,'image_stamp',None)
    if stamp is None or (stamp.sec==0 and stamp.nanosec==0):
        return unavailable('camera_time','相机时间无效，请检查相机连接')
    age=(now-stamp.sec*10**9-stamp.nanosec)/1e9
    if not -.05<=age<=.5 or time.monotonic()-node._image_received_at>.5:
        return unavailable('camera_stale','相机画面延迟，请停下等待画面恢复')
    if len(node.path)<2:return unavailable('route','尚无可显示的路线')
    grounded=node._grounded_route(stamp)
    if grounded is None:return unavailable('pose_time','正在等待与相机画面同步的位置和方向')
    center,frame,visible=grounded
    left,right=make_ribbon(center,node.style.ribbon_width_m)
    points=np.vstack((center,left,right));count=len(center)
    pixels=valid=None;approximate=False
    if node.projection_mode in ('auto','calibrated') and node.camera_matrix is not None and node.camera_frame:
        transform=node.lookup_matrix(node.camera_frame,frame,stamp)
        if transform is None:return unavailable('camera_pose','相机与设备的姿态连接尚未就绪')
        K=node.camera_matrix.copy()
        if node.camera_info_size:
            sx=image_shape[1]/node.camera_info_size[0];sy=image_shape[0]/node.camera_info_size[1]
            K[0,:]*=sx;K[1,:]*=sy
            K[0,2]+=(sx-1)/2;K[1,2]+=(sy-1)/2
        try:
            pixels,valid=project_camera(apply_transform(points,transform),K,
                getattr(node,'camera_distortion',()),getattr(node,'camera_distortion_model','plumb_bob'))
        except ValueError as exc:return unavailable('calibration',str(exc))
    elif node.projection_mode=='calibrated':
        return unavailable('calibration','等待相机标定参数')
    else:
        transform=node.lookup_matrix(node.base_frame,frame,stamp)
        if transform is None:return unavailable('pose_time','正在等待与相机画面同步的位置和方向')
        pixels,valid=project_ground(apply_transform(points,transform),image_shape,node.style)
        approximate=True
    center_px,left_px,right_px=np.split(pixels,(count,count*2))
    a,b,c=np.split(valid,(count,count*2));valid=visible&a&b&c
    if len(valid)<2 or not any(route_intersects_image(edge,valid,image_shape)
                              for edge in (center_px,left_px,right_px)):
        return unavailable('outside_view','路线在相机视野外，请转向路线方向或查看地图')
    estimated = getattr(node, 'ar_ground_source', '') == 'configured_height'
    node.ar_status=dict(code='approximate' if approximate or estimated else 'ready',
                        message='示意路线，请结合地图确认方向' if approximate else
                        '地面高度按配置投影，请核对现场' if estimated else '',
                        ground_source=getattr(node, 'ar_ground_source', ''))
    return center_px,left_px,right_px,valid
