"""Separate a late preview from missing source images; never relax AR freshness."""
import math


def camera_message(preview_age, source_age=None, running=None):
    def fresh(value):
        return isinstance(value, (int, float)) and math.isfinite(value) and 0 <= value <= .5
    if fresh(preview_age):
        return '实时画面'
    if fresh(source_age):
        return '画面处理延迟，原始图像仍在接收'
    if running is False:
        return '相机已停止'
    if source_age is not None and 0 <= source_age < 2:
        return '相机图像暂未更新'
    return '未收到相机图像，请检查连接'
