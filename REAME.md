首先安装carla 0.9.15.

然后运行服务器
./CarlaUE4.sh -quality-level=Low -resx=800 -resy=600 -world-port=2000

然后python3 generate_traffic.py --tm-port 3001 -n 50 -w 50 --safe 生成行人、车辆

然后运行目标检测。
cd object_detection/yolov3
python test0.py



常见问题：
1.  
libGL error: No matching fbConfigs or visuals found
libGL error: failed to load driver: swrast

解决方法：
export __GLX_VENDOR_LIBRARY_NAME=nvidia 解决

2.

