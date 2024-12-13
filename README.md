# Carla 环境搭建与目标检测

### 环境搭建
1. **下载 Carla**
  
 下载Carla 0.9.15，建议从国内镜像下载二进制，如下：
```bash
wget https://mirrors.sustech.edu.cn/carla/carla/0.9.15/CARLA_0.9.15.tar.gz
wget https://mirrors.sustech.edu.cn/carla/carla/0.9.15/AdditionalMaps_0.9.15.tar.gz
 ```

2. **安装 Carla 服务端**

  将 CARLA_0.9.15.tar.gz 解压，比如解压到dbz/carla_0.9.15下面， 然后将AdditionalMaps_0.9.15.tar.gz放到
上述目录的Import子目录下面，然后用运行./ImportAssets.sh。

```bash
cd dbz
mkdir carla_0.9.15
mv CARLA_0.9.15.tar.gz carla_0.9.15
cd carla_0.9.15
tar -xvf CARLA_0.9.15.tar.gz
mv ../AdditionalMaps_0.9.15.tar.gz Import
bash ./ImportAssets.sh
```

3. **安装 Carla 客户端**

 首先创建python虚拟环境
 ```bash
pip3 install virtualenv
virtualenv carla_venv

 ```
然后进入到虚拟环境进行安装
```bash
cd carla_venv
source bin/activate
pip3 install carla==0.9.15
cd /PythonAPI/examples
python3 -m pip install -r requirements.txt
 ```
4. **下载本 github 代码**
```bash
git clone https://github.com/sploving1/carla/
 ```
5. **下载 YOLOv3 权重**
   ```bash
   wget https://pjreddie.com/media/files/yolov3.weights
    ```


### 运行

1. **启动服务端**
```bash
./CarlaUE4.sh -quality-level=Low -resx=800 -resy=600 -world-port=2000
```

2. **生成行人和车辆**

 在虚拟环境里面运行：
```bash
cd PythonAPI/examples
python3 generate_traffic.py --tm-port 3001 -n 50 -w 50 --safe
```
3. **目标检测**
 
 在虚拟环境里面运行：
```bash
cd object_detection/yolov3
python test0.py
```


### 常见问题

1. **图形库相关问题**

 ##### 报错：
```bash
libGL error: No matching fbConfigs or visuals found
libGL error: failed to load driver: swrast
```

 ##### 原因：
```bash
nvidia 显卡提供的 glvnd 前端库 (libGLX.so) 在运行时加载了 mesa 实现的库 (libGLX_mesa.so)，
而不是加载 nvidia 自己实现的库 (libGLX_nvidia.so)导致的。
```
 ##### 解决方法：
```bash
export __GLX_VENDOR_LIBRARY_NAME=nvidia
```

2. **交通管理器端口冲突**

 ##### 报错：
```bash
RuntimeError: trying to create rpc server for traffic manager; but the system failed to create 
because of bind error.
```
 ##### 原因：
```bash
交通管理器默端口8000 被占用导致的。
```
 
 ##### 解决方法：
```python
tm = self.client.get_trafficmanager(3001)
tm_port = tm.get_port()
self.car.set_autopilot(True, tm_port)
```
