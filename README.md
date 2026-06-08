
# MapMatching

Need to download the xxx.osm.pbf map first from https://download.geofabrik.de/。中国江苏的地图从这里下载https://download.geofabrik.de/asia/china/jiangsu.html/。

This map file is very big about 500MB, which is why not pushed to github.com

# Configure the C/C++ running environment and install the lib 
# WSL environment in window, or Linux environment

C++ 源代码在 src目录下面：“src/osm_pbf_to_scene_gltf.cpp”。C++代码需要先编译再运行。步骤如下：

First install below lib:
sudo apt install nlohmann-json3-dev
sudo apt install     libosmium2-dev     libboost-dev     zlib1g-dev
sudo apt install     zlib1g-dev     libbz2-dev     libexpat1-dev

Then compile the C++ code like below: 
g++ src/osm_pbf_to_scene_gltf.cpp     -Ithird_party/earcut.hpp/include     -Ithird_party/json/single_include     -std=c++17     -o osm_pbf_to_scene_gltf     -lz -lbz2 -lexpat

After that:
./osm_pbf_to_scene_gltf ../Maps/jiangsu260603.osm.pbf buildings.glb --format glb --
bbox 32.0000 118.7000 32.0900 118.8600 --material-mode infer


输出是模型文件building.glb 和模型中的建筑物属性文件buildings_extras.json

模型可以用这个网站导入模型：https://threejs.org/editor/，用高德地图验证是否正确：https://www.amap.com/search?query=%E5%8D%97%E4%BA%AC&city=110000&geoobj=115.41888%7C39.294693%7C118.249433%7C40.571008&zoom=9.04 （定位南京）

1. 选择区域，根据经纬度生成glb，gltf（和bin）的地图文件
--bbox 32.0000 118.7000 32.0900 118.8600，南京市新街口附件区域。
经纬度的格式是： 
bbox = (min_lon, min_lat, max_lon, max_lat)，最小经度，最小纬度，最大经度，最大纬度

另外还有一下的经纬度方便测试 （用的时候去掉逗号）：
    "New York Manhattan": (40.70, -74.02, 40.72, -73.99), (建筑很多，程序比较慢)
    "London Camden": (51.52, -0.19, 51.56, -0.11),
    "London Canary Wharf": (51.49, -0.02, 51.51, 0.02),
    "Shanghai Pudong": (31.20, 121.45, 31.27, 121.55),
    "Nanjing Jiangning": (31.63, 118.42, 32.10, 119.05),
    "Nanjing Xinjiekou": (32.0000, 118.7000, 32.0900, 118.8600),
    "Nanjing Baijiahu": (31.9100, 118.7750, 31.9700, 118.8500)

国内地图的经纬度一般可以从AI中直接问出来，也可以从高德的地图找：https://lbs.amap.com/demo/javascript-api/example/3d/map3d。在右侧的HTML脚本中有这个function mapInit()， 其中center:[116.333926,39.997245] 就是地图任意位置的经纬度值。

2. 选择输出 .glb 还是 .gltf
--format glb, 或者 --format gltf

3. 给建筑物赋材料
--material-mode infer
OSM 里没有 material 时，JSON 里标明 unknown，也可以选择按建筑类型推测一个材质
OSM 的确经常没有建筑材料信息。很多建筑只有：

building=yes
building=residential
building:levels=6
height=20
name=...

但没有：

building:material
facade:material
material
wall:material
roof:material

这里的处理方式是：
--material-mode actual：只使用 OSM 真实材料，没有就 unknown
--material-mode infer：没有真实材料时，根据 building=* 推测，比如 residential -> concrete

在gltf中还有buildings_extras.json中直接把
        "inferred_material": "brick",
        "material_inference_source": "default_brick",
        "dielectric_constant": 4.44,
        "conductivity_s_per_m": 0.018,
写入到gltf中的extras扩展字段中，也写在生成单独的JSON文件。 

-------------------------------------------------------
另外"osm_type": "way" 表示这个对象在 OSM 里的原始类型是 way。
way：一串 OSM 点连成的线或面
relation multipolygon：多个 way 拼起来形成的复杂面
他们的关系是：
way = 基础零件
multipolygon relation = 用多个 way 拼出来的大区域

"osm_type": "way"
表示普通 OSM way。

"osm_type": "relation"
表示来自 relation multipolygon，比如复杂湖泊、水库、公园等。

"osm_type": "generated"
表示程序生成的默认 bbox 地面 bbox_ground。
------------------------------------------------------

# 一些材料的颜色列表如下：
water  水体   -> 不透明亮蓝色
land   土壤   -> 棕土色
grass  草地   -> 亮绿色
forest 森林   -> 深绿色
road   道路   -> 中灰色
sand   沙地   -> 沙黄色
