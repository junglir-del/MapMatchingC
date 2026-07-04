
# MapMatching

1. Need to download the xxx.osm.pbf map first from https://download.geofabrik.de/。中国江苏的地图从这里下载https://download.geofabrik.de/asia/china/jiangsu.html/。

This map file is very big about 500MB, which is why not pushed to github.com

2. 程序运行时也需要从opentopography.org下载DEM地图信息包含地形起伏的数据，这个会用API key。需要先申请账号，然后申请免费的API key。然后再Linux下执行以下命令把API key添加进去
export OPENTOPO_API_KEY = <paste your API key here directly>

# Configure the C/C++ running environment and install the lib （WSL environment in Windows, or Linux environment）

C++ 源代码在 src目录下面：“src/osm_pbf_to_scene_gltf.cpp” 和“osm_pbf_DEM4_precise_soil_fill.cpp”。C++代码需要先编译再运行。步骤如下：

First install below lib (seperate commands or combine them):
sudo apt install    nlohmann-json3-dev
sudo apt install    libosmium2-dev     libboost-dev     zlib1g-dev
sudo apt install    zlib1g-dev     libbz2-dev     libexpat1-dev
还有libgdal-dev 和 gdal-bin, 及其他库函数

Then compile the C++ code like below: 
1. 编译没有地形高低起伏的
g++ src/osm_pbf_to_scene_gltf.cpp     -Ithird_party/earcut.hpp/include     -Ithird_party/json/single_include     -std=c++17     -o osm_pbf_to_scene_gltf     -lz -lbz2 -lexpat

After that:
./osm_pbf_to_scene_gltf ../Maps/jiangsu260603.osm.pbf buildings.glb --format glb --
bbox 32.0000 118.7000 32.0900 118.8600 --material-mode infer


2. 编译包含地形起伏高低 （重庆，爱丁堡，等地）
g++ src/osm_pbf_DEM4_precise_soil_fill.cpp \
    -I/usr/include/gdal \
    -Ithird_party/earcut.hpp/include \
    -Ithird_party/json/single_include \
    -std=c++17 \
    -o osm_pbf_to_scene_gltf \
    -lgdal -lcurl -lpolyclipping \
    -lz -lbz2 -lexpat

After that: 
./osm_pbf_to_scene_gltf ../Maps/chongqing-260703.osm.pbf output_precise.glb     --format glb     --bbox 29.5410 106.5238 29.5865 106.5948  --material-mode infer     --download-dem     --dem-type COP30     --soil-clip-tiles 12

如果Precise soil clipping with Clipper过程慢，可以试：
--soil-clip-tiles 12 或者--soil-clip-tiles 16, 
tile 越多，单块 Clipper 压力越小，进度也更细；但会生成更多分块边界上的 soil polygon。建筑、道路、水体、植被这些真实边界的裁切精度没有降低

3. 输出是模型文件output_precise.glb 和模型中的建筑物属性文件output_precise_extras.json


最后生成的模型可以用这个网站导入模型：https://threejs.org/editor/，用高德地图验证是否正确：https://www.amap.com/search?query=%E5%8D%97%E4%BA%AC&city=110000&geoobj=115.41888%7C39.294693%7C118.249433%7C40.571008&zoom=9.04 （定位南京）


# Model generation/output

1. 选择区域，根据经纬度生成glb，gltf（和bin）的地图文件
--bbox 32.0000 118.7000 32.0900 118.8600，南京市新街口附件区域。
经纬度的格式是： 
bbox = (min_lon, min_lat, max_lon, max_lat)，最小经度，最小纬度，最大经度，最大纬度

另外还有一下的经纬度方便测试 （用的时候去掉逗号）：
    "New York Manhattan": (40.70 -74.02 40.72 -73.99), (建筑很多，程序比较慢)
    "London Camden": (51.52 -0.19 51.56 -0.11),
    "London Canary Wharf": (51.49 -0.02 51.51 0.02),
    "Shanghai Pudong": (31.20 121.45 31.27 121.55),
    "Nanjing Jiangning": (31.63 118.42 32.10 119.05),
    "Nanjing Xinjiekou": (32.0000 118.7000 32.0900 118.8600),
    "Nanjing Baijiahu": (31.9100 118.7750 31.9700 118.8500)
    "nanjing zijinshan" : (118.825 32.032 118.8655 32.0605)
    "chongqing": (29.5410 106.5238 29.5865 106.5948)
    "Edinburgh": (55.9429 -3.1752 55.9609 -3.2074)

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


# 以下是ChatGPT写的windows下怎么安装运行环境：
# Configure the C/C++ running environment and install the lib in Windows
如果你的目标是 **在 Windows 11 原生环境下编译和运行这个 OSM → glTF 项目**，那我建议直接走：

```text
Windows 11
├─ VS Code
├─ Visual Studio Build Tools 2022 (MSVC)
└─ vcpkg
```

不要用 MinGW。

原因是你的项目依赖：

```cpp
#include <osmium/handler.hpp>
#include <nlohmann/json.hpp>
#include <mapbox/earcut.hpp>
```

而 libosmium 官方长期测试的主要平台就是 GCC 和 MSVC。

---

# 第一步：安装 MSVC 编译器

安装：

[Visual Studio Build Tools 2022](https://visualstudio.microsoft.com/downloads/?utm_source=chatgpt.com)

安装时勾选：

```text
Desktop development with C++
```

安装完成后打开：

```text
x64 Native Tools Command Prompt for VS 2022
```

测试：

```cmd
cl
```

应该显示类似：

```text
Microsoft (R) C/C++ Optimizing Compiler Version ...
```

---

# 第二步：安装 vcpkg

打开 PowerShell：

```powershell
git clone https://github.com/microsoft/vcpkg.git C:\vcpkg
cd C:\vcpkg
.\bootstrap-vcpkg.bat
```

然后：

```powershell
.\vcpkg integrate install
```

---

# 第三步：安装依赖

你的 Linux 依赖对应到 Windows：

| Ubuntu             | Windows(vcpkg) |
| ------------------ | -------------- |
| nlohmann-json3-dev | nlohmann-json  |
| libosmium2-dev     | libosmium      |
| libboost-dev       | boost          |
| zlib1g-dev         | zlib           |
| libbz2-dev         | bzip2          |
| libexpat1-dev      | expat          |

安装：

```powershell
C:\vcpkg\vcpkg install ^
    nlohmann-json ^
    libosmium ^
    boost ^
    zlib ^
    bzip2 ^
    expat
```

如果你是 64 位：

```powershell
C:\vcpkg\vcpkg install ^
    nlohmann-json:x64-windows ^
    libosmium:x64-windows ^
    boost:x64-windows ^
    zlib:x64-windows ^
    bzip2:x64-windows ^
    expat:x64-windows
```

---

# 第四步：处理 earcut.hpp

Earcut 是 Header Only。

下载：

[Mapbox Earcut.hpp Repository](https://github.com/mapbox/earcut.hpp?utm_source=chatgpt.com)

放到：

```text
MapMatchingC
├─ src
├─ third_party
│   └─ earcut.hpp
│       └─ include
│           └─ mapbox
│               └─ earcut.hpp
```

---

# 第五步：VS Code 配置

`.vscode/c_cpp_properties.json`

```json
{
  "configurations": [
    {
      "name": "Win64",
      "compilerPath": "C:/Program Files/Microsoft Visual Studio/2022/BuildTools/VC/Tools/MSVC/.../bin/Hostx64/x64/cl.exe",
      "includePath": [
        "${workspaceFolder}/**",
        "${workspaceFolder}/third_party/earcut.hpp/include",
        "C:/vcpkg/installed/x64-windows/include"
      ],
      "cppStandard": "c++17"
    }
  ],
  "version": 4
}
```

---

# 第六步：编译

MSVC：

```cmd
cl /std:c++17 ^
   /EHsc ^
   src\osm_pbf_to_scene_gltf.cpp ^
   /I third_party\earcut.hpp\include ^
   /I C:\vcpkg\installed\x64-windows\include ^
   /link ^
   /LIBPATH:C:\vcpkg\installed\x64-windows\lib
```

---

# 更现代的方案（推荐）

如果项目没有 CMake，我建议加一个。

`CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.20)

project(MapMatchingC)

find_package(nlohmann_json CONFIG REQUIRED)
find_package(expat CONFIG REQUIRED)
find_package(ZLIB REQUIRED)

add_executable(osm_pbf_to_scene_gltf
    src/osm_pbf_to_scene_gltf.cpp
)

target_include_directories(
    osm_pbf_to_scene_gltf PRIVATE
    third_party/earcut.hpp/include
)

target_link_libraries(
    osm_pbf_to_scene_gltf PRIVATE
    nlohmann_json::nlohmann_json
    Expat::Expat
    ZLIB::ZLIB
)
```

然后：

```powershell
cmake -B build ^
  -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake

cmake --build build --config Release
```

这样以后增加库时几乎不用再手动写 `/I`、`/LIBPATH`。

---

我还想先确认一件事：你的 `MapMatchingC` 项目是**你自己写的**，还是从 GitHub 克隆的？

如果是 GitHub 项目，把仓库地址发给我，我可以直接告诉你在 Windows 下应该怎么编译，甚至帮你生成完整的 `CMakeLists.txt`。
