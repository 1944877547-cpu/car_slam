#!/usr/bin/env python3
"""修正构建兼容和RGB-D纯定位启动状态，保留官方估计器和优化器。"""
from pathlib import Path
import sys

root = Path(sys.argv[1])
for name in ('CMakeLists.txt', 'Thirdparty/DBoW2/CMakeLists.txt', 'Thirdparty/g2o/CMakeLists.txt'):
    p = root / name
    text = p.read_text().replace('-march=native', '').replace('-std=c++11', '-std=c++14')
    if name == 'CMakeLists.txt':
        # 相机通过ROS输入，不编译数据集/RealSense示例。
        text = text.split('# Build examples')[0]
    p.write_text(text)

# 上游issue #515：加载Atlas后仍创建空地图，随后初始化而非重定位。
# 只在本适配器显式指定System.LocalizeOnly时选已有地图，建图路径保持原样。
p = root / 'src/System.cc'
text = p.read_text()
if '// mycar: use the saved map for localization' not in text:
    old = '        loadedAtlas = true;\n\n        mpAtlas->CreateNewMap();'
    new = '''        loadedAtlas = true;

        // mycar: use the saved map for localization
        if (static_cast<int>(fsSettings["System.LocalizeOnly"]) != 0)
        {
            Map* savedMap = nullptr;
            for (Map* map : mpAtlas->GetAllMaps())
                if (!map->IsBad() && (!savedMap || map->KeyFramesInMap() > savedMap->KeyFramesInMap()))
                    savedMap = map;
            if (!savedMap || savedMap->KeyFramesInMap() == 0)
                throw std::runtime_error("Atlas has no keyframes for localization");
            mpAtlas->ChangeMap(savedMap);
        }
        else
            mpAtlas->CreateNewMap();'''
    assert text.count(old) == 1, 'Unexpected upstream System.cc; check pinned revision'
    p.write_text(text.replace(old, new))

p = root / 'src/Tracking.cc'
text = p.read_text()
if '// mycar: loaded-map tracking starts with relocalization' not in text:
    old = '        mState = NOT_INITIALIZED;\n    }\n\n    mLastProcessedState=mState;'
    new = '''        // mycar: loaded-map tracking starts with relocalization
        mState = mbOnlyTracking ? LOST : NOT_INITIALIZED;
    }

    mLastProcessedState=mState;'''
    assert text.count(old) == 1, 'Unexpected upstream Tracking.cc initialization'
    text = text.replace(old, new)
    old = '// Localization Mode: Local Mapping is deactivated (TODO Not available in inertial mode)\n            if(mState==LOST)'
    new = '// Localization Mode: Local Mapping is deactivated (TODO Not available in inertial mode)\n            if(mState==LOST || mState==RECENTLY_LOST)'
    assert text.count(old) == 1, 'Unexpected upstream Tracking.cc localization state'
    p.write_text(text.replace(old, new))
