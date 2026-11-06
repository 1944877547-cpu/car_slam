#!/usr/bin/env bash
# 下载固定版本并在当前机器编译；PC和NX使用相同命令。
set -euo pipefail
orb_package=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
orb_workspace=$(cd -- "$orb_package/../.." && pwd)
orb_deps="$orb_workspace/.orbslam3"
orb_jobs=${1:-2}
[[ "$orb_jobs" =~ ^[1-9][0-9]*$ ]] || { echo 'Usage: bash build_orb_slam3.sh [jobs]'; exit 1; }
mkdir -p "$orb_deps"
touch "$orb_deps/COLCON_IGNORE"

fetch_source() {
  local url=$1 revision=$2 directory=$3
  if [[ ! -d "$directory/.git" ]]; then
    if [[ -e "$directory" ]]; then echo "Not a Git checkout: $directory"; exit 1; fi
    git init -q "$directory"
    git -C "$directory" remote add origin "$url"
  fi
  if [[ $(git -C "$directory" rev-parse HEAD 2>/dev/null || true) != "$revision" ]]; then
    if [[ -n $(git -C "$directory" status --porcelain) ]]; then
      echo "Source contains local changes; please check $directory"; exit 1
    fi
    git -c http.version=HTTP/1.1 -C "$directory" fetch --depth 1 origin "$revision"
    git -C "$directory" checkout --detach FETCH_HEAD
  fi
}

fetch_source https://github.com/UZ-SLAMLab/ORB_SLAM3.git \
  4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4 "$orb_deps/ORB_SLAM3"
fetch_source https://github.com/stevenlovegrove/Pangolin.git \
  dd801d244db3a8e27b7fe8020cd751404aa818fd "$orb_deps/Pangolin"
python3 "$orb_package/scripts/prepare_core.py" "$orb_deps/ORB_SLAM3"

cmake -S "$orb_deps/Pangolin" -B "$orb_deps/Pangolin/build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$orb_deps/prefix" \
  -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_TESTS=OFF \
  -DBUILD_PANGOLIN_VIDEO=OFF -DBUILD_PANGOLIN_PYTHON=OFF
cmake --build "$orb_deps/Pangolin/build" -j "$orb_jobs"
cmake --install "$orb_deps/Pangolin/build"
cmake -S "$orb_deps/ORB_SLAM3/Thirdparty/DBoW2" -B "$orb_deps/ORB_SLAM3/Thirdparty/DBoW2/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$orb_deps/ORB_SLAM3/Thirdparty/DBoW2/build" -j "$orb_jobs"
cmake -S "$orb_deps/ORB_SLAM3" -B "$orb_deps/ORB_SLAM3/build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$orb_deps/prefix" \
  -DCMAKE_DISABLE_FIND_PACKAGE_realsense2=ON
# 官方CMake通过绝对路径链接g2o，先显式生成库以避免并行构建竞态。
cmake --build "$orb_deps/ORB_SLAM3/build" --target g2o -j "$orb_jobs"
cmake --build "$orb_deps/ORB_SLAM3/build" --target ORB_SLAM3 -j "$orb_jobs"
if [[ ! -s "$orb_deps/ORB_SLAM3/Vocabulary/ORBvoc.txt" ]]; then
  tar -xzf "$orb_deps/ORB_SLAM3/Vocabulary/ORBvoc.txt.tar.gz" -C "$orb_deps/ORB_SLAM3/Vocabulary"
fi
printf '\nORB-SLAM3 ready. Next: colcon build --packages-select mycar_orbslam3 --cmake-args -DCMAKE_BUILD_TYPE=Release\n'
