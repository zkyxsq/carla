echo "Building FBX2OBJ"
echo "----------------"

FBXSDK_URL=https://www.autodesk.com/content/dam/autodesk/www/adn/fbx/2020-0-1/fbx202001_fbxsdk_linux.tar.gz

DOCKER_UTILS_DIR=Util/DockerUtils
FBX_DIR=${DOCKER_UTILS_DIR}/dist/fbx
FBX_INSTALL=${FBX_DIR}/fbx_install
INCLUDE_DIR=${FBX_INSTALL}/include
BUILD_DIR=${FBX_DIR}/build

mkdir -p ${DOCKER_UTILS_DIR}/dist
cp ${DOCKER_UTILS_DIR}/addOBJ.py ${DOCKER_UTILS_DIR}/dist/
cp ${DOCKER_UTILS_DIR}/build.sh ${DOCKER_UTILS_DIR}/dist/
cp ${DOCKER_UTILS_DIR}/get_xodr_crosswalks.py ${DOCKER_UTILS_DIR}/dist/
cp -r ${DOCKER_UTILS_DIR}/fbx/ ${DOCKER_UTILS_DIR}/dist/

echo "Downloaing FBX SDK 2020"
wget -c ${FBXSDK_URL} -O ${FBX_DIR}/fbx202001_fbxsdk_linux.tar.gz

echo "Unpacking"
tar -xvzf ${FBX_DIR}/fbx202001_fbxsdk_linux.tar.gz -C ${FBX_DIR}
rm ${FBX_DIR}/fbx202001_fbxsdk_linux.tar.gz

echo "Installing"
mkdir -p ${FBX_INSTALL}
printf "y\nyes\nn\n" | ./${FBX_DIR}/fbx202001_fbxsdk_linux ${FBX_INSTALL}
rm ${FBX_DIR}/fbx202001_fbxsdk_linux
rm ${FBX_DIR}/Install_FbxSdk.txt

echo "Compiling FBX2OBJ..."
mkdir -p ${BUILD_DIR}
cmake -B${BUILD_DIR} -S${FBX_DIR}
make -C ${BUILD_DIR}

echo "Copy binary FBX2OBJ"
mv ${BUILD_DIR}/FBX2OBJ ${DOCKER_UTILS_DIR}/dist/
rm -Rf ${BUILD_DIR}
