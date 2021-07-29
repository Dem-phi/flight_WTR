#! /bin/sh

mkdir build
mkdir include
mkdir src

cp ~/backups/CMakeLists.txt .
touch readme.md
cat >> ./readme.md << EOF
需在当前目录下CMakeLists.txt中补全
PROJECT_NAME
FILE_NAME (可执行文件名)
其他根据需求解注释使用

请在build目录下进行
cmake ..
make

生成的可执行文件在build/bin目录下
EOF
tree
