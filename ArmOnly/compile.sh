#!/bin/bash   
clear
      
echo "Checking input arguments.............."
if [[ $# -eq 0 ]] ; then
    echo "MISSING Filename !!!"
    echo "Usage: ./compile_caffe_cpp.sh filename.cpp"
    exit 1
else
	filename=$(basename "$1")
	output="${filename%.*}"
	echo "Compiling $1 file.............."
	g++ -I/usr/local/include/opencv -I/usr/local/include/opencv2 -I/usr/local/cuda-8.0/include -L/usr/local/lib/ -L/usr/local/cuda-8.0/lib64  -g -o $output $1 abhi.cpp -lopencv_calib3d -lopencv_core -lopencv_xfeatures2d -lopencv_features2d -lopencv_ximgproc -fopenmp -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_photo -lpmdaccess2 -lPvAPI -lImagelib -lQt5Widgets -lQt5Gui -lQt5Test -lQt5Concurrent -lQt5Core -ldxl_x64_cpp -lglog -lgflags -lprotobuf -lphidget21  -lcaffe -lcublas -lcudart -lcudnn -lboost_system -lpcl_visualization -lpcl_common -lvtkRenderingCore-6.2 -lvtkCommonDataModel-6.2 -lvtkCommonMath-6.2 -lvtkCommonCore-6.2 
fi

echo "Compilation done........ output saved as..... $output"
