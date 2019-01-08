# ROS image transport for FFmpeg encoding

This ROS image transport supports encoding/decoding with the FFMpeg
library. With this transport, you can encode h264 and h265, using
nvidia hardware acceleration when available.

## Downloading

Create a catkin workspace (if not already there) and download the
ffmpeg image transport and other required packages:

    mkdir -p catkin_ws/src
    cd catkin_ws/src
    git clone https://github.com/daniilidis-group/ffmpeg_image_transport_msgs.git
    git clone https://github.com/daniilidis-group/ffmpeg_image_transport.git


## Requirements: FFmpeg v4.0 or later

If you have ffmpeg 4.0, you may be able to compile against default header files:

	sudo apt install ffmpeg

If you don't have ffmpeg 4.0 or later,
[compile ffmpeg from scratch](docs/compile_ffmpeg.md), and point the
transport to the right place: 

    ffmpeg_dir=<here the ffmpeg_dir>
    catkin bt -DFFMPEG_LIB=${ffmpeg_dir}/build/lib -DFFMPEG_INC=${ffmpeg_dir}/build/include

## Compiling

This should be easy as running the following inside your catkin workspace

    catkin config -DCMAKE_BUILD_TYPE=Release
    catkin build ffmpeg_image_transport
	
Then compile:

    cd ffmpeg_image_transport

## Trouble shooting:


On e.g. Ubuntu 16.04, you need a newer version of ffmpeg. If you get an error like this one,
you need a newer version of ffmpeg:

    In member function ‘bool ffmpeg_image_transport::FFMPEGDecoder::decodePacket(const ConstPtr&)’:
    /home/pfrommer/Documents/birds/src/ffmpeg_image_transport/src/ffmpeg_decoder.cpp:104:47:
    error: ‘avcodec_send_packet’ was not declared in this scope
    int ret = avcodec_send_packet(ctx, &packet);

If the build still fails, make sure you start from scratch:

    catkin clean ffmpeg_image_transport
