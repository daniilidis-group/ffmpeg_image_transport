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


## Requirements: FFmpeg v4.0 or later, and a "reasonable" resolution

If you have ffmpeg 4.0, you may be able to compile against default header files:

	sudo apt install ffmpeg

If you don't have ffmpeg 4.0 or later, or you hit compile errors
(supposedly even happens now on Ubuntu 18.04), 
[follow these instructions to compile ffmpeg from scratch](docs/compile_ffmpeg.md), and point the
transport to the right place: 

    ffmpeg_dir=<here the ffmpeg_dir>
    catkin config -DCMAKE_BUILD_TYPE=Release -DFFMPEG_LIB=${ffmpeg_dir}/build/lib -DFFMPEG_INC=${ffmpeg_dir}/build/include

Note that not all resolutions are supported in hardware accelerated
mode for h264 encoding/decoding, so you may get strange results if you
have cameras with resolutions other than those supported by
e.g. nvenc. Plain 1920x1200 works, probably also VGA, but other
resolutions need to be verified. Experiments indicate that the line
width must be a multiple of 32!

## Compiling

This should be easy as running the following inside your catkin workspace

    catkin config -DCMAKE_BUILD_TYPE=Release

or, if you have your own version of ffmpeg installaed under ``${ffmpeg_dir`` (see above)

    catkin config -DCMAKE_BUILD_TYPE=Release -DFFMPEG_LIB=${ffmpeg_dir}/build/lib -DFFMPEG_INC=${ffmpeg_dir}/build/include

then compile should be as easy as this:

    catkin build ffmpeg_image_transport

## How to use

Usage should be transparent, but the ffmpeg image transport plugin
needs to be available on both the publishing and the subscribing
node. You also *need to add the location to your custom built ffmpeg
library* to ``LD_LIBRARY_PATH``, like so (change path to match yours):
```
export LD_LIBRARY_PATH=$HOME/catkin_ws/ffmpeg/build/lib:$LD_LIBRARY_PATH
```
Once done, you should get the following output when you run
``list_transports``:
```
rosrun image_transport list_transports 
Declared transports:
... some stuff ...
image_transport/ffmpeg
... some stuff ...

Details:
----------
... some stuff ...
"image_transport/ffmpeg"
 - Provided by package: ffmpeg_image_transport
 - Publisher: 
      This plugin encodes frame into ffmpeg compressed packets
    
 - Subscriber: 
      This plugin decodes frame from ffmpeg compressed packets
```

If it says something about "plugins not built", that means the
``LD_LIBRARY_PATH`` is not set correctly.


## Trouble shooting:


On e.g. Ubuntu 16.04, you need a newer version of ffmpeg. If you get an error like this one,
you need a newer version of ffmpeg:

    In member function ‘bool ffmpeg_image_transport::FFMPEGDecoder::decodePacket(const ConstPtr&)’:
    /home/pfrommer/Documents/birds/src/ffmpeg_image_transport/src/ffmpeg_decoder.cpp:104:47:
    error: ‘avcodec_send_packet’ was not declared in this scope
    int ret = avcodec_send_packet(ctx, &packet);

If the build still fails, make sure you start from scratch:

    catkin clean ffmpeg_image_transport
