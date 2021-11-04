# ROS image transport for FFmpeg encoding

This ROS image transport supports encoding/decoding with the FFMpeg
library. With this transport, you can encode h264 and h265, using
nvidia hardware acceleration when available.

Also have a look at [this repo tools to help with decoding](https://github.com/daniilidis-group/ffmpeg_image_transport_tools).

## Downloading

Create a catkin workspace (if not already there) and download the
ffmpeg image transport and other required packages:

    mkdir -p catkin_ws/src
    cd catkin_ws/src
    git clone https://github.com/daniilidis-group/ffmpeg_image_transport_msgs.git
    git clone https://github.com/daniilidis-group/ffmpeg_image_transport.git

Optionally get the additional set of ffmpeg tools:

    git clone git@github.com:daniilidis-group/ffmpeg_image_transport_tools.git


## Requirements: FFmpeg v4.0 or later, and a "reasonable" resolution

At some point this software worked on the following systems:

- Ubuntu 16.04 + ROS kinetic (not tested in a long while, maybe broken
  by now)
- Ubuntu 18.04 + ROS melodic
- Ubuntu 20.04 + ROS noetic (recently tested, should work with stock ffmpeg)

If you have ffmpeg 4.0 or later you may be able to compile against default header files:

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

### republishing
Most ROS nodes that publish images use an image transport for that. If
not you can use a republish node to convert images into an ffmpeg
stream. The following line will start a republish node in ffmpeg format:

```
rosrun image_transport republish raw in:=/webcam/image_raw ffmpeg out:=/webcam/image_raw/ffmpeg _/webcam/image_raw/ffmpeg/encoding:=x264
```

### encoder parameters

The image transport has various parameters that you can configure
dynamically via ``rqt_reconfigure``, or specify at startup by
prepending the topic name. For instance to use the unaccelerated
``libx264`` encoding, start a republish node like this:
```
rosrun image_transport republish raw in:=/webcam/image_raw ffmpeg __name:=repub out:=~/image_raw  _/image_raw/ffmpeg/encoder:=libx264
```
Note: I had to ``__name`` the node to get the ros parameter show up under the topic name.

Parameters (see ffmpeg h264 documentation for explanation):

- ``encoder``: ffmpeg encoding to use. Allowed values: ``h265_nvenc``,
  ``hevc_nvenc``, ``libx264`` (no GPU). Default: ``hevc_nvenc``.
- ``profile``: ffmpeg profile. Allowed values: ``main``, ``main10``,
  ``rext``, ``high``. Default: ``main``.
- ``qmax``: maximum allowed quantization, controls quality. Range 0-63, default: 10.
- ``bit_rate``: bit rate in bits/sec. Range 10-2000000000, default: 8242880.
- ``gop_size``: gop size (number of frames): Range 1-128, default: 15
- ``measure_performance``: switch on performance debugging output. Default: false.
- ``performance_interval``: number of frames between performance printouts. Default: 175.

### encoder parameters

Usually the decoder infers from the received encoding type what decoding scheme to use.
You can overwrite it with a node-level parameter though:

- ``decoder_type``:
  - ``h264``: used if encoding is ``libx264`` or ``h264_nvenc``.
  - ``hevc_cuvid``: first tried if encoding is ``hevc_nvenc``.
  - ``hevc``: second tried if encoding is ``hevc_nvenc``.

For example to force ``hevc`` decoding, run a node like this:
```
rosrun image_transport republish ffmpeg in:=/repub/image_raw raw out:=~/image_raw _/ffmpeg/decoder_type:=hevc
```

## Trouble shooting:


On e.g. Ubuntu 16.04, you need a newer version of ffmpeg. If you get an error like this one,
you need a newer version of ffmpeg:

    In member function ‘bool ffmpeg_image_transport::FFMPEGDecoder::decodePacket(const ConstPtr&)’:
    /home/pfrommer/Documents/birds/src/ffmpeg_image_transport/src/ffmpeg_decoder.cpp:104:47:
    error: ‘avcodec_send_packet’ was not declared in this scope
    int ret = avcodec_send_packet(ctx, &packet);

If the build still fails, make sure you start from scratch:

    catkin clean ffmpeg_image_transport
