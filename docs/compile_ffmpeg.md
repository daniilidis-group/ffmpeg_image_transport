# How to compile FFmpeg from scratch for your project

## Prepare for FFmpeg installation

Install ffmpeg build dependencies according to
[this website](https://trac.ffmpeg.org/wiki/CompilationGuide/Ubuntu): 

    sudo apt-get update -qq && sudo apt-get -y install \
    autoconf \
    automake \
    build-essential \
    cmake \
    git-core \
    libass-dev \
    libfreetype6-dev \
    libsdl2-dev \
    libtool \
    libva-dev \
    libvdpau-dev \
    libvorbis-dev \
    libxcb1-dev \
    libxcb-shm0-dev \
    libxcb-xfixes0-dev \
    pkg-config \
    texinfo \
    wget \
    zlib1g-dev \
	libx264-dev \
	libx265-dev

Make your own ffmpeg directory:

    ffmpeg_dir=<absolute_path_to_empty_directory_for_ffmpeg_stuff>
    mkdir -p $ffmpeg_dir/build $ffmpeg_dir/bin

Build yasm:

    cd $ffmpeg_dir
    wget -O yasm-1.3.0.tar.gz https://www.tortall.net/projects/yasm/releases/yasm-1.3.0.tar.gz && \
    tar xzvf yasm-1.3.0.tar.gz && \
    cd yasm-1.3.0 && \
    ./configure --prefix="$ffmpeg_dir/build" --bindir="$ffmpeg_dir/bin" && \
    make && \
    make install

Build nasm:


    cd $ffmpeg_dir
    wget https://www.nasm.us/pub/nasm/releasebuilds/2.13.03/nasm-2.13.03.tar.bz2 && \
    tar xjvf nasm-2.13.03.tar.bz2 && \
    cd nasm-2.13.03 && \
    ./autogen.sh && \
    PATH="${ffmpeg_dir}/bin:$PATH" ./configure --prefix="${ffmpeg_dir}/build" --bindir="${ffmpeg_dir}/bin" && \
    make && \
    make install


Get nvcodec headers:


    cd $ffmpeg_dir
    git clone https://git.videolan.org/git/ffmpeg/nv-codec-headers.git
    cd nv-codec-headers

At this point you will have to pick the right version of the headers. You need the one that matches
your driver. For instance:

    git checkout n8.2.15.8

will get you a version that works with with Linux 410.48 or newer (see README file). If you mess
up here, you will later get an error that looks like this:

    [hevc_nvenc @ 0x7fc67c03a580] Cannot load libnvidia-encode.so.1
    [hevc_nvenc @ 0x7fc67c03a580] The minimum required Nvidia driver for nvenc is 418.30 or newer

Now create a modified Makefile that points to the right location:

    tmp_var="${ffmpeg_dir//"/"/"\/"}"
    sed "s/\/usr\/local/${tmp_var}\/build/g" Makefile > Makefile.tmp
    make -f Makefile.tmp install


# Compile FFMpeg

Clone and configure ffmpeg to your needs. The following configuration gives you
cuda and the hardware accelerated nvidia (nvenc) encoding for x264/x265.
This builds an ffmpeg that has more components than you need.

    cd $ffmpeg_dir
    git clone https://github.com/FFmpeg/FFmpeg.git	
    cd $ffmpeg_dir/FFmpeg

To build full NVidia encode/decode support, do this:

    PATH="$ffmpeg_dir/bin:$PATH" PKG_CONFIG_PATH="$ffmpeg_dir/build/lib/pkgconfig" ./configure --prefix=${ffmpeg_dir}/build --extra-cflags=-I${ffmpeg_dir}/build/include --extra-ldflags=-L${ffmpeg_dir}/build/lib --bindir=${ffmpeg_dir}/bin --enable-cuda-nvcc --enable-cuvid --enable-libnpp --extra-cflags=-I/usr/local/cuda/include/ --extra-ldflags=-L/usr/local/cuda/lib64/ --enable-gpl --enable-nvenc --enable-libx264 --enable-libx265 --enable-nonfree --enable-shared

If you don't have an NVidia card, do this:

    PATH="$ffmpeg_dir/bin:$PATH" PKG_CONFIG_PATH="$ffmpeg_dir/build/lib/pkgconfig" ./configure --prefix=${ffmpeg_dir}/build --extra-cflags=-I${ffmpeg_dir}/build/include --extra-ldflags=-L${ffmpeg_dir}/build/lib --bindir=${ffmpeg_dir}/bin --enable-gpl --enable-libx264 --enable-libx265 --enable-nonfree --enable-shared

Now build and install (runs for a few minutes!):

    PATH="$ffmpeg_dir/bin:${PATH}:/usr/local/cuda/bin" make && make install && hash -r


