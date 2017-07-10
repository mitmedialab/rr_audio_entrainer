# Audio entrainment

Entrain an audio file (the source) to match an audio stream from the microphone
or another wav file (the target). Detect various features in the target, such
as speaking rate and pitch, and morph the source to match these features.

## Setup and Dependencies

### Python
`pip install` the following libraries (ideally in your virtualenv)...

- aubio
    - For pitch detection on the incoming audio
    - GNU/GPL3 license
- librosa
    - Needed for some parts of the demo only
    - To morph audio in python, no Praat
    - ISC license
- portaudio
    - Needed for PyAudio
    - Sometimes pip has issues finding the portaudio.h file if you don't
      install this manually.
    - On Ubuntu 14.04, you may need to install portaudio19-dev instead of
      libportaudio for it to compile successfully. You may need to sudo apt-get
      this instead of pip installing it.
- PyAudio
    - To open and read from the microphone
    - MIT license
- numpy
    - Because PyAudio uses numpy arrays for holding data
    - BSD-new license
- scipy
    - Used to read in wav files
    - Used in morphing audio in python, no Praat
    - BSD license
    - You may need to sudo apt-get this because of lapack/blas.

#### PYTHONPATH
You will probably have to add python's default path to your shell config. For
Ubuntu 14.04, the line to add is:

`export PYTHONPATH=$PYTHONPATH:/usr/lib/python2.7/dist-packages/'

You have to add the default location to `PYTHONPATH` because initially,
`PYTHONPATH` is not set, and thus python defaults to checking a default
location...  but other shell config files may add to `PYTHONPATH`, thus
_setting_ `PYTHONPATH`, which means python no longer checks its default
location...  So we have to add the default back into the path.

In particular, if you have added the ROS setup shell config files to your shell
config file (e.g., if you have a line like `source /opt/ros/indigo/setup.bash`
in your .bashrc), you'll need to add this line.

`

### Praat

To morph audio using Praat, you must have [Praat](http://www.praat.org) installed. The
version you can install from ports on linux is old and the Praat script may not
run correctly, so I recommend installing the binary from the website. Then, add
the path to the Praat executable to your `PATH`. For example, if you've put
Praat in the directory `/home/myusername/bin`, you would run:

`export PATH=/home/myusername/bin:$PATH`

If you don't want to remember to run this, add it to your shell config file
(e.g., .bashrc or config.fish).

On Ubuntu, Praat requires libstdc++ with symbol versioning for GLIBCXX\_3.4.21
which isn't in Ubuntu releases prior to 16.04. As a workaround, you can embed a
newer version of libstdc++ from Ubuntu 16.04 for Praat to use. Run:

`source libpath_for_praat.sh`

This will add the newer version of libstdc++, which is in the `lib/` directory,
to the `LD_LIBRARY_PATH`, which means it'll be loaded in preference to the one
in `/usr/lib/` that's included in the OS. Note that this is a hack, and if
weird stuff happens or some Praat things don't work as expected, this could be
the cause.

To check what symbol versioning your libstdc++ has,on 32-bit Ubuntu, you can run:

`strings /usr/lib/i386-linux-gnu/libstdc++.so.6 | grep GLIBC`

For 64-bit Ubuntu, replace the path with 64-bit Ubuntu's path to
`libstdc++.so.6`.


## Demo

### Usage

`python entrain_speech.py [-h] [-p] [-i [INCOMING_AUDIO]] [-o [OUT_FILE]] [-d
[OUT_DIR]] [audio_to_morph] [target_age]`

positional arguments:
- `audio_to_morph`: Audio file to morph (.wav).
- `target_age`: Age of the speaker providing audio to match.

optional arguments:
- `-h`, `--help`: show this help message and exit
- `-p`, `--use_praat`: Use the built-in Praat script for audio processing
- `-i [INCOMING_AUDIO]`, `--incoming_audio [INCOMING_AUDIO]`: Optional audio
  file to match (.wav).
- `-o [OUT_FILE]`, `--outfile [OUT_FILE]`: Optional filename for morphed audio
  (.wav).
- `-d [OUT_DIR]`, `--outdir [OUT_DIR]`: Optional directory for saving audio.
  Default is the current working directory.

## ROS node

If you are using a virtualenv, make sure to source the `activate` shell script.

If you are using Ubuntu 14.04, make sure to source the `libpath_for_praat.sh`
script.

### Usage
`entrain_speech_node.py [-h] [-i IP_ADDR] [-d [OUT_DIR]] [-r USE_ROS]`

Given an audio stream over ROS, detect various audio features. Morph an audio
file (specified via a ROS msg) to match those features. Only use audio
collected during the participant's turn to speak, when the participant is
speaking (both also specified via ROS msgs). Send the morphed file to a robot.
Also save the morphed file to the specified output directory.

optional arguments:
  - `-h`, `--help`: show this help message and exit
  - `-i IP_ADDR`, `--ipaddr IP_ADDR`: The IP address of the machine running
    this node. Used to serve audio files to the robot.
  - `-d [OUT_DIR]`, `--outdir [OUT_DIR]`: Optional directory for saving audio.
    Default is the current working directory.
  - `-r USE_ROS`, `--use-ros USE_ROS`: Use a local microphone or an audio
    stream ROS from an Android mic (i.e., the robot). Default local mic.

For the audio files to be streamed, for now, from the directory where your
audio will be saved, you will need to run:

`python -m SimpleHTTPServer`

In the future, this functionality will be incorporated into the node itself.

### ROS messages

#### R1D1 messages

The node subscribes to "/[r1d1\_msgs](https://github.com/mitmedialab/r1d1_msgs
"/r1d1_msgs")/AndroidAudio" on the ROS topic "/android_audio".

#### Speaking binary

The node subscribes to Int32 messages from the [speaking binary
classifier](https://github.com/mitmedialab/Moody_BackChanneling) on the ROS
topic "/msg_sb/raw".

#### Relational robot messages

The node subscribes to
"/[rr_msgs](https://github.com/mitmedialab/rr_msgs)/EntrainAudio" messages on
the ROS topic "/rr/entrain_audio".

The node also subscribes to
"/[rr_msgs](https://github.com/mitmedialab/rr_msgs)/InteractionState" messages
on the ROS topic "/rr/state".


## Version notes

This program was developed and tested with:

- Python 2.7.6
- Praat 6.0.29
- Mac OS X 10.10.5 (demo only)
- Ubuntu 14.04 LTS 32-bit (demo and ROS node)
- Ubuntu 14.04 LTS 64-bit (ROS node only)
- ROS Indigo (ROS node only)

## Bugs and issues

Please report all bugs and issues on the [rr_audio_entrainer github issues
page](https://github.com/mitmedialab/rr_audio_entrainer/issues).


