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
    - Needed for some parts of the demo only
    - Used in morphing audio in python, no Praat
    - BSD license

### Praat

To morph audio using Praat, you must have [Praat](www.praat.org) installed.
Then, add the path to the Praat executable to your `PATH`. For example, if you've put Praat in the directory `/home/myusername/bin`, you would run:

`export PATH=/home/myusername/bin:$PATH`

If you don't want to remember to run this, add it to your shell config file (e.g., .bashrc or config.fish).

On Ubuntu, Praat requires libstdc++ with symbol versioning for GLIBCXX\_3.4.21
which isn't in Ubuntu releases prior to 16.04. As a workaround, you can embed a
newer version of libstdc++ from Ubuntu 16.04 for Praat to use. Run:

`source libpath_for_praat.sh`

This will add the newer version of libstdc++, which is in the `lib/` directory,
to the `LD_LIBRARY_PATH`, which means it'll be loaded in preference to the one
in `/usr/lib/` that's included in the OS. Note that this is a hack, and if
weird stuff happens or some Praat things don't work as expected, this could be
the cause.

To check what symbol versioning your libstdc++ has, you can run:

`strings /usr/lib/i386-linux-gnu/libstdc++.so.6 | grep GLIBC`

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
- `-o [OUT_FILE]`, `--outfile [OUT_FILE]`: Optional filename for morphed audio (.wav).
- `-d [OUT_DIR]`, `--outdir [OUT_DIR]`: Optional directory for saving audio.
  Default is the current working directory.

## Version notes

This program was developed and tested with:

- Python 2.7.6
- Praat 6.0.29
- Mac OS X 10.10.5 (demo only)
- Ubuntu 14.04 LTS 32-bit (demo and ROS node)
- ROS Indigo

## Bugs and issues

Please report all bugs and issues on the [rr_audio_entrainer github issues
page](https://github.com/mitmedialab/rr_audio_entrainer/issues).


