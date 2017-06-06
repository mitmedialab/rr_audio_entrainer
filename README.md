# Audio entrainment

Entrain an audio file (the source) to match an audio stream from the microphone
or another wav file (the target). Detect various features in the target, such
as speaking rate and pitch, and morph the source to match these features.

## Setup and Dependencies

`pip install` the following libraries...

- aubio
    - for pitch detection on the incoming audio
    - GNU/GPL3 license
- librosa
    - to morph audio in python, no Praat
    - ISC license
- PyAudio
    - to open and read from the microphone
    - MIT license
- numpy
    - because PyAudio uses numpy arrays for holding data
    - BSD-new license
- scipy
    - used in morphing audio in python, no Praat
    - BSD license

To morph audio using Praat, you must have [Praat](www.praat.org) installed.
List the location of the Praat executable in `entrain_speech.py`.

## Usage

`python entrain_speech.py [-h] [-p] [-i [INCOMING_AUDIO]] [-o [OUT_FILE]] [-d
[OUT_DIR]] [audio_to_morph]`

positional arguments:
- `audio_to_morph`: Audio file to morph (.wav).

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
- Mac OS X 10.10.5

## Bugs and issues

Please report all bugs and issues on the [rr_audio_entrainer github issues
page](https://github.com/mitmedialab/rr_audio_entrainer/issues).


