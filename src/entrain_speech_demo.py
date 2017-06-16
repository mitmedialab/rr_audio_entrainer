#! /usr/bin/env python
#
# Jacqueline Kory Westlund
# May 2017
#
# Entrain an outgoing/source audio file to an incoming target. Detect various
# features of the incoming signal (e.g., pitch, speaking rate, intensity) and
# morph the outgoing sound to match.
# Theoretically, there is the option to use python or Praat to detect and morph,
# but currently, only the Praat script is being developed.
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program.  If not, see http://www.gnu.org/licenses/


import argparse
import pyaudio # (MIT license)
import numpy
import aubio # pitch detection (GNU/GPL license)
from collections import deque # queue for incoming audio
import os
import subprocess
import wave # for saving wav files
import struct
# Needed only if not using Praat for audio file processing:
#import scipy.ndimage.interpolation
#import librosa (ISC license)

class EntrainAudio():
    """ Given an audio stream and an audio file, detect the pitch and tempo of
    the stream, then modify the pitch and tempo of the second to match.
    """

    def __init__(self):
        """ Initialize entrainer for detecting pitch and tempo. """
        # Where is Praat?
        self.praat = "/Applications/Praat.app/Contents/MacOS/Praat"
        # Where is the Praat script we will use for processing?
        self.script = "rr_entrain_speech.praat"

        # Define pitch ranges that will be considered speech.
        # - Human child speech: generally 250-400Hz.
        # - Human adult male speech: generally 85-180Hz.
        # - Human adult female speech: generally 165-255Hz.
        self._floor_pitch = 100
        self._ceiling_pitch = 600
        # For audio recording.
        self._buffer_size = 2048
        self._samplerate = 44100 #TODO Pass sample rate in as argument?
        self._n_channels = 1

        # Set up to detect pitch.
        _tolerance = 0.8
        _win_s = 4096 # fft size
        _hop_s = self._buffer_size # hop size
        # Set up the aubio pitch detector.
        self.pitch_detector = aubio.pitch("default", _win_s, _hop_s, self._samplerate)
        self.pitch_detector.set_unit("Hz")
        self.pitch_detector.set_tolerance(_tolerance)


    def _get_mean_ff(self, age):
        """ Return the mean fundamental frequency for a given age group.
        """
        if age < 4:
            # No data. Some research suggests their pitch may be higher.
            return 260

        # Based on data from Geifer and Denor 2014, Hacki and Heitmuller 1999,
        # Sorenson 1989, Bennett 1983, Weinberg and Zlatin 1970, Baker et al.
        # 2008. None of these papers found significant gender differences at
        # these ages.
        if age == 5:
            # girls: 247, 294
            # boys: 252, 262
            # both: 232
            # mean of all these: 257
            return 257
        if age == 6:
            # girls: 247, 296, 262
            # boys: 247, 250, 277
            # both: 245, 241
            # mean of all these: 258
            return 258
        if age == 7:
            # girls: 257, 277
            # boys: 288, 277
            # both: 230, 253
            # mean of all these: 263
            return 263
        if age == 8:
            # girls: 235, 251, 247
            # boys: 234, 229, 262
            # both: 236, 240
            # mean of all these: 241
            return 241
        if age == 9:
            # girls: 222, 266, 262
            # boys: 226, 221, 233
            # mean of all these: 238
            return 238
        if age == 10:
            # girls: 228, 229
            # boys: 224, 230
            # mean of all these: 227
            return 227

        if age > 10:
            # No specific data. Adults tend to have lower ranges.
            # - Human adult male speech: generally 85-180Hz.
            # - Human adult female speech: generally 165-255Hz.
            # So as not to gender the robot too much, leave its voice in the
            # higher child-like range, as this fits its persona.
            return 220


    def entrain_from_mic(self, source_file, out_file, out_dir, use_praat,
            target_age):
        """ Open the microphone to get an incoming audio stream. If the
        use_praat flag is set, save the first section of audio that is probably
        speech to a wav file and use that as the target when processing with
        Praat.
        """
        # Initialize pyaudio.
        pyaud = pyaudio.PyAudio()

        # Open stream.
        pyaudio_format = pyaudio.paInt16
        stream = pyaud.open(format=pyaudio_format,
                        channels=self._n_channels,
                        rate=self._samplerate,
                        input=True,
                        frames_per_buffer=self._buffer_size)

        # Process microphone stream until we get a keyboard interrupt.
        # Set up a deque to hold incoming data, with a max length, so that when
        # it gets full, the oldest items are automatically discarded. Use one
        # to get the raw data, and another for the pitch values.
        frames = deque([], maxlen=600) #TODO what's a good size?
        f = []
        incoming_pitches = deque([], maxlen=600)

        # TODO: Listen on the microphone only when we know it's the child's turn
        # to speak (and process only the audio collected during their speech
        # turn). For now, assume that it's the child's turn when the program
        # starts, and that the robot's turn starts on a key press.

        # Running total of potential speech frames.
        running_total_speech = 0
        running_total_silence = 0

        while True:
            try:
                # Detect pitch.
                audiobuffer = stream.read(self._buffer_size)
                signal = numpy.fromstring(audiobuffer,
                        dtype=numpy.int16).astype(numpy.float32)
                pitch = self.pitch_detector(signal)[0]
                confidence = self.pitch_detector.get_confidence()
                print("{} / {}".format(pitch,confidence))

                # TODO For now, we just check the pitch to see if it is probably
                # part of human speech or not. If we get enough values in a row
                # that are probably speech, we use those as the target signal
                # for morphing the specified audio file. If we get a long pause,
                # however, we reset, and wait until we get sufficient speech in
                # a row to act as a target. This will probably have to change to
                # something more intelligent later. For example, we could check
                # if the values are over some threshold for volume or energy.
                # Or, use the ROS speaking binary, and stop processing when
                # speech stops.
                if pitch > self._floor_pitch and pitch < self._ceiling_pitch:
                    running_total_speech += 1
                    if running_total_speech > 4:
                        # Potential speech. Start tracking.
                        print("\tspeech?")
                        # Reset silence counter.
                        running_total_silence = 0
                        # Save running stream of pitches.
                        incoming_pitches.append(pitch)
                # If the pitch is zero, there's probably no speech.
                elif pitch < 1 or pitch > self._ceiling_pitch:
                    running_total_silence +=1
                    # If we've had a lot of silence in a row, or non-speech
                    # sound, there's probably a pause or no speech.
                     # TODO Pick good values for these:
                    if running_total_silence > 6 and running_total_speech < 30:
                        print("\tsilence")
                        running_total_speech = 0
                    else:
                        # If it's just a short pause, we can keep it.
                        incoming_pitches.append(pitch)

                # Save running stream of raw audio.
                #TODO see if we can get deque to work
                if running_total_speech > 4:
                    frames.append(audiobuffer)
                    f.append(audiobuffer)

                # Check to see if we need to process the incoming audio yet.
                # If we have sufficient incoming audio and there's been a long
                # silence, stop listening on the mic and process.
                if len(incoming_pitches) >= 30 and running_total_silence > 5:
                    if use_praat:
                        # TODO file name for target? append date/time
                        # so we know later what was processed to get the
                        # morphed source?
                        target_file = "temp.wav"
                        wav_file = wave.open(target_file, 'wb')
                        wav_file.setnchannels(self._n_channels)
                        wav_file.setsampwidth(pyaud.get_sample_size(pyaudio_format))
                        wav_file.setframerate(self._samplerate)

                        wav_file.writeframes(b''.join(f))
                        #print("frames: {}".format(type(frames)))
                        #for frame in frames:
                            #wav_file.writeframes(frame)
                            #print("frame: {}".format(type(frame)))
                            #wav_file.writeframes(struct.pack('s'*len(frame), *frame))
                        wav_file.close()

                        # Use a Praat script to morph the source audio
                        # to match what's coming over the mic.
                        # TODO
                        self._entrain_from_file_praat(target_file, source_file,
                                out_file, out_dir, target_age)

                    # Or don't use Praat. Does not do as much, but is all
                    # in python.
                    else:
                        # Morph source audio file and write to new file.
                        self.morph_audio(incoming_pitches, source_file)
                    break



            # Stop processing.
            except KeyboardInterrupt:
                print("*** Keyboard interrupt, exiting")
                break
        # Close stream and clean up.
        stream.stop_stream()
        stream.close()
        pyaud.terminate()


    def entrain_from_file(self, target_file, source_file, out_file, out_dir,
            use_praat, target_age):
        """ Morph the given audio file to match the target file provided. Save
        with the given output file name in the given output directory. Use
        Praat for processing if specified. Currently, only Praat does all the
        processing.
        """
        if use_praat:
            self._entrain_from_file_praat(target_file, source_file, out_file,
                    out_dir, target_age)
        else:
            print("TODO: Process the target file and morph the source in python.")
            # TODO: Morph source audio file and write to new file.
            # Open and read file.
            # target = ...
            self.morph_audio(target, source_file)


    def _entrain_from_file_praat(self, target_file, source_file, out_file,
            out_dir, target_age):
        """ Use a Praat script to morph the given source audio file to match the
        specified target audio file, and save with the provided output file name
        to the specified output directory.
        """
        if not os.path.exists(self.praat):
            raise FileNotFound(self.praat)
        if not os.path.exists(self.script):
            raise FileNotFound(self.script)

        if out_file is None:
            out_file = source_file + "-morphed.wav"

        # Get mean fundamental frequency for the target age.
        ff_age = self._get_mean_ff(target_age)

        # The Praat script determines several features:
        #  - the speaking rate of the source and target files
        #  - the mean pitch of the source and target files
        # Then, we adjust several features of the source to match the target:
        #  - tempo / speaking rate
        #  - mean pitch (first to match the target's age, and then additionally
        #    within a specified range so it doesn't change too much)
        # and finally, save the adjusted source as a new wav.
        subprocess.call([self.praat, "--run", self.script, source_file,
            target_file, out_file, out_dir, str(ff_age)])

        # TODO error handling? what if praat fails to execute?



    def detect_tempo(self):
        """ Given a numpy array of amplitude samples, detect the tempo of the
        speech in the array.
        """
        #TODO
        print("""TODO: If desired, implement a python way of doing this.
                Currently, use Praat.""")
        # Use number of syllables per second as a measure of speech rate.
        # Detect syllable boundaries using acoustic-only features.
        # Or use mean syllable duration?


    def morph_tempo(self, target, source):
        """ Given a target numpy array of amplitude samples, speed up or slow
        down the source by the rate given.
        """
        # Get tempo of target audio.
        target_tempo = self.detect_tempo(target)
        print("target tempo: {}".format(target_tempo))

        # Get tempo of source audio.
        source_tempo = self.detect_tempo(source)
        print("source tempo: {}".format(target_tempo))

        # Determine difference between the two tempos.
        diff_tempo = 2 #TODO actually calculate this.
        # print("Difference: {}".format(diff_tempo))
        print("TODO: Implement tempo morphing in python. Currently, use Praat.")

        # Given a rate of how much faster or slower the target is compared to
        # the source, we can easily speed up or slow down the source to match.
        source_adjusted = self.adjust_tempo(source, diff_tempo)
        # And write out to a file.
        librosa.output.write_wav("morphed-tempo.wav", shifted, source[1])


    def adjust_tempo(self, source, rate):
        """ Given a rate, speed or slow down the source to match. """
        d = librosa.stft(source, n_fft=2048, hop_length=512)
        d_adjusted = librosa.phase_vocoder(d, rate, hop_length=512)
        source_adjusted = librosa.istft(d_adjusted, hop_length=512)
        return source_adjusted


    def detect_pitches(self, source):
        """ Given an array of amplitude samples, detect pitches. """
        pitches = []
        print("source length: {}".format(len(source)))
        for i in range(0,len(source)/2048):
            t = numpy.array(source[i*2048:i*2048+2048])
            pitch = self.pitch_detector(t)[0]
            confidence = self.pitch_detector.get_confidence()
            print("{} / {}".format(pitch,confidence))
            if pitch > self._floor_pitch and pitch < self._ceiling_pitch:
                pitches.append(pitch)
        return pitches


    def morph_mean_pitch(self, target, source, source_pitches):
        """ Morph the pitch of the source_file based on the pitch contour of the
            target.
            target: numpy array of pitches to match.
            source: numpy array of amplitude samples.
        """
        # Get the mean pitch of the target.
        mean_target_pitch = numpy.mean(target)
        print("Mean target pitch: {}".format(mean_target_pitch))

        # Get the mean pitch of the source file.
        mean_source_pitch = numpy.mean(source_pitches)
        print("Mean source pitch: {}".format(mean_source_pitch))

        # Find the difference between the target's mean pitch and the source's
        # mean pitch. This is still in Hz, but we need it in half-steps for the
        # pitch shifting function.
        diff_hz = mean_target_pitch - mean_source_pitch
        print("Difference between target and source means: {}".format(diff_hz))

        # The basic formula for note frequencies is:
        # fn = f0 * a^n
        # where fn is the source frequency, f0 is the target frequency, a is
        # 2^(1/12) = 1.05946.., and n is the number of half steps between the
        # frequencies. Solving for n, we get the formula:
        # log(fn/f0) / log(a) = n
        diff_steps = numpy.log(mean_source_pitch / mean_target_pitch) / \
                numpy.log(1.05946)

        # Use librosa to shift the pitch of the source file to match the mean
        # pitch of the target.
        shifted = librosa.effects.pitch_shift(source[0], source[1], n_steps=diff_steps)
        # And write out to a file.
        librosa.output.write_wav("morphed-mean-pitch.wav", shifted, source[1])


    def morph_pitch_contour(self, target, source, source_pitches):
        """ Morph the pitch contour of the source based on the pitch contour of
            the target.
            target: numpy array of pitches to match.
            source: numpy array of amplitude samples.
            source_pitches: numpy array of pitches in the source.
        """
        # Stretch or shrink target array to match length of source.
        # How many times bigger or smaller is the source?
        len_factor = len(source) / float(len(target))
        # Rebin array by the difference factor.
        target_rebin = scipy.ndimage.interpolation.zoom(target, len_factor)

        # Get the delta-pitch contour (i.e., the difference between successive
        # pitch values) for the target array.
        target_delta = numpy.diff(target_rebin)

        # Adjust the pitch contour of the source array to match the target.
        #TODO If desired, implement a python-only way of doing this. Currently,
        # use Praat.
        print("TODO: morph pitch contour in python-only way. Currently, if you "
                + "want the pitch morphed, use Praat.")

        # For each target pitch-frame, adjust the pitch of the corresponding
        # source frame.

        # And write out to a file.
        #librosa.output.write_wav("morphed-delta-pitch.wav", deltaed, source[1])



    def morph_audio(self, target, source_file):
        """ Morph the source based on features of the target.
            target: numpy array of incoming sound.
            source_file: .wav to morph.
        """
        # Print out our target array of pitches.
        print(target)
        # Use librosa to read in the source wav file as a numpy array.
        source = librosa.core.load(source_file)
        # Get source pitches.
        source_pitches = self.detect_pitches(source[0])
        print(source_pitches)

        # Morph mean pitch.
        self.morph_mean_pitch(target, source, source_pitches)

        # Morph pitch contour.
        self.morph_pitch_contour(target, source, source_pitches)

        # Then do both?
        # Scale delta pitches based on range and average pitch of source?
        # Morph tempo?
        # Morph energy?




if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description='''Given an audio stream from the mic (default) or an audio
            file, detect the pitch and tempo. Modify the pitch and tempo of a
            specified audio file to match.
            ''')
    parser.add_argument("-p", "--use_praat", action='store_true', default=True,
            help="Use the built-in Praat script for audio processing. Default" +
            " true.")
    parser.add_argument("audio_to_morph", type=str, nargs='?', action='store',
            default="sample.wav", help="Audio file to morph. Default sample " +
            "file.")
    parser.add_argument("target_age", type=int, nargs='?', action='store',
            default=5, help="Age of speaker providing audio to match. Default"
            + " 5.")
    parser.add_argument("-i", "--incoming_audio", type=str, action='store',
            nargs='?', dest="incoming_audio",
            help="Optional audio file to match.")
    parser.add_argument("-o", "--outfile", type=str, nargs='?', action='store',
            default="sample-out.wav", dest="out_file",
            help="Optional filename for morphed audio. Default sample-out.wav")
    parser.add_argument("-d", "--outdir", type=str, nargs='?', action='store',
            dest="out_dir", default="", help="Optional directory for saving "
            "audio. Default is the current working directory.")

    # Get arguments.
    args = parser.parse_args()
    print(args)

    # If no output directory was provided, default to the current working
    # directory.
    if not args.out_dir:
        args.out_dir = os.getcwd()

    # Set up audio entrainer.
    entrainer = EntrainAudio()

    # If an incoming audio file was provided, open it for processing.
    if args.incoming_audio:
            entrainer.entrain_from_file(args.incoming_audio,
                    args.audio_to_morph, args.out_file, args.out_dir,
                    args.use_praat, args.target_age)

    # Otherwise, open the microphone stream for processing. Only stop processing
    # from the mic when we get a keyboard interrupt and exit.
    else:
        entrainer.entrain_from_mic(args.audio_to_morph, args.out_file,
                args.out_dir, args.use_praat, args.target_age)


