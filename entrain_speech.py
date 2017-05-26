#! /usr/bin/env python
#
# Jacqueline Kory Westlund
# May 2017
#
# Entrain an outgoing/target audio file to an incoming source. Detect pitch and
# energy of the incoming signal; morph the outgoing sound to follow the same
# pitch contour.
#
# Following the demo_pyaudio.py script from the aubio library...
# - Use pyaudio to get incoming sound on the microphone.
# - Use aubio to detect the pitch and get a pitch contour for the incoming sound.
# Then, use librosa to ...
#

import argparse
import librosa
import pyaudio
import numpy
import aubio # pitch detection
from collections import deque # queue for incoming audio

class EntrainAudio():
    """ Given an audio stream and an audio file, detect the pitch and tempo of
    the stream, then modify the pitch and tempo of the second to match.
    """

    def __init__(self):
        """ Initialize entrainer for detecting pitch and tempo. """
        self.buffer_size = 2048
        self.samplerate = 44100
        # Set up to detect pitch.
        tolerance = 0.8
        win_s = 4096 # fft size
        hop_s = self.buffer_size # hop size
        self.pitch_detector = aubio.pitch("default", win_s, hop_s, self.samplerate)
        self.pitch_detector.set_unit("Hz")
        self.pitch_detector.set_tolerance(tolerance)

    def entrain_from_mic(self, target_file):
        """ Open the microphone to get an incoming audio stream. """
        # Initialize pyaudio.
        pyaud = pyaudio.PyAudio()

        # Open stream.
        pyaudio_format = pyaudio.paFloat32
        n_channels = 1
        stream = pyaud.open(format=pyaudio_format,
                        channels=n_channels,
                        rate=self.samplerate,
                        input=True,
                        frames_per_buffer=self.buffer_size)
        # Process microphone stream until we get a keyboard interrupt.
        # Set up a deque to hold incoming data, with a max length, so that when
        # it gets full, the oldest items are automatically discarded.
        incoming = deque([], maxlen=200)

        # TODO: Listen on the microphone only when we know it's the child's turn
        # to speak (and process only the audio collected during their speech
        # turn). For now, assume that it's the child's turn when the program
        # starts, and that the robot's turn starts on a key press.
        #
        # NOTE: temporary. Listen to the keyboard for key presses to determine
        # when to start and end listening to the microphone.

        # Running total of potential speech frames.
        running_total_speech = 0
        running_total_silence = 0

        while True:
            try:
                # Detect pitch.
                audiobuffer = stream.read(self.buffer_size)
                signal = numpy.fromstring(audiobuffer, dtype=numpy.float32)
                pitch = self.pitch_detector(signal)[0]
                confidence = self.pitch_detector.get_confidence()
                print("{} / {}".format(pitch,confidence))

                # TODO For now, we just check the pitch to see if it is probably
                # part of human speech or not. If we get enough values in a row
                # that are probably speech, we use those as the source signal
                # for morphing the specified audio file. If we get a long pause,
                # however, we reset, and wait until we get sufficient speech in
                # a row to act as a source. This will probably have to change to
                # something more intelligent later.

                # If the incoming sound is within the range of human pitches,
                # it's probably speech. TODO check if over some threshold?
                # - Human child speech: generally 250-400Hz.
                # - Human adult male speech: generally 85-180Hz.
                # - Human adult female speech: generally 165-255Hz.
                if pitch > 40 and pitch < 600:
                    running_total_speech += 1
                    if running_total_speech > 4:
                        # Potential speech. Start tracking.
                        print("\tspeech?")
                        # Reset silence counter.
                        running_total_silence = 0

                        # Save running stream of pitches.
                        incoming.append(pitch)

                        if len(incoming) > 50:
                            # Stop listening on microphone for now. TODO
                            # Print contour, morph target audio file.
                            self.morph_pitch(incoming, target_file)
                            break

                # If the pitch is zero, there's probably no speech.
                elif pitch < 1:
                    running_total_silence +=1
                    # If we've had a lot of silence in a row, there's probably
                    # a pause.
                    if running_total_silence > 5:
                        print("\tsilence")
                        running_total_speech = 0
                    else:
                        # If it's just a short pause, we can keep it.
                        incoming.append(pitch)

            # Stop processing.
            except KeyboardInterrupt:
                print("*** Keyboard interrupt, exiting")
                break
        # Close stream and clean up.
        stream.stop_stream()
        stream.close()
        pyaud.terminate()


    def entrain_from_file(self, source_file, target_file):
        """ Open and read an audio file for processing. """
                # Detect tempo.
                # Detect pitch.
                # Read the audio file to morph.
                # Adjust tempo to match.
                # Adjust pitch to match.
                # Save adjusted audio to new file.


    def detect_tempo(self):
        print("TODO")
        # Use number of syllables per second as a measure of speech rate.
        # Detect syllable boundaries using acoustic-only features.
        # Or use mean syllable duration?

    def morph_tempo(self):
        print("TODO")


    def detect_pitches(self, target):
        """ Given an array of amplitude samples, detect pitches. """
        pitches = []
        print("Target length: {}".format(len(target)))
        for i in range(0,len(target)/2048):
            t = numpy.array(target[i*2048:i*2048+2048])
            pitch = self.pitch_detector(numpy.array(t))[0]
            confidence = self.pitch_detector.get_confidence()
            print("{} / {}".format(pitch,confidence))
            if pitch > 40 and pitch < 800:
                pitches.append(pitch)
        return pitches
        #TODO or potentially use librosa for this? https://github.com/tyrhus/pitch-detection-librosa-python/blob/master/script_final.py


    def morph_pitch(self, source, target_file):
        """ Morph the pitch of the target_file based on the pitch contour of the
            source.
            source: numpy array of incoming sound.
            target_file: .wav to morph.
        """
        print("TODO: adapt pitch")

        # Print out our source array.
        print(source)
        # Get the mean pitch of the source.
        mean_source_pitch = numpy.mean(source)
        print("Mean source pitch: {}".format(mean_source_pitch))

        # Use librosa to read in the target wav file as a numpy array.
        target = librosa.core.load(target_file)

        # Get target pitches.
        target_pitches = self.detect_pitches(target[0])
        print(target_pitches)

        # Get the mean pitch of the target file.
        mean_target_pitch = numpy.mean(target_pitches)
        print("Mean target pitch: {}".format(mean_target_pitch))

        # Find the difference between the source's mean pitch and the target's
        # mean pitch. This is still in Hz, but we need it in half-steps for the
        # pitch shifting function.
        diff_hz = mean_source_pitch - mean_target_pitch
        print("Difference between source and target: {}".format(diff_hz))

        # The basic formula for note frequencies is:
        # fn = f0 * a^n
        # where fn is the target frequency, f0 is the source frequency, a is
        # 2^(1/12) = 1.05946.., and n is the number of half steps between the
        # frequencies. Solving for n, we get the formula:
        # log(fn/f0) / log(a) = n
        diff_steps = numpy.log(mean_target_pitch / mean_source_pitch) / \
                numpy.log(1.05946)

        # Use librosa to shift the pitch of the target file to match the mean
        # pitch of the source.
        shifted = librosa.effects.pitch_shift(target[0], target[1], n_steps=diff_steps)
        # And write out to a file.
        librosa.output.write_wav("test.wav", shifted, target[1])



        # TODO: need to adapt to pitch contour, not just change pitch of entire
        # file. check lubold praat script?

        # then use delta-pitch (pitch derivative?), not absolute pitch



if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description='''Given an audio stream from the mic (default) or an audio
            file, detect the pitch and tempo. Modify the pitch and tempo of a
            specified audio file to match.
            ''')
    parser.add_argument("audio_to_morph", type=str, nargs=1, action='store',
            help="Audio file to morph.")
    parser.add_argument("-i", "--incoming_audio", type=str, nargs='+',
            dest="incoming_audio", help="Optional audio file to match.")

    # Get arguments.
    args = parser.parse_args()
    print(args)

    # Set up audio entrainer.
    entrainer = EntrainAudio()

    # If an incoming audio file was provided, open it for processing.
    if args.incoming_audio:
        entrainer.entrain_from_file(args.incoming_audio, args.audio_to_morph[0])
    # Otherwise, open the microphone stream for processing. Only stop processing
    # from the mic when we get a keyboard interrupt and exit.
    else:
        entrainer.entrain_from_mic(args.audio_to_morph[0])



