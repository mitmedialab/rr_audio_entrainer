#! /usr/bin/env python
#
# Jacqueline Kory Westlund
# May 2017
#
# Use a Praat script to morph an audio file to match audio coming in over ROS.
# Detect various features of the incoming signal (e.g., pitch, speaking rate)
# and morph the outgoing sound to match.
#
# #############################################################################
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
# ROS and ROS msgs
import rospy
from r1d1_msgs.msg import AndroidAudio
from rr_msgs.msg import EntrainAudio
from rr_msgs.msg import InteractionState
from std_msgs.msg import String
from std_msgs.msg import Int32

class AudioEntrainer():
    """ Given an audio stream and an audio file, detect the pitch and tempo of
    the stream, then modify the pitch and tempo of the second to match.
    """

    def __init__(self, get_audio_over_ros):
        """ Initialize entrainer for detecting pitch and tempo. """
        # Where is Praat? Assumes it has been added to your PATH.
        self.praat = "praat"
        # Where is the Praat script we will use for processing?
        self.script = "rr_entrain_speech.praat"

        # Define pitch ranges that will be considered speech.
        # - Human child speech: generally 250-400Hz.
        # - Human adult male speech: generally 85-180Hz.
        # - Human adult female speech: generally 165-255Hz.
        self._floor_pitch = 100
        self._ceiling_pitch = 600

        # These are the settings that the ROS android microphone node uses for
        # getting audio. TODO we may want that node to send an "audio info
        # "message with this information, so that we don't have to hope we stay
        # in sync with its settings.
        if get_audio_over_ros:
            self._buffer_size = 2048
            self._samplerate = 16000
            self._n_channels = 1
            self.sample_size = 2
        # Otherwise, we record audio from a local microphone.
        else:
            self._buffer_size = 2048
            self._samplerate = 44100
            self._n_channels = 1

        # Set up to detect pitch.
        _tolerance = 0.8
        _win_s = 4096 # fft size
        _hop_s = self._buffer_size # hop size
        # Set up the aubio pitch detector.
        self.pitch_detector = aubio.pitch("default", _win_s, _hop_s,
                self._samplerate)
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


    def entrain_from_mic(self, source_file, out_file, out_dir, target_age):
        """ Open the microphone to get an incoming audio stream. Save the first
        section of audio that is probably speech to a wav file and use that as
        the target when processing with Praat.
        """
        # Initialize pyaudio.
        pyaud = pyaudio.PyAudio()

        # Open stream.
        pyaudio_format = pyaudio.paInt16
        self.sample_size = pyaud.get_sample_size(pyaudio_format)
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
                elif pitch < self._floor_pitch or pitch > self._ceiling_pitch:
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
                    # TODO file name for target? append date/time
                    # so we know later what was processed to get the
                    # morphed source?
                    target_file = "target-temp.wav"
                    self.save_to_wav(f, target_file)

                    # Use a Praat script to morph the source audio
                    # to match what's coming over the mic.
                    # TODO
                    self.entrain_from_file_praat(target_file, source_file,
                            out_file, out_dir, target_age)
                    break

            # Stop processing.
            except KeyboardInterrupt:
                print("*** Keyboard interrupt, exiting")
                break
        # Close stream and clean up.
        stream.stop_stream()
        stream.close()
        pyaud.terminate()


    def entrain_from_file_praat(self, target_file, source_file, out_file,
            out_dir, target_age):
        """ Use a Praat script to morph the given source audio file to match the
        specified target audio file, and save with the provided output file name
        to the specified output directory.
        """
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


    def save_to_wav(self, data, filename):
        """ Save the given audio data to a wav file. """
        target_file = filename
        wav_file = wave.open(target_file, 'wb')
        wav_file.setnchannels(self._n_channels)
        wav_file.setsampwidth(self.sample_size)
        wav_file.setframerate(self._samplerate)
        wav_file.writeframes(b''.join(data))
        wav_file.close()



def on_android_audio_msg(data):
    """ When we get an AndroidAudio message, collect the audio into an
    array for later processing, but only if it's the participant's turn to
    speak and the participant is speaking.
    """
    if data.is_streaming and is_speaking and is_participant_turn:
        global audio_data
        audio_data.append(data.data)


def on_speaking_binary_msg(data):
    """ When we get a speaking binary message, store whether or not someone
    is speaking, for later reference.
    """
    global is_speaking
    is_speaking = int(data.data)


def on_interaction_state_msg(data):
    """ When we get a message with information about the interaction state,
    such as whether it's the participant's turn to speak, store for later
    reference, and collect audio to entrain to.
    """
    # If it's the participant's turn to speak, we should collect audio, and
    # use what the participant says to entrain the robot's next speech.
    global is_participant_turn
    is_participant_turn = data.is_participant_turn


def on_entrain_audio_msg(data):
    """ When we get an message telling us to entrain audio, use the audio we've
    recently collected and the given age to morph the given audio file, and
    send that audio file to the robot.
    """
    if not args.use_mic:
        # Save audio collected so far to wav file.
        # TODO file name for target? Append participant ID, date, and time so
        # we know later what target was processed to get the morphed source.
        entrainer.save_to_wav(audio_data, "target-temp.wav")

        # Give the source wav file (that was given to us) and the target wav
        # file (that we collected) to Praat for processing.
        # TODO outfile and outdir?
        out_file = "sample-out.wav"
        entrainer.entrain_from_file_praat("target-temp.wav", data.audio,
                out_file, args.out_dir, data.age)
    else:
        # For now, collect some audio from the local mic and entrain to that.
        # TODO Use the speaking binary and interaction state to decide when
        # to collect audio from the local mic.
        out_file = "sample-out.wav"
        entrainer.entrain_from_mic(data.audio, out_file, args.out_dir, data.age)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description='''Given an audio stream over ROS, detect various audio
            features. Morph an audio file (specified via a ROS msg) to match
            those features. Only use audio collected during the participant's
            turn to speak, when the participant is speaking (both also
            specified via ROS msgs). Send the morphed file to a robot. Also
            save the morphed file to the specified output directory.
            ''')
    # The user provides an output directory where we can save audio files.
    parser.add_argument("-d", "--outdir", type=str, nargs='?', action='store',
            dest="out_dir", default="", help="Optional directory for saving "
            "audio. Default is the current working directory.")
    # The user can decide whether to get audio from ROS or a local microphone.
    parser.add_argument("-m", "--use-local-mic", action='store_true',
            dest="use_mic", default=False, help="Use a local microphone " +
            "instead of an audio stream over ROS. Default false.")

    # Get arguments.
    args = parser.parse_args()
    print(args)

    # Set up defaults.
    age = 5
    # If no output directory was provided, default to the current working
    # directory.
    if not args.out_dir:
        args.out_dir = os.getcwd()

    # Set up audio entrainer.
    entrainer = AudioEntrainer(args.use_mic)

    if not args.use_mic:
        # Set up a deque to hold incoming data, with a max length, so that when
        # it gets full, the oldest items are automatically discarded.
        audio_data = deque([], maxlen=600) #TODO what's a good size?

    # ROS node setup:
    # TODO If running on a network where DNS does not resolve local hostnames,
    # get the public IP address of this machine and export to the environment
    # variable $ROS_IP to set the public address of this node, so the user
    # doesn't have to remember to do this before starting the node.
    ros_node = rospy.init_node('rr_audio_entrainer', anonymous=False)

    # This node will send the morphed wav file to the robot using an rftp or
    # http stream, which can be used a source for the robot's mediaplayer.
    # It only needs to publish messages for logging purposes and to let the
    # teleop interface or state machine node (depending on whether this node is
    # used as part of a teleoperated or autonomous robot) about its status.
    # Set up rostopics we publish: log messages.
    pub_ae = rospy.Publisher('rr/audio_entrainer', String, queue_size = 10)

    # This node will listen for incoming audio, whether or not someone is
    # speaking, and messages from the teleop interface or state machine node
    # regarding whether it is the child's turn to speak or not and what the
    # name of the next audio file to morph is.
    # Subscribe to other ros nodes:
    #  - r1d1_msgs/AndroidAudio to get incoming audio stream from the robot's
    #    microphone (if we are not using a local microphone)
    #  - speaking binary, from the backchanneling module TODO
    #  - child turn, perhaps part of an overall interaction state, from teleop
    #    interface or state machine node TODO
    #  - entrainment message, which sends a string with the name of the audio
    #    to morph next and the age of the speaker

    # TODO fill in topic names for all of the below:
    if not args.use_mic:
        sub_audio = rospy.Subscriber('topic_name', AndroidAudio,
                on_android_audio_msg)
    sub_sb = rospy.Subscriber('msg_sb/raw', Int32, on_speaking_binary_msg)
    sub_state = rospy.Subscriber('/rr/state', InteractionState,
            on_interaction_state_msg)
    sub_entrain = rospy.Subscriber('/rr/entrain_audio', EntrainAudio,
            on_entrain_audio_msg)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down audio entrainment module"

