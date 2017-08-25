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
import os.path
import subprocess
import wave # For saving wav files
import time # For adding timestamps to audio filenames.
import struct
import scipy.io.wavfile # For reading wav files for energy processing.
# ROS and ROS msgs
import rospy
from r1d1_msgs.msg import AndroidAudio
from r1d1_msgs.msg import TegaAction
from r1d1_msgs.msg import Viseme
from rr_msgs.msg import EntrainAudio
from rr_msgs.msg import InteractionState
from std_msgs.msg import Int32
from std_msgs.msg import String
# For streaming audio to the robot.
import SimpleHTTPServer
import SocketServer

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
        # getting audio. These are defaults; we will update them after we get
        # the actual values from the audio messages.
        if get_audio_over_ros:
            self._buffer_size = 2048
            self._samplerate = 44100
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
        # to get the raw data, and another for the pitch values. The max
        # length is set such that about 30s of audio are kept (at 24 fps).
        frames = deque([], maxlen=720)
        f = []
        incoming_pitches = deque([], maxlen=720)

        # TODO: Listen on the microphone only when we know it's the child's turn
        # to speak (and process only the audio collected during their speech
        # turn).

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

                # For now, we just check the pitch to see if it is probably a
                # part of human speech or not. If we get enough values in a row
                # that are probably speech, we use those as the target signal
                # for morphing the specified audio file. If we get a long pause,
                # however, we reset, and wait until we get sufficient speech in
                # a row to act as a target. This will probably have to change to
                # something more intelligent later. For example, we could check
                # if the values are over some threshold for volume or energy.
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
                    if running_total_silence > 6 and running_total_speech < 20:
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
                if len(incoming_pitches) >= 20 and running_total_silence > 5:
                    # Append the current date and time to our output files so we
                    # don't overwrite previous output files.
                    t = time.strftime("%Y-%m-%d.%H:%M:%S")
                    target_file = "target" + t + ".wav"
                    self.save_to_wav(f, full_audio_out_dir + target_file)

                    # Use a Praat script to morph the source audio
                    # to match what's coming over the mic.
                    success = self.entrain_from_file_praat(full_audio_out_dir +
                            target_file, source_file, out_file, out_dir,
                            target_age)
                    break
            # Stop processing.
            except KeyboardInterrupt:
                print("*** Keyboard interrupt, exiting")
                break
        # Close stream and clean up.
        stream.stop_stream()
        stream.close()
        pyaud.terminate()
        return success


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
        try:
            subprocess.call([self.praat, "--run", self.script, source_file,
                target_file, out_dir + out_file, str(ff_age)])
            # If Praat successfully returned, it may have successfully morphed
            # the source audio file, but it may not have. Check that the file
            # exists before actually calling this a success.
            if os.path.isfile(out_dir + out_file):
                return True
            else:
                return False
        except Exception as e:
            print e
            print "Praat didn't work!"
            return False


    def save_to_wav(self, data, filename):
        """ Save the given audio data to a wav file. """
        print ("saving to wav: " + filename)
        wav_file = wave.open(filename, 'wb')
        wav_file.setnchannels(self._n_channels)
        wav_file.setsampwidth(self.sample_size)
        wav_file.setframerate(self._samplerate)
        if args.use_ros:
            while len(data) > 0:
                d = data.popleft()
                wav_file.writeframes(struct.pack("h"*len(d),*d))
        else:
            wav_file.writeframes(b''.join(data))
        wav_file.close()


    def process_visemes(self, original_audio, morphed_audio, morphed_dir,
            viseme_file):
        """ Because the audio entrainer may change the speaking rate of the
        audio file provided, we also should adjust the timestamps of the
        phonemes in the viseme file to match.
        """
        # Open the original audio file and the morphed file, get the durations,
        # and figure out how much to change the viseme file (if at all).
        print "Updating viseme file to match morphed audio..."
        diff = 0
        try:
            orig = wave.open(original_audio, 'r')
            orig_length = orig.getnframes() / (float)(orig.getframerate())
            morphed = wave.open(morphed_dir + morphed_audio, 'r')
            morphed_length = morphed.getnframes() / \
                    (float)(morphed.getframerate())
            diff = (morphed_length - orig_length) * 1000.0
            print "Source: {}, morphed: {}, diff: {}".format(orig_length,
                    morphed_length, diff)
        except Exception as e:
            print "Couldn't open file to check length - maybe it doesn't exist?"
            print e

        # Read in the viseme file for processing.
        lines = []
        try:
            vfile = open(viseme_file)
            for line in vfile:
                lines.append(line.strip().split(" "))
            vfile.close()
        except Exception as e:
            print "Could not read viseme file: " + viseme_file
            print e
            return []

        # The viseme files have a header line, so subtract it from the total.
        # Change each viseme time by a small portion of the total difference.
        change_by = 0
        if len(lines) > 1:
            change_by = diff / (len(lines) - 1)
            print "Change viseme lines by {}".format(change_by)
        vs = []
        for line in lines:
            print line
            if len(line) > 1:
                try:
                    v = Viseme(line[1], int(int(line[0]) + change_by))
                    vs.append(v)
                except Exception as e:
                    print "Could not change viseme line: %s" % line
                    print e
        # Return array of phoneme / viseme-time pairs.
        return vs


    def process_energy(self, audio_directory, audio_file):
        """ Compute the energy timeseries for the entrained audio file so we can
        send it to the robot. The robot uses this to bounce while speaking with
        an amount of energy reflecting the energy of the audio.
        """
        # Read in wav file.
        try:
            rate, data = scipy.io.wavfile.read(audio_directory + audio_file)
            if rate == 0 or not data.any():
                return [], []
        except Exception as e:
            print e
            return [], []
        # Get data type max value.
        max_value = numpy.iinfo(data.dtype).max
        # Make data an array of floats.
        data = data.astype(float)
        # If there are multiple channels, average across them to make it mono.
        if len(data.shape) > 1:
            data = data.sum(axis=1) / data.shape[1]
        # Convert to floats that are a percentage of the max possible value.
        scaled_data = [ v / max_value for v in data]

        # Divide into chunks: at 50Hz like r1d1_action audio energy processor
        # (i.e., 0.02 seconds per chunk). Use sample rate from audio file to
        # figure out how many samples per chunk.
        samples_per_chunk = int(rate * 0.02)

        # Split data into chunks.
        chunked_data = numpy.array_split(scaled_data, samples_per_chunk)
        # Get Hamming window for scaling energy values. Because numpy's
        # array_split does not guarantee that each of the chunks will be exactly
        # the same size (e.g., the last one might be shorter), we should create
        # a window for each one with the right size. But since many chunks will
        # be the same size, we can create a dictionary of Hamming windows of
        # different sizes, so we don't have to make so many.
        hamming = {}
        # Based on the Hamming window, in r1d1_action a window correction is
        # applied during the energy calculation. Again, we can compute it once
        # for each size chunk and re-use them.
        window_corrections = {}

        # List of energy values and times.
        energies = []
        times = []

        # The sample index is used to compute the time of each energy value.
        sample_index = 0.0

        # For each chunk:
        for c in chunked_data:
            # Apply a Hamming window for scaling the audio values.
            # Then, get the energy value for this chunk: the sum of squares of
            # the samples / number of samples * window correction.
            if c.size not in hamming:
                hamming[c.size] = numpy.hamming(c.size)
                window_corrections[c.size] = c.size / sum(hamming[c.size])
            energy = sum([v * v for v in (c * hamming[c.size])]) / (c.size *
                    window_corrections[c.size])

            # Save energy value and its time (sample index / samples per second).
            energies.append(energy)
            times.append(sample_index / rate)
            # Increment the sample index.
            sample_index += c.size

        # Return the energies and times.
        return energies, times


def on_android_audio_msg(data):
    """ When we get an AndroidAudio message, collect the audio into an
    array for later processing.
    """
    # Collect audio if it's the participant's turn to speak, and if the
    # participant is speaking. We arbitrarily wait for at least a few samples of
    # speech before collecting, and we collect silences/non-speech audio only
    # if they are sufficiently short. #TODO threshold for collecting?
    if is_participant_turn and data.is_streaming and is_speaking > 4:
        global audio_data
        audio_data.append(data.samples)
        global samplerate
        samplerate = data.sample_rate
        global n_channels
        n_channels = data.nchannels
        global sample_size
        sample_size = data.sample_width


def on_speaking_binary_msg(data):
    """ When we get a speaking binary message, store whether or not someone
    is speaking, for later reference.
    """
    if data.data:
        global is_speaking
        is_speaking += 1
        if is_speaking > 4: #TODO threshold?
            global not_speaking
            not_speaking = 0
    else:
        global not_speaking
        not_speaking += 1
        if not_speaking > 40: #TODO threshold?
            global is_speaking
            is_speaking = 0


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
    is_participant_turn = False
    visemes = []
    success = False
    # Append the current date and time to our output files so we don't
    # overwrite previous output files.
    t = time.strftime("%Y-%m-%d.%H:%M:%S")
    target = full_audio_out_dir + "target-" + t + ".wav"
    if args.use_ros:
        # Create outfile name.
        out_file = os.path.basename(data.audio.replace(".wav", "")) + t + ".wav"

        # If we have audio data to use as a target, and we are supposed to
        # entrain to it, then we should save it and entrain to it.
        if len(audio_data) > 0 and data.entrain:
            # Save audio collected so far to wav file.
            entrainer.save_to_wav(audio_data, target)
            # Reset audio data.
            global audio_data
            audio_data = deque([], maxlen=720)

            # Give the source wav file (that was given to us) and the target wav
            # file (that we collected) to Praat for processing.
            success = entrainer.entrain_from_file_praat(target, data.audio,
                    out_file, full_audio_out_dir, data.age)
        elif not data.entrain:
            print "We were told not to entrain. Skipping entrainment."
        else:
            print "We were supposed to entrain, but we had no audio data to " +\
                    "entrain to... skipping entrainment."

        # Adjust the viseme file times to match the morphed audio.
        visemes = entrainer.process_visemes(data.audio, out_file,
                full_audio_out_dir, data.viseme_file)

        # Get audio energy to send to the robot.
        if success:
            energies, times = entrainer.process_energy(
                full_audio_out_dir, out_file)
        else:
            energies, times = entrainer.process_energy("", data.audio)

    else:
        # For now, collect some audio from the local mic and entrain to that.
        # TODO Use the speaking binary and interaction state to decide when
        # to collect audio from the local mic.
        out_file = "out" + t + ".wav"
        success = entrainer.entrain_from_mic(data.audio, out_file,
                full_audio_out_dir, data.age)

    # After audio is entrained, stream to the robot.
    if success:
        send_tega_action_message(server + output_audio_dir + out_file, visemes,
                energies, times)
    else:
        send_tega_action_message(server + source_audio_dir +
                (os.path.basename(data.audio)), visemes, energies, times)


def send_tega_action_message(audio_file, visemes, energies, times):
    """ Publish TegaAction message to playback audio. """
    print '\nsending speech message: %s' % audio_file
    msg = TegaAction()
    msg.wav_filename = audio_file
    msg.visemes = visemes
    msg.energy_values = energies
    msg.energy_times = times
    pub_tega_action.publish(msg)
    # rospy.loginfo(msg)


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
    # The user has to provide the IP address of the machine running this node.
    parser.add_argument("-i", "--ipaddr", type=str, nargs=1, action='store',
            dest="ip_addr", default="192.168.1.254", help="The IP address of the"
            + " machine running this node. Used to serve audio files to the "
            + "robot.")
    # The user provides an output directory where we can save audio files.
    parser.add_argument("-d", "--audiodir", type=str, nargs=1, action='store',
            dest="base_audio_dir", required=True, help="Directory containing " +
            "subdirectory with the source audio (\"source\") and a directory " +
            "for saving output (\"output\").")
    # The user can decide whether to get audio from ROS or a local microphone.
    parser.add_argument("-r", "--use-ros", action='store', dest="use_ros",
            default=True, help="Use a local microphone or an audio stream " +
            "ROS from an Android mic (i.e., the robot). Default ROS.")

    # Get arguments.
    args = parser.parse_args()
    print(args)

    # Set up defaults.
    source_audio_dir = "source/"
    output_audio_dir = "output/"
    full_audio_out_dir = args.base_audio_dir[0] + output_audio_dir
    age = 5
    global is_participant_turn
    is_participant_turn = False
    global is_speaking
    is_speaking = 5 # TODO temporary for testing
    global not_speaking
    not_speaking = 0

    # Set up HTTP server to serve morphed wav files to the robot.
    # TODO this needs to be on its own thread, as it is blocking. For now, run
    # in a separate python shell.
    # Change directory to serve from the directory where we save audio output.
    #os.chdir(full_audio_out_dir)
    port = 8000
    #Handler = SimpleHTTPServer.SimpleHTTPRequestHandler
    #httpd = SocketServer.TCPServer(("", port), Handler)
    #print "Serving at port", port
    #httpd.serve_forver()

    # Stream audio to the robot from this address.
    server = "http://" + str(args.ip_addr[0]) + ":" + str(port) + "/"

    # Set up audio entrainer.
    entrainer = AudioEntrainer(args.use_ros)

    if args.use_ros:
        # Set up a deque to hold incoming data, with a max length, so that when
        # it gets full, the oldest items are automatically discarded. The max
        # length is set such that about 30s of audio are kept (at 24 fps).
        audio_data = deque([], maxlen=720)

    # ROS node setup:
    # TODO If running on a network where DNS does not resolve local hostnames,
    # get the public IP address of this machine and export to the environment
    # variable $ROS_IP to set the public address of this node, so the user
    # doesn't have to remember to do this before starting the node.
    ros_node = rospy.init_node('rr_audio_entrainer', anonymous=False)

    # This node will send the morphed wav file to the robot using an http
    # stream, which can be used a source for the robot's mediaplayer. Thus, it
    # publishes TegaAction messages to the robot with the http address of each
    # audio file to stream. It may also publish publish messages for logging
    # purposes and to let the teleop interface or state machine node (depending
    # on whether this node is used as part of a teleoperated or autonomous
    # robot) about its status.
    # Set up rostopics we publish: log messages, TegaAction.
    pub_ae = rospy.Publisher('rr/audio_entrainer', String, queue_size = 10)
    pub_tega_action = rospy.Publisher('tega', TegaAction, queue_size = 10)

    # This node will listen for incoming audio, whether or not someone is
    # speaking, and messages from the teleop interface or state machine node
    # regarding whether it is the child's turn to speak or not and what the
    # name of the next audio file to morph is. Subscribe to other ros nodes:
    if args.use_ros:
        # If we are not using the local microphone, we are using ROS...
        # Speaking binary, from the backchanneling module.
        sub_sb = rospy.Subscriber('msg_sb/raw', Int32, on_speaking_binary_msg)
        # Use r1d1_msgs/AndroidAudio to get incoming audio stream from the
        # robot's microphone or a standalone android microphone app.
        sub_audio = rospy.Subscriber('android_audio', AndroidAudio,
                on_android_audio_msg)

    #  Child turn, perhaps part of an overall interaction state, from teleop
    #  interface or state machine node.
    sub_state = rospy.Subscriber('/rr/state', InteractionState,
            on_interaction_state_msg)
    #  Entrainment message, which sends a string with the name of the audio to
    #  morph next and the age of the speaker.
    sub_entrain = rospy.Subscriber('/rr/entrain_audio', EntrainAudio,
            on_entrain_audio_msg)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down audio entrainment module"

