#! /usr/bin/env python
"""
Jacqueline Kory Westlund
May 2017

Use a Praat script to morph an audio file to match audio coming in over ROS.
Detect various features of the incoming signal (e.g., pitch, speaking rate)
and morph the outgoing sound to match.

#############################################################################

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
details.

You should have received a copy of the GNU General Public License along with
this program.  If not, see http://www.gnu.org/licenses/
"""

import argparse
import pyaudio  # (MIT license)
import numpy
import aubio  # pitch detection (GNU/GPL license)
from collections import deque  # queue for incoming audio
import logging  # Log messages.
import os
import os.path
import subprocess
import wave  # For saving wav files
import time  # For adding timestamps to audio filenames.
import struct
import scipy.io.wavfile  # For reading wav files for energy processing.
# ROS and ROS msgs
import rospy
from r1d1_msgs.msg import AndroidAudio
from r1d1_msgs.msg import TegaAction
from r1d1_msgs.msg import Viseme
from rr_msgs.msg import EntrainAudio
from rr_msgs.msg import InteractionState
from rr_msgs.msg import EntrainmentData
from std_msgs.msg import Header  # Standard ROS msg header.
from std_msgs.msg import String
# TODO For streaming audio to the robot.
# import SimpleHTTPServer
# import SocketServer


class AudioEntrainer(object):
    """ Given an audio stream and an audio file, detect the pitch and tempo of
    the stream, then modify the pitch and tempo of the second to match.
    """

    def __init__(self, get_audio_over_ros):
        """ Initialize entrainer for detecting pitch and tempo. """
        # Where is Praat? Assumes it has been added to your PATH.
        self.praat = "praat"
        # Where is the Praat script we will use for processing?
        self.script = "rr_entrain_speech.praat"

        # These are the settings that the ROS android microphone node uses for
        # getting audio. These are defaults; we will update them after we get
        # the actual values from the audio messages.
        if get_audio_over_ros:
            self._buffer_size = 2048
            self.samplerate = 44100
            self.n_channels = 1
            self.sample_size = 2
        # Otherwise, we record audio from a local microphone.
        else:
            self._buffer_size = 2048
            self.samplerate = 44100
            self.n_channels = 1

        # Set up the aubio pitch detector.
        self.pitch_detector = aubio.pitch(
            "default",
            4096,  # FFT size
            self._buffer_size,  # hop size
            self.samplerate)
        self.pitch_detector.set_unit("Hz")
        self.pitch_detector.set_tolerance(0.8)

    @staticmethod
    def _get_mean_ff(age):
        """ Return the mean fundamental frequency for a given age group.
        """
        # pylint: disable=too-many-return-statements
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
        # pylint: disable=too-many-locals
        # Initialize pyaudio.
        pyaud = pyaudio.PyAudio()

        # Define pitch ranges that will be considered speech.
        # - Human child speech: generally 250-400Hz.
        # - Human adult male speech: generally 85-180Hz.
        # - Human adult female speech: generally 165-255Hz.
        floor_pitch = 100
        ceiling_pitch = 600

        # Open stream.
        pyaudio_format = pyaudio.paInt16
        self.sample_size = pyaud.get_sample_size(pyaudio_format)
        stream = pyaud.open(
            format=pyaudio_format,
            channels=self.n_channels,
            rate=self.samplerate,
            input=True,
            frames_per_buffer=self._buffer_size)

        # Process microphone stream until we get a keyboard interrupt.
        # Set up a deque to hold incoming data, with a max length, so that when
        # it gets full, the oldest items are automatically discarded. Use one
        # to get the raw data, and another for the pitch values. The max
        # length is set such that about 30s of audio are kept (at 24 fps).
        frames = deque([], maxlen=720)
        fra = []
        incoming_pitches = deque([], maxlen=720)

        # TODO: Listen on the microphone only when it's the child's turn
        # to speak (and process only the audio collected during their speech
        # turn).

        # Running total of potential speech frames.
        running_total_speech = 0
        running_total_silence = 0

        while True:
            try:
                # Detect pitch.
                audiobuffer = stream.read(self._buffer_size)
                signal = numpy.fromstring(
                    audiobuffer,
                    dtype=numpy.int16).astype(numpy.float32)
                pitch = self.pitch_detector(signal)[0]
                confidence = self.pitch_detector.get_confidence()
                LOGGER.info("{} / {}".format(pitch, confidence))

                # For now, we just check the pitch to see if it is probably a
                # part of human speech or not. If we get enough values in a row
                # that are probably speech, we use those as the target signal
                # for morphing the specified audio file. If we get a long
                # pause, however, we reset, and wait until we get sufficient
                # speech in a row to act as a target. This will probably have
                # to change to something more intelligent later. For example,
                # we could check if the values are over some threshold for
                # volume or energy.
                if pitch > floor_pitch and pitch < ceiling_pitch:
                    running_total_speech += 1
                    if running_total_speech > 4:
                        # Potential speech. Start tracking.
                        LOGGER.debug("\tspeech?")
                        # Reset silence counter.
                        running_total_silence = 0
                        # Save running stream of pitches.
                        incoming_pitches.append(pitch)
                # If the pitch is zero, there's probably no speech.
                elif pitch < floor_pitch or pitch > ceiling_pitch:
                    running_total_silence += 1
                    # If we've had a lot of silence in a row, or non-speech
                    # sound, there's probably a pause or no speech.
                    # TODO Pick good values for these:
                    if running_total_silence > 6 and running_total_speech < 20:
                        LOGGER.debug("\tsilence")
                        running_total_speech = 0
                    else:
                        # If it's just a short pause, we can keep it.
                        incoming_pitches.append(pitch)

                # Save running stream of raw audio.
                # TODO see if we can get deque to work
                if running_total_speech > 4:
                    frames.append(audiobuffer)
                    fra.append(audiobuffer)

                # Check to see if we need to process the incoming audio yet.
                # If we have sufficient incoming audio and there's been a long
                # silence, stop listening on the mic and process.
                if len(incoming_pitches) >= 20 and running_total_silence > 5:
                    # Append the current date and time to our output files so
                    # we don't overwrite previous output files.
                    d_t = time.strftime("%Y-%m-%d.%H:%M:%S")
                    target_file = "target" + d_t + ".wav"
                    self.save_to_wav(fra, FULL_AUDIO_OUT_DIR + target_file)

                    # Use a Praat script to morph the source audio
                    # to match what's coming over the mic.
                    success, data = self.entrain_from_file_praat(
                        FULL_AUDIO_OUT_DIR + target_file,
                        source_file,
                        out_file,
                        out_dir,
                        target_age)
                    break
            # Stop processing.
            except KeyboardInterrupt:
                LOGGER.info("*** Keyboard interrupt, exiting")
                break
        # Close stream and clean up.
        stream.stop_stream()
        stream.close()
        pyaud.terminate()
        return success, data

    # pylint: disable=too-many-arguments
    def entrain_from_file_praat(self, target_file, source_file, out_file,
                                out_dir, target_age):
        """ Use a Praat script to morph the given source audio file to match
        the specified target audio file, and save with the provided output file
        name to the specified output directory.
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
            proc = subprocess.Popen(
                [self.praat,
                    "--run",
                    self.script,
                    source_file,
                    target_file,
                    out_dir + out_file,
                    str(ff_age)],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE)
            # Get the output from Praat so we can record the data regarding the
            # speaking rate, pitch, etc.
            stdout, stderr = proc.communicate()
            data = self.parse_praat_output(stdout + stderr)

            # If Praat successfully returned, it may have successfully morphed
            # the source audio file, but it may not have. Check that the file
            # exists before actually calling this a success.
            if os.path.isfile(out_dir + out_file):
                return True, data
            else:
                return False, data
        except Exception as ex:
            LOGGER.warning("Praat didn't work! {}".format(ex))
            return False, None

    def parse_praat_output(self, output):
        """ Parse the output that was printed by Praat to stdout and stderr in
        order to get relevant data about the source file and the target it was
        entrained to.
        """
        LOGGER.info("Parsing Praat output...")
        data = {}
        output_lines = output.split("\n")
        for line in output_lines:
            # Relevant data output lines are tagged with "**".
            if "**Target file:" in line:
                data["target_file"] = line.split(": ")[1]
            elif "**Source file:" in line:
                data["source_file"] = line.split(": ")[1]
            elif "**Output file:" in line:
                data["output_file"] = line.split(": ")[1]
            elif "**" in line:
                try:
                    value = float(line.split(": ")[1])
                    if "**Target mean intensity: " in line:
                        data["target_mean_intensity"] = value
                    if "**Source mean intensity: " in line:
                        data["source_mean_intensity"] = value
                    if "**Target speaking rate: " in line:
                        data["target_speech_rate"] = value
                    if "**Source speaking rate: " in line:
                        data["source_speech_rate"] = value
                    if "**Source original duration: " in line:
                        data["source_dur"] = value
                    if "**Duration morph factor: " in line:
                        data["dur_factor"] = value
                    if "**Adjusted duration morph factor: " in line:
                        data["adjusted_dur_factor"] = value
                    if "**Morphed duration: " in line:
                        data["morphed_duration"] = value
                    if "**Source mean pitch: " in line:
                        data["source_mean_pitch"] = value
                    if "**Target mean pitch: " in line:
                        data["target_mean_pitch"] = value
                    if "**Adjust pitch by: " in line:
                        data["adjust_pitch_by"] = value
                except ValueError as valerr:
                    LOGGER.warning("Could not get value! {}".format(valerr))
        return data

    def save_to_wav(self, data, filename):
        """ Save the given audio data to a wav file. """
        LOGGER.info("saving to wav: {}".format(filename))
        wav_file = wave.open(filename, 'wb')
        wav_file.setnchannels(self.n_channels)
        wav_file.setsampwidth(self.sample_size)
        wav_file.setframerate(self.samplerate)
        if ARGS.use_ros:
            while len(data) > 0:
                _da = data.popleft()
                wav_file.writeframes(struct.pack("h"*len(_da), *_da))
        else:
            wav_file.writeframes(b''.join(data))
        wav_file.close()

    def process_visemes(self, original_audio, morphed_audio, morphed_dir,
                        viseme_file):
        """ Because the audio entrainer may change the speaking rate of the
        audio file provided, we also should adjust the timestamps of the
        phonemes in the viseme file to match.
        """
        # pylint: disable=too-many-locals
        # Open the original audio file and the morphed file, get the durations,
        # and figure out how much to change the viseme file (if at all).
        LOGGER.info("Updating viseme file to match morphed audio...")
        diff = 0
        try:
            orig = wave.open(original_audio, 'r')
            orig_length = orig.getnframes() / (float)(orig.getframerate())
            morphed = wave.open(morphed_dir + morphed_audio, 'r')
            morphed_length = morphed.getnframes() / \
                (float)(morphed.getframerate())
            diff = (morphed_length - orig_length) * 1000.0
            LOGGER.info("Source: {}, morphed: {}, diff: {}".format(
                orig_length,
                morphed_length,
                diff))
        except Exception as ex:
            LOGGER.warning("Couldn't open file to check length. Maybe it "
                           "doesn't exist? {}".format(ex))

        # Read in the viseme file for processing.
        lines = []
        try:
            vfile = open(viseme_file)
            for line in vfile:
                lines.append(line.strip().split(" "))
            vfile.close()
        except Exception as ex:
            LOGGER.warning("Could not read viseme file: {} {}".format(
                viseme_file, ex))
            return []

        # The viseme files have a header line, so subtract it from the total.
        # Change each viseme time by a small portion of the total difference.
        change_by = 0
        if len(lines) > 1:
            change_by = diff / (len(lines) - 1)
            LOGGER.info("Change viseme lines by {}".format(change_by))
        _vs = []
        for line in lines:
            LOGGER.debug(line)
            if len(line) > 1:
                try:
                    _vi = Viseme(line[1], int(int(line[0]) + change_by))
                    _vs.append(_vi)
                except Exception as ex:
                    LOGGER.warning("Couldn't change viseme line: {} {}".format(
                        line, ex))
        # Return array of phoneme / viseme-time pairs.
        return _vs

    def process_energy(self, audio_directory, audio_file):
        """ Compute the energy timeseries for the entrained audio file so we
        can send it to the robot. The robot uses this to bounce while speaking
        with an amount of energy reflecting the energy of the audio.
        """
        # pylint: disable=too-many-locals
        # Read in wav file.
        try:
            rate, data = scipy.io.wavfile.read(audio_directory + audio_file)
            if rate == 0 or not data.any():
                return [], []
        except Exception as ex:
            LOGGER.warning(ex)
            return [], []
        # Get data type max value.
        max_value = numpy.iinfo(data.dtype).max
        # Make data an array of floats.
        data = data.astype(float)
        # If there are multiple channels, average across them to make it mono.
        if len(data.shape) > 1:
            data = data.sum(axis=1) / data.shape[1]
        # Convert to floats that are a percentage of the max possible value.
        scaled_data = [v / max_value for v in data]

        # Divide into chunks: at 50Hz like r1d1_action audio energy processor
        # (i.e., 0.02 seconds per chunk). Use sample rate from audio file to
        # figure out how many samples per chunk.
        samples_per_chunk = int(rate * 0.02)

        # Split data into chunks.
        chunked_data = numpy.array_split(scaled_data, samples_per_chunk)
        # Get Hamming window for scaling energy values. Because numpy's
        # array_split does not guarantee that each of the chunks will be
        # exactly the same size (e.g., the last one might be shorter), we
        # should create a window for each one with the right size. But since
        # many chunks will be the same size, we can create a dictionary of
        # Hamming windows of different sizes, so we don't have to make so many.
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
        for chunk in chunked_data:
            # Apply a Hamming window for scaling the audio values.
            # Then, get the energy value for this chunk: the sum of squares of
            # the samples / number of samples * window correction.
            if chunk.size not in hamming:
                hamming[chunk.size] = numpy.hamming(chunk.size)
                window_corrections[chunk.size] = chunk.size / \
                    sum(hamming[chunk.size])
            energy = sum([v * v for v in chunk * hamming[chunk.size]]) / \
                (chunk.size * window_corrections[chunk.size])

            # Save energy value and its corresponding time
            # (sample index / samples per second).
            energies.append(energy)
            times.append(sample_index / rate)
            # Increment the sample index.
            sample_index += chunk.size

        # Return the energies and times.
        return energies, times


def on_android_audio_msg(data):
    """ When we get an AndroidAudio message, collect the audio into an
    array for later processing.
    """
    # Collect audio if it's the participant's turn to speak, and if the
    # participant is speaking. We arbitrarily wait for at least a few samples
    # of speech before collecting, and we collect silences/non-speech audio
    # only if they are sufficiently short. #TODO threshold for collecting?
    if IS_PARTICIPANT_TURN and data.is_streaming:
        AUDIO_DATA.append(data.samples)
        ENTRAINER.samplerate = data.sample_rate
        ENTRAINER.n_channels = data.nchannels
        ENTRAINER.sample_size = data.sample_width


def on_interaction_state_msg(data):
    """ When we get a message with information about the interaction state,
    such as whether it's the participant's turn to speak, store for later
    reference, and collect audio to entrain to.
    """
    # If it's the participant's turn to speak, we should collect audio, and
    # use what the participant says to entrain the robot's next speech.
    global IS_PARTICIPANT_TURN
    IS_PARTICIPANT_TURN = data.is_participant_turn


def on_entrain_audio_msg(data):
    """ When we get an message telling us to entrain audio, use the audio we've
    recently collected and the given age to morph the given audio file, and
    send that audio file to the robot.
    """
    global IS_PARTICIPANT_TURN
    IS_PARTICIPANT_TURN = False
    visemes = []
    success = False
    entrain_data = {}
    # Append the current date and time to our output files so we don't
    # overwrite previous output files.
    tim = time.strftime("%Y-%m-%d.%H:%M:%S")
    target = FULL_AUDIO_OUT_DIR + "target-" + tim + ".wav"
    if ARGS.use_ros:
        # Create outfile name.
        out_file = os.path.basename(data.audio.replace(".wav", "")) + tim + \
            ".wav"

        # If we have audio data to use as a target, then we should save it and
        # entrain to it. Even if we are told not to entrain, we do this anyway
        # so that the latency for entrainment vs. not is the same -- i.e., we
        # do all the same computation, but we will stream the not-entrained
        # version of the source file later.
        if len(AUDIO_DATA) > 0:
            # Save audio collected so far to wav file.
            ENTRAINER.save_to_wav(AUDIO_DATA, target)
            # Reset audio data.
            global AUDIO_DATA
            AUDIO_DATA = deque([], maxlen=720)

            # Give the source wav file (that was given to us) and the target
            # wav file (that we collected) to Praat for processing. Get back
            # whether it worked, and if so, the entrainment data, such as the
            # speaking rate and pitch of the source and target.
            success, entrain_data = ENTRAINER.entrain_from_file_praat(
                target,
                data.audio,
                out_file,
                FULL_AUDIO_OUT_DIR,
                data.age)
        else:
            LOGGER.info("No audio data available for doing entrainment!")

        # Adjust the viseme file times to match the morphed audio.
        visemes = ENTRAINER.process_visemes(
            data.audio,
            out_file,
            FULL_AUDIO_OUT_DIR,
            data.viseme_file)

        # Get audio energy to send to the robot.
        if success:
            energies, times = ENTRAINER.process_energy(
                FULL_AUDIO_OUT_DIR,
                out_file)
        else:
            energies, times = ENTRAINER.process_energy("", data.audio)

    # If we are not using ROS, collect from the local mic and entrain to that.
    else:
        # TODO Use the speaking binary and interaction state to decide when
        # to collect audio from the local mic.
        out_file = "out" + tim + ".wav"
        success, entrain_data = ENTRAINER.entrain_from_mic(
            data.audio,
            out_file,
            FULL_AUDIO_OUT_DIR,
            data.age)

    # After audio is entrained, send to the robot. Stream the morphed output
    # if it worked and if we are supposed to entrain; otherwise, stream the
    # original source file.
    if success and data.entrain:
        LOGGER.info("Sending entrained audio file to robot...")
        send_tega_action_message(
            SERVER + OUTPUT_AUDIO_DIR + out_file,
            visemes,
            energies,
            times)
    else:
        LOGGER.info("We were either told not to entrain, or could not entrain,"
                    " so we are sending the not-entrained source file.")
        send_tega_action_message(
            SERVER + SOURCE_AUDIO_DIR + (os.path.basename(data.audio)),
            visemes,
            energies,
            times)
    # Also send the entrainment data we collected. Since we computed this data
    # regardless, we have it whether or not the morphed/entrained audio was
    # sent.
    send_entrainment_data_message(entrain_data)


def send_tega_action_message(audio_file, visemes, energies, times):
    """ Publish TegaAction message to playback audio. """
    LOGGER.info("\nsending speech message: {}".format(audio_file))
    msg = TegaAction()
    # Add header.
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.wav_filename = audio_file
    msg.visemes = visemes
    msg.energy_values = energies
    msg.energy_times = times
    PUB_TEGA_ACTION.publish(msg)


def send_entrainment_data_message(data):
    """ Publish an EntrainmentData message containing the latest entrainment
    information from the latest two files entrained (source and target), such
    as speaking rate and pitch.
    """
    # Build message.
    msg = EntrainmentData()
    # Add header.
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    # Fill in data.
    if "source_file" in data:
        msg.source_audio = data["source_file"]
    if "target_file" in data:
        msg.target_audio = data["target_file"]
    if "output_file" in data:
        msg.output_audio = data["output_file"]
    if "target_mean_intensity" in data:
        msg.target_mean_intensity = data["target_mean_intensity"]
    if "source_mean_intensity" in data:
        msg.source_mean_intensity = data["source_mean_intensity"]
    if "target_speech_rate" in data:
        msg.target_speaking_rate = data["target_speech_rate"]
    if "source_speech_rate" in data:
        msg.source_speaking_rate = data["source_speech_rate"]
    if "source_dur" in data:
        msg.source_original_duration = data["source_dur"]
    if "dur_factor" in data:
        msg.duration_morph_factor = data["dur_factor"]
    if "adjusted_dur_factor" in data:
        msg.adjusted_duration_morph_factor = data["adjusted_dur_factor"]
    if "morphed_duration" in data:
        msg.source_morphed_duration = data["morphed_duration"]
    if "source_mean_pitch" in data:
        msg.source_mean_pitch = data["source_mean_pitch"]
    if "target_mean_pitch" in data:
        msg.target_mean_pitch = data["target_mean_pitch"]
    if "adjust_pitch_by" in data:
        msg.adjust_pitch_by = data["adjust_pitch_by"]
    # Send.
    PUB_DATA.publish(msg)


if __name__ == '__main__':
    # Set up logger.
    LOGGER = logging.getLogger(__name__)
    LOGGER.info("Initializing audio entrainer...")

    PARSER = argparse.ArgumentParser(
        description="""Given an audio stream over ROS, detect various audio
            features. Morph an audio file (specified via a ROS msg) to match
            those features. Only use audio collected during the participant's
            turn to speak, when the participant is speaking (both also
            specified via ROS msgs). Send the morphed file to a robot. Also
            save the morphed file to the specified output directory.
            """)
    # The user has to provide the IP address of the machine running this node.
    PARSER.add_argument("-i", "--ipaddr", type=str, nargs=1, action='store',
                        dest="ip_addr",
                        default="192.168.1.254",
                        help="""The IP address of the machine running this
                        node. Used to serve audio files to the robot.""")
    # The user provides an output directory where we can save audio files.
    PARSER.add_argument("-d", "--audiodir", type=str, nargs=1, action='store',
                        dest="base_audio_dir",
                        required=True,
                        help="""Directory containing subdirectory with the
                        source audio (\"source\") and a directory for saving
                        output (\"output\").""")
    # The user can decide whether to get audio from ROS or a local microphone.
    PARSER.add_argument("-r", "--use-ros", action='store_true', dest="use_ros",
                        default=True,
                        help="""Use a local microphone or an audio stream ROS
                        from an Android mic (i.e., the robot). Default ROS.""")

    # Get arguments.
    ARGS = PARSER.parse_args()
    LOGGER.debug("Got args: {}".format(ARGS))

    # Set up defaults.
    SOURCE_AUDIO_DIR = "source/"
    OUTPUT_AUDIO_DIR = "output/"
    FULL_AUDIO_OUT_DIR = ARGS.base_audio_dir[0] + OUTPUT_AUDIO_DIR
    IS_PARTICIPANT_TURN = False

    # Set up HTTP server to serve morphed wav files to the robot.
    # TODO this needs to be on its own thread, as it is blocking. For now, run
    # in a separate python shell.
    # Change directory to serve from the directory where we save audio output.
    # os.chdir(FULL_AUDIO_OUT_DIR)
    PORT = 8000
    # Handler = SimpleHTTPServer.SimpleHTTPRequestHandler
    # httpd = SocketServer.TCPServer(("", port), Handler)
    # LOGGER.info("Serving at port {}".format(port))
    # httpd.serve_forver()
    # Stream audio to the robot from this address.
    SERVER = "http://" + str(ARGS.ip_addr[0]) + ":" + str(PORT) + "/"

    # Set up audio entrainer.
    ENTRAINER = AudioEntrainer(ARGS.use_ros)

    if ARGS.use_ros:
        # Set up a deque to hold incoming data, with a max length, so that when
        # it gets full, the oldest items are automatically discarded. The max
        # length is set such that about 30s of audio are kept (at 24 fps).
        AUDIO_DATA = deque([], maxlen=720)

    # ROS node setup:
    # TODO If running on a network where DNS does not resolve local hostnames,
    # get the public IP address of this machine and export to the environment
    # variable $ROS_IP to set the public address of this node, so the user
    # doesn't have to remember to do this before starting the node.
    ROS_NODE = rospy.init_node('rr_audio_entrainer', anonymous=False)

    # This node will send the morphed wav file to the robot using an http
    # stream, which can be used a source for the robot's mediaplayer. Thus, it
    # publishes TegaAction messages to the robot with the http address of each
    # audio file to stream. It may also publish publish messages for logging
    # purposes and to let the teleop interface or state machine node (depending
    # on whether this node is used as part of a teleoperated or autonomous
    # robot) about its status.
    # Set up rostopics we publish: log messages, TegaAction.
    PUB_AE = rospy.Publisher('rr/audio_entrainer', String, queue_size=10)
    PUB_TEGA_ACTION = rospy.Publisher('tega', TegaAction, queue_size=10)

    # This node will also publish entrainment results with data about the audio
    # files' speaking rate, pitch, intensity, etc in case anyone cares.
    PUB_DATA = rospy.Publisher('rr/entrainment_data', EntrainmentData,
                               queue_size=10)

    # This node will listen for incoming audio, whether or not someone is
    # speaking, and messages from the teleop interface or state machine node
    # regarding whether it is the child's turn to speak or not and what the
    # name of the next audio file to morph is. Subscribe to other ros nodes:
    if ARGS.use_ros:
        # If we are not using the local microphone, we are using ROS...
        # Use r1d1_msgs/AndroidAudio to get incoming audio stream from the
        # robot's microphone or a standalone android microphone app.
        SUB_AUDIO = rospy.Subscriber(
            'android_audio',
            AndroidAudio,
            on_android_audio_msg)

    #  Child turn, perhaps part of an overall interaction state, from teleop
    #  interface or state machine node.
    SUB_STATE = rospy.Subscriber(
        '/rr/state',
        InteractionState,
        on_interaction_state_msg)
    #  Entrainment message, which sends a string with the name of the audio to
    #  morph next and the age of the speaker.
    SUB_ENTRAIN = rospy.Subscriber(
        '/rr/entrain_audio',
        EntrainAudio,
        on_entrain_audio_msg)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        LOGGER.info("Shutting down audio entrainment module")
