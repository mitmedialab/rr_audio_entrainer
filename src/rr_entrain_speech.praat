###########################################################################
# Audio entrainment with Praat
# Jacqueline Kory Westlund
# June 2017
#
# Uses the Praat Script Syllable Nuclei from de Jong and Wempe (information
# below) for speaking rate detection. Their script has been moved into a
# procedure and minimally modified.
#
# Note that some of the Praat functions used in this script will only work
# with mono-channeled audio and will return an error on stereo.
# TODO Modify so Praat checks/converts to mono first.
#
###########################################################################
#                                                                         #
#  Praat Script Syllable Nuclei                                           #
#  Copyright (C) 2008  Nivja de Jong and Ton Wempe                        #
#                                                                         #
#    This program is free software: you can redistribute it and/or modify #
#    it under the terms of the GNU General Public License as published by #
#    the Free Software Foundation, either version 3 of the License, or    #
#    (at your option) any later version.                                  #
#                                                                         #
#    This program is distributed in the hope that it will be useful,      #
#    but WITHOUT ANY WARRANTY; without even the implied warranty of       #
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the        #
#    GNU General Public License for more details.                         #
#                                                                         #
#    You should have received a copy of the GNU General Public License    #
#    along with this program.  If not, see http://www.gnu.org/licenses/   #
#                                                                         #
###########################################################################
# modified 2010.09.17 by Hugo Quené, Ingrid Persoon, & Nivja de Jong
# Overview of changes:
# + change threshold-calculator: rather than using median, use the almost maximum
#     minus 25dB. (25 dB is in line with the standard setting to detect silence
#     in the "To TextGrid (silences)" function.
#     Almost maximum (.99 quantile) is used rather than maximum to avoid using
#     irrelevant non-speech sound-bursts.
# + add silence-information to calculate articulation rate and ASD (average syllable
#     duration.
#     NB: speech rate = number of syllables / total time
#         articulation rate = number of syllables / phonation time
# + remove max number of syllable nuclei
# + refer to objects by unique identifier, not by name
# + keep track of all created intermediate objects, select these explicitly,
#     then Remove
# + provide summary output in Info window
# + do not save TextGrid-file but leave it in Object-window for inspection
#     (if requested in startup-form)
# + allow Sound to have starting time different from zero
#      for Sound objects created with Extract (preserve times)
# + programming of checking loop for mindip adjusted
#      in the orig version, precedingtime was not modified if the peak was rejected !!
#      var precedingtime and precedingint renamed to currenttime and currentint
#
# + bug fixed concerning summing total pause, feb 28th 2011
###########################################################################

# The inputs are two wav files, the source and the target, one output file name,
# and an output directory.
# The source file will be morphed to match the target. The morphed sound will be
# saved to the output file name in the output directory.
form Counting Syllables in Sound Utterances
    sentence source_file sample.wav
    sentence target_file sample2.wav
    sentence output_file sample-morphed.wav
    real age_mean_pitch 290
endform

# Got files:
printline Target file: 'target_file$'
printline Source file: 'source_file$'
printline Output file: 'output_file$'

# Pitch range to consider (children generally 200-400 Hz).
floor = 100
ceiling = 500

# Read target file and detect features.
if fileReadable(target_file$)
    # Detect speech rate.
    printline Detecting target speech rate...
    @detectSpeechRate: target_file$
    target_speech_rate = speakingrate
    target_sound_id = soundid
    target_textgrid_id = textgridid

    # Get mean pitch of target.
    select 'target_sound_id'
    To Pitch... 0 floor ceiling
    target_mean_pitch = Get mean... 0 0 Hertz
else
    printline Cannot read target file
    exit
endif

# Read source file and detect features.
if fileReadable(source_file$)
    # Detect speech rate.
    printline Detecting source speech rate...
    @detectSpeechRate: source_file$
    source_speech_rate = speakingrate
    source_dur = originaldur
    source_sound_id = soundid
    source_textgrid_id = textgridid

    # Get mean pitch of source.
    select 'source_sound_id'
    To Pitch... 0 floor ceiling
    source_mean_pitch = Get mean... 0 0 Hertz
else
    printline Cannot read source file
    exit
endif

# Given the target speech rate, morph source file to match.
# Get the factor by which to adjust (i.e., speed up or slow down) the source.
printline Morphing source to match target...
if target_speech_rate <> 0
    dur_factor = 'source_speech_rate'/'target_speech_rate'
else
    dur_factor = 1
endif

printline Target rate: 'target_speech_rate:2'
printline Source rate: 'source_speech_rate:2'
printline Source original duration: 'source_dur:2'
printline Duration morph factor: 'dur_factor:2'

# Cap the amount the source file can be sped up or slowed down to reasonable
# values, since if it's too fast or slow it'll sound weird.
# Slows it down...
if dur_factor > 1.3
    dur_factor = 1.3
    printline Adjusted duration morph factor: 'dur_factor:2'
endif
# Speeds it up...
if dur_factor < 0.7
    dur_factor = 0.7
    printline Adjusted duration morph factor: 'dur_factor:2'
endif

# Select the source sound.
select 'source_sound_id'
# Adjust length of sound by the factor computed earlier.
#TODO default frequencies for this command?
result = Lengthen (overlap-add)... 100 600 'dur_factor'
# Check the new duration to see that it's changed appropriately.
result_duration = Get total duration
printline Morphed duration: 'result_duration:2'
select result

# Adjust the pitch of the source file a little up or down to be closer to the
# pitch of the target. Won't shift the pitch entirely or replace the pitch
# contour because the source has a lot of pitch variation and is in a particular
# range, so changing these would change the source too much and may make it
# sound pretty weird (e.g. emphasis on wrong syllables).
#
# If the mean pitch of the target is higher than the source's mean pitch by some
# threshold amount, shift the source up a little. Otherwise, if the the target's
# mean pitch is lower than the source's mean pitch by some threshold amount,
# shift the source down a little.

# TODO age removed for now -- do we want to do something with it?
#TODO range?
if target_mean_pitch = undefined
    adjust_pitch_by = 0
elsif ('target_mean_pitch' > ('source_mean_pitch' + 30))
    adjust_pitch_by = 30
elsif ('target_mean_pitch' < ('source_mean_pitch' - 30))
    adjust_pitch_by = -30
else
    adjust_pitch_by = 'target_mean_pitch' - 'source_mean_pitch'
endif

printline Age mean pitch: 'age_mean_pitch'
printline Source mean pitch: 'source_mean_pitch:2'
printline Target mean pitch: 'target_mean_pitch:2'
printline Adjust pitch by: 'adjust_pitch_by:2'

# Now adjust the pitch.
if adjust_pitch_by <> 0
    # Get source start and end times.
    start_time = Get start time
    end_time = Get end time
    # Extract pitch tier and shift frequency.
    manipulation = To Manipulation... 0.01 floor ceiling
    pitch_tier = Extract pitch tier
    select pitch_tier
    Shift frequencies... start_time end_time adjust_pitch_by Hertz
    # Replace original pitch tier with shifted one.
    plus manipulation
    Replace pitch tier
    select manipulation
    result = Get resynthesis (overlap-add)
    select result
endif

# Save the adjusted sound to a new file.
Write to WAV file... 'output_file$'
Save as WAV file... 'output_file$'

# Remove any extra sounds and objects.
printline Cleaning up...
select 'source_sound_id'
plus 'source_textgrid_id'
plus 'target_textgrid_id'
plus 'target_sound_id'
plus result
Remove

# Done!
printline Done!

# Speech rate detection.
# counts syllables of all sound utterances in a directory
# NB unstressed syllables are sometimes overlooked
# NB filter sounds that are quite noisy beforehand
# NB use Silence threshold (dB) = -25 (or -20?)
# NB use Minimum dip between peaks (dB) = between 2-4 (you can first try;
#                                         For clean and filtered: 4)
procedure detectSpeechRate: .wav_file$

    # Silence threshold in dB, default -25.
    silencedb = -25
    # Minimum dip between peaks in dB, default 2.
    mindip = 2
    # Minimum pause duration in s, default 0.3.
    minpause = 0.3
    # Keep soundfiles and textgrids, default false.
    showtext = 0

    # Read in wav file.
    Read from file... '.wav_file$'

    # use object ID
    soundname$ = selected$("Sound")
    soundid = selected("Sound")

    originaldur = Get total duration
    # allow non-zero starting time
    bt = Get starting time

    # Use intensity to get threshold
    To Intensity... 50 0 yes
    intid = selected("Intensity")
    start = Get time from frame number... 1
    nframes = Get number of frames
    end = Get time from frame number... 'nframes'

    # estimate noise floor
    minint = Get minimum... 0 0 Parabolic
    # estimate noise max
    maxint = Get maximum... 0 0 Parabolic
    #get .99 quantile to get maximum (without influence of non-speech sound bursts)
    max99int = Get quantile... 0 0 0.99

    # estimate Intensity threshold
    threshold = max99int + silencedb
    threshold2 = maxint - max99int
    threshold3 = silencedb - threshold2
    if threshold < minint
        threshold = minint
    endif

    # get pauses (silences) and speakingtime
    # This also trims extra silence from the start and end of the file.
    To TextGrid (silences)... threshold3 minpause 0.1 silent sounding
    textgridid = selected("TextGrid")
    silencetierid = Extract tier... 1
    silencetableid = Down to TableOfReal... sounding
    nsounding = Get number of rows
    npauses = 'nsounding'
    speakingtot = 0
    for ipause from 1 to npauses
        beginsound = Get value... 'ipause' 1
        endsound = Get value... 'ipause' 2
        speakingdur = 'endsound' - 'beginsound'
        speakingtot = 'speakingdur' + 'speakingtot'
    endfor

    select 'intid'
    Down to Matrix
    matid = selected("Matrix")
    # Convert intensity to sound
    To Sound (slice)... 1
    sndintid = selected("Sound")

    # use total duration, not end time, to find out duration of intdur
    # in order to allow nonzero starting times.
    intdur = Get total duration
    intmax = Get maximum... 0 0 Parabolic

    # estimate peak positions (all peaks)
    To PointProcess (extrema)... Left yes no Sinc70
    ppid = selected("PointProcess")

    numpeaks = Get number of points

    # fill array with time points
    for i from 1 to numpeaks
        t'i' = Get time from index... 'i'
    endfor


    # fill array with intensity values
    select 'sndintid'
    peakcount = 0
    for i from 1 to numpeaks
        value = Get value at time... t'i' Cubic
        if value > threshold
              peakcount += 1
              int'peakcount' = value
              timepeaks'peakcount' = t'i'
        endif
    endfor


    # fill array with valid peaks: only intensity values if preceding
    # dip in intensity is greater than mindip
    select 'intid'
    validpeakcount = 0
    currenttime = timepeaks1
    currentint = int1

    for p to peakcount-1
        following = p + 1
        followingtime = timepeaks'following'
        dip = Get minimum... 'currenttime' 'followingtime' None
        diffint = abs(currentint - dip)

        if diffint > mindip
            validpeakcount += 1
            validtime'validpeakcount' = timepeaks'p'
        endif
        currenttime = timepeaks'following'
        currentint = Get value at time... timepeaks'following' Cubic
    endfor


    # Look for only voiced parts
    select 'soundid'
    To Pitch (ac)... 0.02 30 4 no 0.03 0.25 0.01 0.35 0.25 450
    # keep track of id of Pitch
    pitchid = selected("Pitch")

    voicedcount = 0
    for i from 1 to validpeakcount
        querytime = validtime'i'

        select 'textgridid'
        whichinterval = Get interval at time... 1 'querytime'
        whichlabel$ = Get label of interval... 1 'whichinterval'

        select 'pitchid'
        value = Get value at time... 'querytime' Hertz Linear

        if value <> undefined
            if whichlabel$ = "sounding"
                 voicedcount = voicedcount + 1
                 voicedpeak'voicedcount' = validtime'i'
            endif
        endif
    endfor


    # calculate time correction due to shift in time for Sound object versus
    # intensity object
    timecorrection = originaldur/intdur

    # Insert voiced peaks in TextGrid
    if showtext > 0
        select 'textgridid'
        Insert point tier... 1 syllables

        for i from 1 to voicedcount
            position = voicedpeak'i' * timecorrection
            Insert point... 1 position 'i'
        endfor
    endif

    # clean up before next sound file is opened
    select 'intid'
    plus 'matid'
    plus 'sndintid'
    plus 'ppid'
    plus 'pitchid'
    plus 'silencetierid'
    plus 'silencetableid'
    Remove

    # summarize results in Info window
    if originaldur <> 0
        speakingrate = 'voicedcount'/'originaldur'
    else
        speakingrate = 0
    endif
    if speakingtot <> 0
        articulationrate = 'voicedcount'/'speakingtot'
    else
        articulationrate = 0
    endif
    npause = 'npauses'-1
    if voicedcount <> 0
        asd = 'speakingtot'/'voicedcount'
    else
        asd = 0
    endif

    # print a single header line with column names and units
    printline soundname, nsyll, npause, dur (s), phonationtime (s), speechrate (nsyll/dur), articulation rate (nsyll / phonationtime), ASD (speakingtime/nsyll)

    printline 'soundname$', 'voicedcount', 'npause', 'originaldur:2', 'speakingtot:2', 'speakingrate:2', 'articulationrate:2', 'asd:3'

endproc
