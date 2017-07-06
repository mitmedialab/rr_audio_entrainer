# Praat requires a libstdc++ with symbol versioning for GLIBCXX_3.4.21
# which isn't in ubuntu releases prior to 16.04. As a workaround, we're
# embedding a newer version of libstdc++ from ubuntu 16.04 for Praat to use.
# Putting it in LD_LIBRARY_PATH means it'll be loaded in preference to the one
# in usr/lib that's included in the OS.
export LD_LIBRARY_PATH=$(pwd)/lib/$(uname -m)
