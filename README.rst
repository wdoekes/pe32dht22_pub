pe32dht22_pub
=============

This still needs some documentation.

What *is* documented however, is the automatic ``build_version.h`` updates.


Auto updating build_version.h
-----------------------------

Find platform.txt and edit platform.local.txt:

.. code-block:: console

    $ find ~/Jailed/arduino/ -name platform.txt -o -name 'platform.local.txt'
    /home/walter/Jailed/arduino/.arduino15/packages/esp32/hardware/esp32/2.0.5/platform.local.txt
    /home/walter/Jailed/arduino/.arduino15/packages/esp32/hardware/esp32/2.0.5/platform.txt
    /home/walter/Jailed/arduino/.arduino15/packages/arduino/hardware/avr/1.8.6/platform.txt

    $ cat >>/home/walter/Jailed/arduino/.arduino15/packages/esp32/hardware/esp32/2.0.5/platform.local.txt <<EOF
    recipe.hooks.sketch.prebuild.1.pattern=/usr/local/bin/exec-build-version-h-for-arduino-ide "{build.source.path}/build_version.h" "{build.path}/sketch/build_version.h"
    EOF

Add the following file with execute permissions as
``/usr/local/bin/exec-build-version-h-for-arduino-ide``:

.. code-block:: sh

    #!/bin/sh

    # Setup, in platform.[local.]txt you add this:
    # recipe.hooks.sketch.prebuild.1.pattern=/usr/local/bin/exec-build-version-h-for-arduino-ide "{build.source.path}/build_version.h" "{build.path}/sketch/build_version.h"

    first_line=$(head -n1 "$1" 2>/dev/null)
    if test "$first_line" != "/*#!/usr/local/bin/exec-build-version-h-for-arduino-ide 2>/dev/null"; then
        echo "$0: first line of '$1' is not '/*#!/usr/local/bin/exec-build-version-h-for-arduino-ide 2>/dev/null'" >&2
        echo "$0: skipping..." >&2
        exit 0
    fi

    # Check whitelist so that we do not run random stuff from the internet.
    # (Double check config path if you're (fire)jailing this into a private dir.)
    if ! grep -qxF "$1" "$HOME/.config/arduino-ide/exec-build-version-h.whitelist"; then
        echo "$0: '$1' not in ~/.config/arduino-ide/exec-build-version-h.whitelist" >&2
        echo "$0: not executing..." >&2
        exit 0
    fi

    # Jump to source directory so git can be run.
    cd "$(dirname "$1")"

    # Run it (and write to $2).
    if test -n "$2"; then
        # Write to file {build.path}/sketch/build_version.h
        /bin/sh "$1" >"$2.tmp" && mv "$2.tmp" "$2"
        sed -e 's@^@(debug) exec-build-version-h-for-arduino-ide: @' "$2" >&2
    else
        # Simply dump to stdout.
        /bin/sh "$1"
    fi

Restart *Arduino IDE* for the ``platform.local.txt`` changes to take effect.


Changing build_version.h
------------------------

A skeleton ``build_version.h`` looks like this:

.. code-block:: c

    /*#!/usr/local/bin/exec-build-version-h-for-arduino-ide 2>/dev/null
    ... shell script here ...
    #(header, do not touch next two lines)
    cat <</*
    //*/
    ... header file here ...
    /*
    #(trailer, do not touch this and previous line)*/

For the shell script, we can add:

.. code-block:: sh

    GIT_VERSION=$(git describe --tags --dirty --always)

For the header file, we can add:

.. code-block:: c

    #define GIT_VERSION "$GIT_VERSION"

When asking the *Arduino IDE* to compile, it will run
``/usr/local/bin/exec-build-version-h-for-arduino-ide`` on the
``build_version.h``. If this fails, then original ``build_version.h``
which is legal C is used. If it succeeds, the output is written to a
temporary ``build_version.h`` in the compilation directory.

Build output then shows::

    (debug) exec-build-version-h-for-arduino-ide: //*/
    (debug) exec-build-version-h-for-arduino-ide: #define GIT_VERSION "b9e7973-dirty"

And ``GIT_VERSION`` can be used as a regular macro.
