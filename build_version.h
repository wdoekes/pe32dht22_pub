/*#!/usr/local/bin/exec-build-version-h-for-arduino-ide 2>/dev/null
GIT_VERSION=$(git describe --tags --dirty --always)
BUILD_TIME=$(date -Iseconds)
BUILD_HOST=$(hostname -f)
#(header, do not touch next two lines)
cat <</*
//*/
//*/ Build info, automatically written using exec-build-version-h-for-arduino-ide
//*/ during prebuild.hooks.sketch stage, using platform.local.txt edit:
//*/ recipe.hooks.sketch.prebuild.1.pattern=/usr/local/bin/exec-build-version-h-for-arduino-ide "{build.source.path}/build_version.h" "{build.path}/sketch/build_version.h"
#define GIT_VERSION "$GIT_VERSION"
#define BUILD_TIME "$BUILD_TIME"
#define BUILD_HOST "$BUILD_HOST"
/*
#(trailer, do not touch this and previous line)*/
