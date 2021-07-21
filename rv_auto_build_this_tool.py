#!/usr/bin/env python3
import logging
import os
import shutil
import stat

WHISPER_LOCK_FILE = 'LOCK.compile.whisper'
WHISPER_QUAL_FILE = 'QUAL.build_ok.whisper'
if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    cmd = os.path.abspath(__file__)
    release_dir = os.path.abspath(os.path.dirname(__file__))
    build_dir = os.path.join(release_dir, 'build-Linux')
    logging.info("Running cmd=(%s) in (%s) to build (%s) in (%s)" % (cmd, release_dir, 'whisper', build_dir))
    os.makedirs(build_dir, exist_ok=False)  # This dir should not exist, unless someone checked in by mistake
    os.chdir(release_dir)
    os.system('touch ' + WHISPER_LOCK_FILE)
    if os.path.exists(WHISPER_QUAL_FILE):
        os.remove(WHISPER_QUAL_FILE)
    print("\nWARNING: Compile (whisper) may take more than 2+ mins, please do NOT Ctrl-C or kill this!\n" * 10)
    os.system('make BOOST_DIR=/usr/local SOFT_FOLAT=1')
    expected_exe = os.path.join(build_dir, 'whisper')
    final_exe = os.path.join(release_dir, 'whisper')
    logging.info("Attempt to move executable (%s) to (%s) as (%s)" % (expected_exe, release_dir, final_exe))
    os.rename(expected_exe, final_exe)
    os.chmod(final_exe, stat.S_IRUSR | stat.S_IXUSR | stat.S_IRGRP | stat.S_IXGRP)
    if not os.access(final_exe, os.X_OK):
        raise PermissionError("For some reason (%s) is still not executable" % final_exe)
    os.chdir(release_dir)  # just to make sure
    os.remove(WHISPER_LOCK_FILE)
    if os.path.exists(os.path.join(build_dir, 'whisper.cpp.d')):  # Sanity check to make sure it looks like a build area
        shutil.rmtree(path=build_dir, ignore_errors=False)  # don't ignore error, let the tool log that for you.
    else:
        raise PermissionError("rmdir aborted, temporary build area (%s) does not meet expectation" % build_dir)
    logging.info('[whispering] It is all good. Best luck to you, shhhhhhh! ')
    exit(os.system('touch ' + WHISPER_QUAL_FILE))
