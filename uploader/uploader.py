"""Script that uploads captured blaster JPGs to a central server.

For now, all this does is monitor the shots directory (or the given directory)
and log out the paths that would have been uploaded.
"""

import logging
import os

import gflags
import inotify.adapters

from file_collector import FileCollector

gflags.DEFINE_string('watch_dir', 'shots', 'Directory to watch for media')
FLAGS = gflags.FLAGS

logging.basicConfig(level=logging.INFO)
log = logging.getLogger(__name__)

def _get_abs_path():
  abs_path = os.path.abspath(FLAGS.watch_dir)
  if not os.path.exists(abs_path):
    raise ValueError('Given watch dir (%s) does not exist' % abs_path)
  if not os.path.isdir(abs_path):
    raise ValueError('Given watch dir (%s) is not a directory' % abs_path)
  return abs_path

def main():
  abs_path = _get_abs_path()
  fc = FileCollector(abs_path)
  for package in fc.packages():
    print(repr(package))

if __name__ == '__main__':
  from sys import argv
  FLAGS(argv)
  main()
