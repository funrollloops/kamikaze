#!/usr/bin/env python

"""Script that uploads captured blaster JPGs to a central server.

For now, all this does is monitor the shots directory (or the given directory)
and log out the paths that would have been uploaded.
"""
import logging
import multiprocessing
import os

import gflags
import inotify.adapters

import db
from file_collector import FileCollector
import upload_worker

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

def _preload_queue(q):
  for package_id in db.pending_packages():
    q.put(package_id)

def _queue_worker(abs_path, q):
  fc = FileCollector(abs_path)
  # fc.packages() is a generator that yields paths based on file events forever.
  for package in fc.packages():
    package_id = db.write_package(package)
    q.put(package_id)

def main():
  db.init()
  abs_path = _get_abs_path()

  q = multiprocessing.Queue()

  _preload_queue(q)

  upload_job = multiprocessing.Process(
    target=upload_worker.upload_worker, args=(q,))
  upload_job.start()

  queue_job = multiprocessing.Process(target=_queue_worker, args=(abs_path, q))
  queue_job.daemon = True
  queue_job.start()

  try:
    upload_job.join()
    queue_job.join()
  except KeyboardInterrupt:
    upload_job.terminate()
    queue_job.terminate()
    upload_job.join()
    queue_job.join()

if __name__ == '__main__':
  from sys import argv
  FLAGS(argv)
  main()
