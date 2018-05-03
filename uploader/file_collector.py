from collections import defaultdict
import logging
import os
import re

import inotify.adapters

log = logging.getLogger(__name__)

class FileCollector(object):
  DEFAULT_PATTERNS = [
    re.compile('(.*)\+annotated.jpg'),
    re.compile('(.*)\+0000ms.jpg'),
    re.compile('(.*)\+0500ms.jpg'),
    re.compile('(.*)\+1000ms.jpg'),
    re.compile('(.*)\+1500ms.jpg'),
    re.compile('(.*)\+2000ms.jpg'),
    re.compile('(.*)\+2500ms.jpg'),
  ]

  def __init__(self, watch_dir, patterns=None):
    if patterns is None:
      patterns = self.DEFAULT_PATTERNS
    self.watch_dir = watch_dir
    self.patterns = patterns

  def packages(self):
    i = inotify.adapters.Inotify()
    i.add_watch(self.watch_dir.encode('utf-8'))

    self.packages = defaultdict(set)
    for event in i.event_gen():
      if event is not None:
        (header, type_names, watch_path, filename) = event
        if 'IN_CLOSE_WRITE' not in type_names:
          # Only process files that have "finished writing" to exclude when the
          # files open up again for boto upload.
          continue

        path = os.path.join(watch_path, filename)
        for rgx in self.patterns:
          md = rgx.search(path)
          if md:
            log.debug('%s - pattern match', path)
            prefix = md.group(1)
            self.packages[prefix].add(path)
            if len(self.packages[prefix]) == len(self.patterns):
              log.info('Yielding finished package - %r', self.packages[prefix])
              yield list(self.packages[prefix])
              del self.packages[prefix]
            break
        else:
          log.warn('%s - Unrecognized path in watch dir', path)
