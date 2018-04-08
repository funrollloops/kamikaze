from collections import defaultdict
import logging
import os
import re

import inotify.adapters

log = logging.getLogger(__name__)

class FileCollector(object):
  DEFAULT_PATTERNS = [
    re.compile('(.*)_0_annotated.jpg'),
    re.compile('(.*)_1_before.jpg'),
    re.compile('(.*)_2_during.jpg'),
    re.compile('(.*)_3_after.jpg'),
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
