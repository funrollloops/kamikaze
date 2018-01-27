import logging

import db

log = logging.getLogger(__name__)

def upload_worker(q):
  while True:
    package_id = q.get()
    log.info('Uploading package with id==%s', package_id)
    p = db.get_package(package_id)
    log.info('Found package: %r', p)
