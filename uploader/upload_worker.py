import logging
import os

import boto3

import db

log = logging.getLogger(__name__)
s3 = boto3.resource('s3')

def upload_worker(q):
  while True:
    try:
      package_id = q.get()
      log.info('Uploading package with id=%s', package_id)
      p = db.get_package(package_id)
      log.info('Found package: %r', p)

      for f in p['files']:
        try:
          log.info('Uploading %s', f['path'])
          obj_name = '%s/%s' % (p['code'], os.path.basename(f['path']))
          s3.Object('blaster-gallery', obj_name).put(Body=open(f['path'], 'rb'))
          db.update_file_status(f['id'], 'COMPLETE')
        except:
          log.exception('Exception during file upload - %s', f['path'])
          db.update_file_status(f['id'], 'ERROR') 
    except:
      log.exception('Exception during upload loop')
