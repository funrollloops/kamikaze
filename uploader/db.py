import sqlite3

import gflags

gflags.DEFINE_string('upload_db', 'uploads.db', 'sqlite database file for '
                     'persisting the upload queue between sessions')
FLAGS = gflags.FLAGS

conn = None

def init():
  global conn
  conn = sqlite3.connect(FLAGS.upload_db)
  c = conn.cursor()
  c.execute('''CREATE TABLE IF NOT EXISTS packages
               (id integer primary key, code text, created_on integer)''')
  c.execute('''CREATE TABLE IF NOT EXISTS files
               (id integer primary key, package_id integer, path text,
                status text)''')
  conn.commit()

def write_package(paths):
  c = conn.cursor()
  c.execute('''INSERT INTO packages (code, created_on)
               VALUES ("foobar", strftime("%s", "now"))''')
  package_id = c.lastrowid
  for p in paths:
    c.execute('''INSERT INTO files (package_id, path, status) VALUES
                 (?, ?, 'PENDING')''', (package_id, p))
  conn.commit()
  return package_id

def get_package(package_id):
  c = conn.cursor()
  c.execute('''SELECT p.code, f.id, f.path, f.status FROM
               packages AS p JOIN files AS f
               ON p.id = f.package_id
               WHERE p.id = ?''', (package_id,))
  rows = c.fetchall()
  p = {'id': package_id, 'code': rows[0][0], 'files': []}
  for row in rows:
    p['files'].append({'id': row[1], 'path': row[2], 'status': row[3]})
  return p

def update_file_status(file_id, status):
  c = conn.cursor()
  c.execute('''UPDATE files SET status = ? WHERE id = ?''', (status, file_id))
  conn.commit()
                   
def pending_packages():
  c = conn.cursor()
  c.execute('''SELECT DISTINCT(p.id) FROM
               packages AS p JOIN files AS f
               ON p.id = f.package_id
               WHERE f.status = "PENDING"
               ORDER BY p.created_on''')
  return [x[0] for x in c.fetchall()]
