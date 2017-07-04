#!/usr/bin/env python2

import sys

import cv2

BLUE = (255, 0, 0)
GREEN = (0, 255, 0)
RED = (0, 0, 255)
TEAL = (255, 255, 0)
YELLOW = (0, 255, 255)
WHITE = (255, 255, 255)

DEFAULT_IMG = '/home/sagarm/Pictures/Webcam/2017-06-17-153909.jpg'
TARGET_POS = (320, 240)
TARGET_RANGE = (5, 5)

LEFT = 'left'
RIGHT = 'right'
UP = 'up'
DOWN = 'down'

DETECT_EYES = False
MAX_FACES = 1

def monkeypatch_nopreview():
  def do_nothing(*_args, **_kwargs):
    pass
  cv2.rectangle = do_nothing
  cv2.imshow = do_nothing
  cv2.waitKey = do_nothing

def target(img):
  print dir(img)

class Recognizer(object):
  def __init__(self):
    self.face_cascade = cv2.CascadeClassifier(
        'haarcascades/haarcascade_frontalface_default.xml')
    self.eye_cascade = cv2.CascadeClassifier('haarcascades/haarcascade_eye.xml')
    self.smile_cascade = cv2.CascadeClassifier(
        'haarcascades/haarcascade_smile.xml')

  @staticmethod
  def plot_feature(img, (x, y, w, h), color):
    cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)

  @staticmethod
  def subset_array(array, (x, y, w, h)):
    return array[y:y + h, x:x + w]

  def detect_and_show(self, img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    self.plot_feature(img, TARGET_POS + TARGET_RANGE, TEAL)
    faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
    for face in faces[:MAX_FACES]:
      self.plot_feature(img, face, BLUE)
      self.plot_feature(img, self.guess_mouth_location(face), YELLOW)
      x, y, w, h = face
      for eye in (
          self.eye_cascade.detectMultiScale(self.subset_array(gray, face))
          if DETECT_EYES else []):
        self.plot_feature(self.subset_array(img, face), eye, GREEN)
      smile_roi = (x, y + 2*h//3, w, h//3)
      smile = self.smile_filter(self.smile_cascade.detectMultiScale(
          self.subset_array(gray, smile_roi)))
      if smile is not None:
        self.plot_feature(self.subset_array(img, smile_roi), smile, RED)
        smile = (smile[0] + smile_roi[0], smile[1] + smile_roi[1], smile[2],
                 smile[3])
      else:
        smile = self.guess_mouth_location(face)
      action = self.determine_action(self.mouth_center(smile))
      cv2.putText(img, action, (0, 100), cv2.FONT_HERSHEY_PLAIN, 4, WHITE)
    cv2.imshow('img', img)

  @staticmethod
  def determine_action(mouth_center):
    action = ""
    if mouth_center[0] < TARGET_POS[0]:
      action += LEFT
    elif mouth_center[0] > TARGET_POS[0] + TARGET_RANGE[0]:
      action += RIGHT
    if mouth_center[1] < TARGET_POS[1]:
      action += UP
    elif mouth_center[1] > TARGET_POS[1] + TARGET_RANGE[1]:
      action += DOWN
    return action


  @staticmethod
  def smile_filter(smiles):
    if len(smiles) == 0:
      return None
    return sorted(smiles, key=lambda s: s[2]*s[3])[-1]

  @staticmethod
  def guess_mouth_location((x, y, w, h)):
    return (x + w//4, y + 9 * h//12, w//2, h//6)

  @staticmethod
  def mouth_center(mouth):
    return mouth[0] + mouth[2]//2, mouth[1] + mouth[3]//2

def detect_webcam():
  tt = Recognizer()
  try:
    cap = cv2.VideoCapture(0)
    while True:
      _, frame = cap.read()
      tt.detect_and_show(frame)
      key = cv2.waitKey(delay=1000//30)
      if key == ord('p'):
        key = cv2.waitKey(0)
      if key == ord('q'):
        break
  finally:
    cap.release()

def detect_images(paths):
  tt = Recognizer()
  for img in paths or [DEFAULT_IMG]:
    print '==', img
    tt.detect_and_show(cv2.imread(img))
    key = cv2.waitKey(0)
    if key == ord('q'):
      break

if __name__ == "__main__":
  monkeypatch_nopreview()
  try:
    if len(sys.argv) == 1:
      detect_webcam()
    else:
      detect_images(sys.argv[1:])
  finally:
    cv2.destroyAllWindows()
