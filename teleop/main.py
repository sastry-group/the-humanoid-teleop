from TeleVision import TeleVision
from preprocessing.processing import PreProcessing

preprocessor = PreProcessing()

while True:
    head_rmat, lwrist, rwrist, lhand, rhand = preprocessor.process()