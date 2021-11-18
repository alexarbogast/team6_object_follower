#!/usr/bin/env python

import numpy as np
import argparse
from pathlib import Path
from sklearn import metrics
from os.path import exists

from sklearn.externals import joblib
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler

from team6_image_classifier import BGR2GrayTransformer, HogTransformer
from team6_image_classifier import utilities
from team6_image_classifier.utilities import load_img_data

MODEL_PATH = '../data/classifier/sgdclassifier.pkl'

def test_classifier(filepath, groundtruth='label.txt'):
	# ============== load classifier ===============
	model_path = MODEL_PATH
	print(model_path)

	sgd_classifier = joblib.load(model_path)

	## load images
	data = load_img_data(filepath, groundtruth, width=64, height=128)
	data, label = np.array(data['data']), np.array(data['label'])

	print("Imported {0} data instances".format(data.shape[0]))

	# ============= create pipeline ================
	HOG_pipeline = Pipeline([
	('grayify', BGR2GrayTransformer()),
	('hogify', HogTransformer(
		pixels_per_cell=(14, 14), 
		cells_per_block=(2, 2), 
		orientations=9, 
		block_norm='L2-Hys')
	),
	('scalify', StandardScaler())
	])

	# ============= extract features ===============
	prepd_data = HOG_pipeline.fit_transform(data)

	# =========== evaluate performance =============
	predicted = sgd_classifier.predict(prepd_data)
	print(metrics.classification_report(label, predicted))	 
	
	print("Confusion Matrix\n===============\n")
	cmx = metrics.confusion_matrix(label, predicted)
	print(cmx)
	utilities.plot_confusion_matrix(cmx)

			
def main():
	# construct argument parse and parse arguments
	ap = argparse.ArgumentParser()
	ap.add_argument("-f", "--filepath", type=str,
		help="path to input image directory")
	ap.add_argument("-gt", "--groundtruth", type=str,
		default='label.txt',
		help="name of ground truth file in <filepath> directory")
	args = ap.parse_args()

	# TODO: make sure both paths exist before testing classifier
	filepath, groundtruth = args.filepath, args.groundtruth
	
	if not exists(filepath):
		print("Filepath {0} does not exist".format(filepath))
		return
	elif not exists(filepath + '/' + groundtruth):
		print("Ground turth file {0} does not exist".format(groundtruth))
		return 

	# all files found
	test_classifier(args.filepath, args.groundtruth)


if __name__ == '__main__':
		main()