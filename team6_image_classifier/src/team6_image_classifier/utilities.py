import cv2
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable

from sklearn.externals import joblib

def load_img_data(src, grnd_truth='label.txt', pklname=None, width=150, height=None):
	"""
	load images from path that contains a labels.txt file
	with rows of comma-seperated image name and labels

	ex. <pic_name>, <label>

	Resizes images to (width, height)
	"""
	src = Path(src)
	height = width if height is None else height

	# read image labels
	label_dict = {} # {filename: , label: }
	with (src / grnd_truth).open() as f:
		for line in f:
			key, value = line.split(',')
			label_dict[key] = int(value)

	data = dict()
	data['label'], data['data'], data['filename'] = [], [], []

	# read and label images
	img_paths = sorted(src.glob('*.jpg'))
	for file in img_paths:
			filename = str(Path(file).stem)
			label = label_dict[filename]

			data['label'].append(label)
			data['filename'].append(filename)
			
			img = cv2.imread(str(file))
			img = cv2.resize(img, (width, height))
			data['data'].append(img)

	# write data to pickle if desired
	if pklname is not None:
		pklname = "{pklname}_{width}x{height}px.pkl"
		joblib.dump(data, pklname)

	return data


def plot_train_test_bar(label_train, label_test):
	"""
	Plot a bar graph that shows the distrbuition of training 
	and test data at each label
	"""
	width = 0.35

	# training data
	unique, counts = np.unique(label_train, return_counts=True)
	sorted_index = np.argsort(unique)
	unique, counts  = unique[sorted_index], counts[sorted_index]

	xtemp = np.arange(len(unique))
	plt.bar(xtemp - width/2, counts, width)

	# test data
	unique, counts = np.unique(label_test, return_counts=True)
	sorted_index = np.argsort(unique)
	unique, counts = unique[sorted_index], counts[sorted_index]

	xtemp = np.arange(len(unique))
	plt.bar(xtemp + width/2, counts, width)
	plt.legend(['train ({0})'.format(len(label_train)),
			    'test ({0})'.format(len(label_test))])
	plt.xlabel('label')
	plt.show()


def load_pickle_data(src):
	pass # TODO: load pickle instead of image 

def save_model(model, name):
	joblib.dump(model, name)

def plot_confusion_matrix(conf_mat):
	conf_mat_norm = 100*conf_mat / conf_mat.sum(axis=1, keepdims=True)
	
	ax = plt.subplot()
	im = ax.imshow(conf_mat_norm)
	ax.set_title('Confusion Matrix')
	ax.set_xlabel('Predicted')
	ax.set_ylabel('True')
	
	divider = make_axes_locatable(ax)
	cax = divider.append_axes("right", size="5%", pad=0.05)
	cb = plt.colorbar(im, cax=cax)
	cb.set_label("Percentage Correct")

	# create text annotations
	for i in range(conf_mat_norm.shape[0]):
		for j in range(conf_mat_norm.shape[1]):
			test = ax.text(j, i, conf_mat[i, j], ha='center', va='center', fontweight='bold')

	plt.show()