# dependencies
The team6_image_classifier package depends on the following dependencies

- scikit-learn  
  - pip install -U scikit-learn
- scikit-image
  - pip install -U scikit-image
- opencv
  - pip install opencv-python
- matplotlib
  - pip install -U matplotlib 
- argparse
  - pip install argparse

# usage
To run the image classifier you need a folder with jpg test images. 

Inside the folder there must be a text file that contains a list
of comma-seperated image names and ground truth labels

```
data
│   1.jpg
│   2.jpg
│   ...    
│   N.jpg
└───label.txt
```

To test the classifier, run the following:

```
python classify_images.py --filepath=<filepath> --groundtruth=<groundtruth>
```

Parameters:

filepath: (str)
  -- filepath to folder containing images

groundtruth: (str) 
  -- name of text file containing ground truth labels