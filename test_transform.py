import torchvision
import cv2
import PIL
import numpy as np


def showImage(img):
    cv2.namedWindow('image', cv2.WINDOW_KEEPRATIO)
    cv2.imshow('image', img)
    cv2.resizeWindow('image', 200, 280)
    cv2.waitKey(200)


# Read image using PIL
img = PIL.Image.open('datasets/6outpost/000000.png')

# Define transfrom RandomAffine
transform = torchvision.transforms.RandomAffine(
    degrees=(-5, 5), translate=(0.08, 0.08), scale=(0.9, 1.1))

# Define transfrom
transform2 = torchvision.transforms.RandomErasing(
    scale=(0.02, 0.02))

# Apply transform
for i in range(500):
    img2 = transform(img)
    img_t = torchvision.transforms.ToTensor()(img2)
    img_t = transform2(img_t)
    img2 = torchvision.transforms.ToPILImage()(img_t)
    # PIL to cv2
    img2 = cv2.cvtColor(np.asarray(img2), cv2.COLOR_RGB2BGR)
    showImage(img2)
