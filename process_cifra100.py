import pickle
import cv2


def unpickle(file):
    with open(file, 'rb') as fo:
        dict = pickle.load(fo, encoding='bytes')
    return dict


def processImage(img):
    img = img.reshape(3, 32, 32)
    img = img.transpose(1, 2, 0)
    # Resize to 20x28
    img = cv2.resize(img, (20, 28))
    # Do OTSU 自动计算阈值
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return img


def showImage(img):
    cv2.namedWindow('image', cv2.WINDOW_KEEPRATIO)
    cv2.imshow('image', img)
    cv2.resizeWindow('image', 200, 280)
    cv2.waitKey(1)

def processSet(set, name):
    for i in range(set[b'data'].shape[0]):
        img = set[b'data'][i]
        img = processImage(img)
        # showImage(img)
        path = 'datasets/9negative/' + name + str(i) + '.png'
        cv2.imwrite(path, img)
        print(path)

# Load data
train = unpickle('cifar-100-python/train')
test = unpickle('cifar-100-python/test')

# Process all images to negative dataset
processSet(train, 'cifar100_train_')
processSet(test, 'cifar100_test_')
