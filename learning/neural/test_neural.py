import caffe
import cv2

model = 'learning/neural/cubenet_deploy.prototxt'
weights = 'snapshots_iter_77000.caffemodel'

caffe.set_mode_gpu()
caffe.set_device(0)

net = caffe.Net(model, weights, caffe.TEST)

image = cv2.imread('assets/letters/temp_training_set/0.png')
res = net.forward(image)
prob = res[1]
print res[1]
