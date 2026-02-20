import os
import time
import string
import random
import argparse

import numpy as np

from tensorflow.keras.layers import Dense
from tensorflow.keras.models import Sequential
from tensorflow.keras.callbacks import EarlyStopping
from tensorflow.keras.callbacks import ModelCheckpoint

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument("--model",              type=str, default="12,8,8,6")
parser.add_argument("--directory",          type=str, default=".")
parser.add_argument('--data_resolution',    type=int, default=10)
parser.add_argument('--batch_size',         type=int, default=10)
parser.add_argument('--shuffle',            action='store_true')
args = parser.parse_args()

# Get the user arguements
print("Get the user arguements")
data_resolution = args.data_resolution
shuffle_data = args.shuffle
batch_size = args.batch_size
model_definition = args.model
directory = args.directory

# Create the file name for saving
print("Create the file name for saving")
timestr     = time.strftime("%Y%m%d-%H%M%S")
randomstr   = ""
randomchr   = random.choices(string.ascii_lowercase, k=5)
randomstr   = randomstr.join(randomchr)
filename    = timestr + "-" + randomstr

# Create model dir
if not os.path.exists("{}/results".format(directory)):
    os.makedirs("{}/results".format(directory)) 

# Create model dir
if not os.path.exists("{}/models".format(directory)):
    os.makedirs("{}/models".format(directory))  

# load the dataset
print("Load the dataset")
input_training_data = np.loadtxt('{}/data/training_input_{}.csv'.format(directory, data_resolution), delimiter=',')
output_training_data = np.loadtxt('{}/data/training_output_{}.csv'.format(directory, data_resolution), delimiter=',')
input_validation_data = np.loadtxt('{}/data/validation_input_{}.csv'.format(directory, data_resolution), delimiter=',')
output_validation_data = np.loadtxt('{}/data/validation_output_{}.csv'.format(directory, data_resolution), delimiter=',')

# define the keras model
print("Create the model")
model = Sequential()
layer_def = model_definition.split(",")
first_layer = True

for layer_size in layer_def:
    if first_layer:
        first_layer = False
        model.add(Dense(int(layer_size), input_shape=(8,), activation='relu'))
    else:
        model.add(Dense(int(layer_size), activation='relu'))
# Add the final layer
model.add(Dense(3))

# compile the keras model
print("Compile the model")
model.compile(loss='mean_squared_error',
              optimizer='adam',
              metrics=['accuracy'])

# Stop after increasing validation loss for x epochs
earlystop = EarlyStopping(monitor='val_loss',
                          patience=50,
                          verbose=0,
                          mode='min')

# Save the model with the best validation loss
bestmodel = ModelCheckpoint(filepath="{}/models/{}.h5".format(directory, filename),
                            save_best_only=True,
                            monitor='val_loss',
                            mode='min')

# fit the keras model on the dataset
print("Train the model")
history = model.fit(input_training_data, output_training_data,
                    epochs=5000,
                    batch_size=batch_size,
                    shuffle=shuffle_data,
                    validation_data=(input_validation_data, output_validation_data),
                    callbacks=[earlystop, bestmodel])

# evaluate the keras model on training data
train_loss, train_accuracy = model.evaluate(input_training_data, output_training_data)
print('Training Accuracy: %.3f' % (train_accuracy * 100))
print('Training Loss: %.3f' % (train_loss))

# evaluate the keras model on validation data
val_loss, val_accuracy = model.evaluate(input_validation_data, output_validation_data)
print('Validation Accuracy: %.3f' % (val_accuracy * 100))
print('Validation Loss: %.3f' % (val_loss))

# Save the results
with open('{}/results/{}.txt'.format(directory, filename), 'w') as f:
    f.write('Model Name: {}.h5\n'.format(filename))
    f.write('Date Time: {}\n'.format(timestr))
    f.write('Data Resolution: {}\n'.format(data_resolution))
    f.write('Shuffle Data: {}\n'.format(shuffle_data))
    f.write('Batch Size: {}\n'.format(batch_size))
    f.write('Model: {}\n'.format(model_definition))
    f.write('Directory: {}\n'.format(directory))
    f.write('\n')
    f.write('Training Accuracy: {}\n'.format(history.history["accuracy"]))
    f.write('\n')
    f.write('Training Loss: {}\n'.format(history.history["loss"]))
    f.write('\n')
    f.write('Validation Accuracy: {}\n'.format(history.history["val_accuracy"]))
    f.write('\n')
    f.write('Validation Loss: {}\n'.format(history.history["val_loss"]))
    f.write('\n')
    f.write('Final Training Accuracy: {}\n'.format(train_accuracy))
    f.write('Final Training Loss: {}\n'.format(train_loss))
    f.write('Final Validation Accuracy: {}\n'.format(val_accuracy))
    f.write('Final Validation Loss: {}\n'.format(val_loss))

