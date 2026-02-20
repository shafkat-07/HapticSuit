import glob
import numpy as np

from keras.models import load_model

direction = "inverse"

# Get all models
model_files = glob.glob("./models/{}/*.h5".format(direction))

# Load the data
input_data = np.loadtxt('./data/{}/testing_input_10.csv'.format(direction), delimiter=',')
output_data = np.loadtxt('./data/{}/testing_output_10.csv'.format(direction), delimiter=',')

lowest_loss         = np.inf
lowest_loss_model   = None

# For each model
for model_name in model_files:
    # Display which model we are loading
    print("Testing model: {}".format(model_name))

    # Load the model
    model = load_model(model_name)
    model.compile(loss='mean_squared_error', optimizer='adam', metrics=['accuracy'])

    # Evaluate the model
    loss, accuracy = model.evaluate(input_data, output_data)

    print('Loss: %.3f' % (loss))
    print('Accuracy: %.3f' % (accuracy*100))
    print("-----------------------")

    # Save the lowest loss
    if loss < lowest_loss:
        lowest_loss = loss
        lowest_loss_model = model_name

print("Determining best model:")
print("Best model was: {}".format(lowest_loss_model))
print("Best model loss: {}".format(lowest_loss))
print("\n")
print("Printing a few examples from the test set:")

model = load_model(lowest_loss_model)
model.compile(loss='mean_squared_error', optimizer='adam', metrics=['accuracy'])
for i in range(10):
    x = input_data[i].reshape(1,-1)
    y = output_data[i].reshape(1,-1)

    result = model.predict(x)

    print("Input:\t\t\t{}".format(np.round(x,2)))
    print("Output:\t\t\t{}".format(np.round(result,3)))
    print("Expected output:\t{}".format(np.round(y,3)))
    print("Difference:\t\t{}".format(np.abs(np.round(result-y, 3))))
    print("MSE:\t\t\t{}".format(np.round(((result - y)**2).mean(axis=1),4)))
    print("")