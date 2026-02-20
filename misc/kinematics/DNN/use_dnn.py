from keras.models import load_model
import numpy as np

model = load_model('./models/my_model.h5')
model.compile(loss='mean_squared_error', optimizer='adam', metrics=['accuracy'])

# Load the data
input_data = np.loadtxt('./data/testing_input.csv', delimiter=',')
output_data = np.loadtxt('./data/testing_output.csv', delimiter=',')

for i in range(10):
    x = input_data[i].reshape(1,-1)
    y = output_data[i].reshape(1,-1)

    result = model.predict(x)

    print("Input:\t\t\t{}".format(np.round(x,2)))
    print("Output:\t\t\t{}".format(np.round(result,3)))
    print("Expected output:\t{}".format(np.round(y,3)))
    print("Difference:\t\t{}".format(np.abs(np.round(result-y, 3))))
    print("")