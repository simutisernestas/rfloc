# https://scicoding.com/4-ways-of-calculating-autocorrelation-in-python/
import numpy as np

data = innovation # (y - y_hat)

x = np.array(data) 

# Mean
mean = np.mean(data)

# Variance
var = np.var(data)

# Normalized data
ndata = data - mean

acorr = np.correlate(ndata, ndata, 'full')[len(ndata)-1:] 
acorr = acorr / var / len(ndata)