# Classifies a feature into any number of classes

# Linear classfication defined is
# y = f(x, w, b) where...
#   x is a vector of input features of an object
#   w is a vector of weights to apply to the features
#   b is the bias of the feature-weight system
#   f() is x dot w + b
#   y is the final output score


# Classifies the object into two distinct class based on a cutoff value
# Anything less than the cutoff is of class false, greater than the cutoff is of class true
# 
# @param input The vector of input features
# @param weights The vector of weights to apply to the input features
# @param bias The bias of the features-weight system
# @param cutoff The number which splits the output score of the object into two classes
# @param Returns tuple of the class (true or false) and the given score
def binary_classification(input, weights, bias, cutoff):
    score = linear_classification(input, weights, bias)
    return (score < cutoff, score)

# Returns the raw output score of the linear classifier based on the dot product
#
# @param input The vector of input features
# @param weights The vector of weights to apply to the input features
# @param bias The bias of the features-weight system
def linear_classification(input, weights, bias):
    # Element wise multiplication
    out = map(lambda x, w: x * w, input, weights)

    return sum(out) + bias