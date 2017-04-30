
# Mathematical Optimization

Mathematical optimization involves the maximization or minimization of a function through cleaver choices of the input variables and the analysis of the the output. They are mostly iterative as the underlying function is not easily calculated.

Below is the comparison of the optimization algorithms that we use and their general use cases.

## Gradient Ascent

Gradient Ascent involves using the first order derivative to choose the next point to test. It will then move in the direction it increases the fastest. As it passes the maximum, the step size, or temperature, decreases. Once the gradient is close enough to zero or the temperature is low enough, the optmization algorithm exits.

Gradient Ascent is very good at climbing local maxes but has difficulty when facing a multimodel function. 

## Parallel Gradient Ascent

Since Gradient Ascent has trouble with multiple local maxes, using mulitple starting points allows for the global maximum to be found. To do this, some knowledge of the function must be used to place a starting point near each maximum.

## Nelder-Mead

Nelder-Mead is an algorithm that involves using a simplex to "flip" it's way up a hill, shrinking and expanding when neccessary. The simplex has N+1 vertices where N is the number of inputs to the function you are trying to optimize.

This method is useful when each function call has a very high calculation cost. Nelder-Mead tries to find the maximum through the fewest amount of calls possible.

Nelder-Mead can do both local and global maximization based upon the starting inputs. Special care must be taken when choosing these as the output varies widly. Specifically, the starting location and the input step sizes.


