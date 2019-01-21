# Friction Estimation

Due to the heavy reliance on predicting ball location, a good estimation of friction is needed. Unfortuantly, getting this estimation is sometimes difficult. A seperate branch has been created to directly write a list of positions/velocities as a function of time into a csv file.

## Procedure

1. Checkout the "friction_estimation" branch from robojackets/robocup-software
2. Remove the ball from the view of the cameras
3. Start soccer
4. Wait until the field has updated with the geometry packet (Usually a second or two)
5. Slowly roll the ball onto the field. It should be fast enough to reach the other side in a somewhat straight line, but slow enough to take a second or two at least.
6. Close soccer once the ball bounces off the 2x4 boundry
7. Open the "test.csv" in the root directory of robocup-software
8. Make sure to select "," as the delimeter so everything is correctly split into columns
9. Add "=E1-$E$1" into the cell F1
10. Click and drag the bottom right corner of the cell F1 to apply the formula to all rows with data
11. Add "=sqrt(C1\*C1+D1\*D1)" to cell G1
12. Click and drag the bottom right corner of the cell G1 to apply the formula to all rows with data
13. Select columns F and G using the letters at the top
14. Create a scatter chart without any data point format (Just lines)
15. Remove rows and columns that correspond to the vision filter trying to update to the balls initial velocity and anything after and including the bounce off the wall. The data should resemble a line with negative slow (and noise)
16. Add a trendline making sure to show the formula used. The trendline should be a simple y=ax+b formula
17. The slope should correspond to the correct friction acceleration corrseponding to that run.
18. In the "SystemState.cpp" file, the "ballDecayConstant" represents the acceleration due to friction

## Notes
* The average of multiple runs should be taken
* The full set of ball movement should be explored. (Down the length of the field, diagonally across, across the goal etc)
* Each trial should be saved to compare them at the end.
* Try not to include the stopped ball in the data set. The fact that it stopped will mess up the least squares
* Make sure that the vision coefficients are the same between the current master and this "friction_estimation" branch
* Try to set the simulator friction to the same one found on the field.
* Keep the ball off the field and toss it on to allow for data collection to start at the moment vision sees the ball.
* See PR #1233 for some example graphs