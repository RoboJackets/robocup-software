# Vision Filter

Based on a mixture of the Mannheim Tiger's vision filter from their [auto-ref](https://gitlab.tigers-mannheim.de/open-source/AutoReferee) and something known as the [Multi-Hypothesis Extended Kalman Filter (MHEKF)](https://dainamite.github.io/public/publication/qian_master_thesis.pdf) found in a paper from the RoboCup SPL league team DAInamite. The overall goal of this module is to convert the raw vision packet data from SSL-Vision and creates a good estimate of the true robot and ball positions.

The overall setup is shown in the image below. Vision data is fed into the world object. This world object splits up the vision data by camera ID and feeds it into each corresponding camera object. The camera objects then apply the vision data as measurements to individual kalman filters corresponding to balls and robots. The world object then combines the best kalman filter from each camera for each individual robot and ball.


<img src="https://i.imgur.com/BjU5TU8.png" width="800">

## SSL-Vision Format

Each individual camera sends a frame packet. Each frame packet contains one or more balls and one or more robots depending on what is in the camera view. Consequently, the frames are not sent at a consistent rate and you may get more than one camera frame at a time depending on how things work out. A ball may show up on multiple cameras so you may get a measurement in multiple camera frames. Sometimes extra balls and robots will show up for a frame or two due to noise.

## Kalman Filter

A kalman filter is a statistical approach to estimating positions/velocities (etc) of different objects. In this specific case, the model assumes constant velocity that integrates down to position. This is for each independent axis (X, Y and in the robot's case, heading). The filter keeps track of the X pos/vel, Y pos/vel (and heading pos/vel) as well as how confident it is in each estimate. 

Every iteration the filter either predicts or predicts and updates. When it predicts, it predicts current positions and velocities given what the position and velocities were last frame. When it predicts and updates, it again predicts it's current positions and velocities, but then compares that to the measurements it receives and updates it's position and velocity. If we are really confident in our current estimates, it doesn't really take into account the measurement. On the flip side, if we aren't confident, we almost completely use the measurement as the truth. Additionally, the confidence of our estimate steadily decreases each time we predict without an update.

## Camera

The camera object receives all the measurements for a specific camera ID. The camera then chooses how to apply these measurements to the kalman filters objects it contains. There are two different algorithms to match the measurements to kalman filters, the first is the average kalman filter and the other is the multi-hypothesis kalman filter.

The average kalman filter averages all the measurements corresponding to that specific object and then applies it to the single kalman filter for that object. For example, the ball object will average the positions of all the measurements from the camera frame and use that as the measurement for the single ball kalman filter.

The multi-hypothesis kalman filter takes all measurements that are within a specific distance of the object and averages those together. This average is then used as the measurement for that specific kalman filter. Any that are not "claimed" by a kalman filter are used as the initial position for new kalman filters. There can therefor be multiple kalman filters for a specific object like a ball.

Additionally, we use a simple geometric model to predict how the ball velocity will change when it bounces off of robots to improve the estimation.

## World

The world object takes the list of packets from SSL-Vision and splits it into data for each camera. It then sends the data down to the cameras for processing. Once that processing is done, the world object then takes the best kalman filter for each object from each frame. The set of kalman filters corresponding to a single object from all the cameras are merged together based on their confidence. This set of merged objects are referred to with the World* qualifier and are our best estimates of the pos/vel of the objects.

## VisionFilter

The vision filter module converts the protobuf format from SSL-Vision to a custom format with the Camera* qualifier. This removes all the extra data that isn't needed and makes it easier to edit code. Additionally, there are operations to push the data from the filter into the system state object.

The vision filter module is threaded to run at a very specific timing matching the kalman filter predict/update rate.