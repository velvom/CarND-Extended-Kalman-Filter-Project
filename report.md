## Extended Kalman Filter
- Code ompiles without errors with cmake and make.
- As tabulated below, px, py, vx, vy output coordinates have an RMSE that is less than the __threshold__: [.11, .11, 0.52, 0.52] 

  | Dataset | px        | py        | vx       | vy       |
  |---------|-----------|-----------|----------|----------|
  | 1       | 0.0973178 | 0.0854597 | 0.451267 | 0.439935 |
  | 2       | 0.0726207 | 0.096663  | 0.457881 | 0.496596 |
  
- When turning off either radar data or lidar data, RMSE is greater than the threshold. Lidar provides more accurate readings 
compare to radar. The fact that lidar has higher resolution than radar explains the reason. On the other hand, radar data includes
the speed of moving object measured directly which lidar is not capable of measuring. Fusing both lidar and radar data and using extended 
Kalman filter techniques that linearize the non-linear measurement function to update the predicted state of object derived from 
radar sensor data, the tracking results get improved.
