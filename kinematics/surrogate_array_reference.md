# Surrogate Array Quick-Reference
Shape: (n,m,9) \
Shape[0] = Jounce intervals \
Shape[1] = Steer intervals \
Shape[2] = No. of variables
## Variables:
| Index | Description | Unit |
| ---   | ---         | ---  |
| 0 | Relative shock displacement | mm | front_shock_travel
| 1 | Relative steering rack displacement | mm | steering_rack_delta

| 2 | Motion ratio | | motion ratio: definition = a (a+bx) // done
| 3 | Contact patch position, x | mm | wheelbase and trackwidth // done
| 4 | Contact patch position, y | mm | // done
| 5 | Contact patch position, z | mm | solve with motion ratio 
| 6 | Contact patch travel path unit vector, x | | 0  // done 
| 7 | Contact patch travel path unit vector, y | | 0  // done 
| 8 | Contact patch travel path unit vector, z | | 1  // done
| 9 | Wheel pose, x "camber" | radians | left and right camber gain
| 10 | Wheel pose, z "toe" | radians | steer