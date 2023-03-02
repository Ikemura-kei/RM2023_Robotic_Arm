#CMD message from robot commend
# name : name of the group of robot
# mode : three different mode of control
#   including "0":Cartesian control
#             "1":Point to Point movement
#             "2":Joint control
# positions : goal End effector coordinate x , y and z
# orientation : goal orientation of End effector with x,y,z,w
# topic name: cmd_pub

#coordinate definition:
# end effector with respect to base(usually to cross bearing in real robot)
# right hand coordinate( cross from x axis to y axis give w axis)
# usually the robot front with be the x axis and w axis was pointing up





Header header

string name
uint32 mode
float64[3] positions
float64[4] orientation