from math import atan2

def get_angle_from_point(y,x):
    # if y>=0:
    #     print("y>=0")
    #     return atan2(y,x)
    # else:
    #     print("y<0")
    #     return atan2(y,x)
    return atan2(y,x)

angle =  get_angle_from_point(1,0)
print("弧度：",angle)
print("角度：",angle*180/3.1415)