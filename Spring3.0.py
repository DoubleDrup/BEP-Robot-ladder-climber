import math


r_circle = 0.05
r_width = 0.028
r_grav = 0.05 + r_circle
m = 0.350
g = 9.81
a = 14
a_rad = a*math.pi/180
b_rad = a_rad/2
r_spring = r_width*math.cos(b_rad) - r_circle*math.sin(a_rad-b_rad)
l_a0 = 2*r_circle
l_a = 2*r_circle*math.cos(a_rad-b_rad) + 2*r_spring*math.sin(b_rad)
u = l_a - l_a0
r_grav_sqr = r_grav*math.cos(a_rad)
k = m*g*(r_grav_sqr)/(2*r_spring*u)

print(f"u = {round(u*1000,2)} mm")
print(f"Minimum spring constant = {round((k/1000),2)} N/mm")




