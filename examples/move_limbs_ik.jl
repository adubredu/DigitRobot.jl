using Revise 
using DigitRobot 

mvis, mech, state = load_digit()
qs = solve_left_leg_ik(0.5175, 0.19145, -0.705491)
set_