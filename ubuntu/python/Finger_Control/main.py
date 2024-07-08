import test
# import Dynamic_Matrix_Builder
import Sim_Model
import math




def main():
    # test.helloworld()
    prev_theta_D = 0
    prev_theta_P = 0
    prev_theta_M = 0
    voltage = 12.2
    theta_M, theta_P, theta_D = Sim_Model.finger_pos_update(voltage, prev_theta_D, prev_theta_P, prev_theta_M, time_Step = 0.03)
    print("D ", math.degrees(theta_D))




if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Program interrupted")
    except Exception as e:
        print(f"Unexpected error: {e}")