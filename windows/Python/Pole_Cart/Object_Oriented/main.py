from Cart_Pole_Functions import Visual_Model




if __name__ == '__main__':
    model = Visual_Model(Length = 6.1, mass_p = 2.31, mass_c = 7.92, voltage = 1, theta = 0)
    model.Visual_Modle_Make()
    model.run_visual_sim(run_time= 10)
    model.graph_model()
    model.run_graph_sim(run_time=10)



    while True: continue