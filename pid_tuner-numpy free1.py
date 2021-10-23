
import dearpygui.dearpygui as dpg


import math


class interp1d:
    
    def __init__(self,arr):
        self.arr = arr

    def __call__(self,x):
        x0 = int(math.floor(x))
        x1 = int(math.floor(x+1))
        y0 = self.arr[x0]
        y1 = self.arr[x1]
        return  y0 +  ((y1-y0)*(x-x0)/(x1-x0))

def roll(arr,u):
    """ shifts the elements of an array or list by n to the right"""
    return [u]+ arr[:-1]


def PID( P, I,D,w=0,dt=1,MVmin = 0, MVmax = 100,direction=-1):

    
    ecurr = 0
    eprev = 0
    roc_error = 0 
    t = 0
    u = 0
    
    integral = 0
    
    while True:
        SV,PV = yield u
        eprev2 = eprev
        eprev = ecurr
        ecurr = PV -w*SV
        
        
        integral_error = PV - SV

        delta_e = ecurr - eprev
        roc_error = (ecurr - 2*eprev + eprev2)/dt
        

        
        prop = P*delta_e
        integral = P*integral_error*dt/I
        der = P*roc_error*D/dt
        delta_u = (prop + integral + der)*direction  # velocity form 
        u += delta_u 
        u = max(MVmin,min(MVmax,u))
        
        t += dt
        

def process(k,tau,delay=0):
    y=0
    delay_pipe = [0]*(int(delay+2))
    dydt =0

    while True:
        u = yield y
        delay_pipe = roll(delay_pipe,u)
        
        delay_pipe_int = interp1d(delay_pipe)
        u_int = delay_pipe_int(delay)
        
        dydt = (k * u_int - y)/tau
        y = y + dydt
        

             


def ramp_process(k,delay=0):
    y=0
    
    
    delay_pipe = [0]*(int(delay+2))

    while True:
        u = yield y
        delay_pipe = roll(delay_pipe,u)
        
        delay_pipe_int = interp1d(delay_pipe)
        u_int = delay_pipe_int(delay)
        dy = k* u_int
        y += dy 
        


def simulate(P,I,D,w,kp,tau,delay,ramp=False,closed_loop=True,history_length=1000):
   
    
    controller = PID(P,I,D,w)
    if ramp:
        system = ramp_process(kp,delay)
        
    else:
        system = process(kp,tau,delay)
        
 
    
    system.send(None)
    controller.send(None)
    MV_log = []
    PV_log = []
    SV_log = []
    PV= 0
    MV=0
    SV = 0
    
    for i in range(history_length):
        MV_log.append(MV)
        PV_log.append(PV)
        SV_log.append(SV)
        if (closed_loop):
            if i > 10:
                SV = 1
            
            MV = controller.send((SV,PV))
            PV = system.send(MV)
        
        else:
            if i > 10 :
                MV =1
                PV = system.send(MV)
        
    return PV_log,MV_log,SV_log
    
   

def simulate_dist(P,I,D,w,kp,tau,delay,kp_dist,tau_dist,ramp=False,closed_loop=True, history_length=100):
  
    
    
    controller = PID(P,I,D,w,MVmin=-100)
    if ramp:
        system = ramp_process(kp,delay)
        dist  = ramp_process(kp_dist)
    else:
        system = process(kp,tau,delay)
        dist = process(kp_dist,tau_dist,0)

    
    dist.send(None)
    system.send(None)
    controller.send(None)
    MV_log_dist = []
    PV_log_dist = []
    SV_log_dist = []
    PV= 0
    MV=0
    SV = 0
    MV_dist = 0
    
    for i in range(history_length):
        MV_log_dist.append(MV)
        PV_log_dist.append(PV)
        SV_log_dist.append(SV)
        if( closed_loop):
            if i > 10:
                MV_dist = 1
            PV_dist = dist.send(MV_dist) 
            PV_process = system.send(MV)
            PV = PV_process + PV_dist
            MV = controller.send((SV,PV))
        else:
            if i > 10:
                MV_dist = 1
            PV = dist.send(MV_dist) 
            
            
        
        
    return PV_log_dist,MV_log_dist,SV_log_dist    
def updateplot():
    
    reset = dpg.get_value(check_box_reset)
    history_length = dpg.get_value(input_slider_steps)

    kp = dpg.get_value(input_slider_kp)
    tau = dpg.get_value(input_slider_tau)
    delay = dpg.get_value(input_slider_delay)

    

    kp_dist = dpg.get_value(input_slider_kp_dist)
    tau_dist = dpg.get_value(input_slider_tau_dist)
    
    
    
    ramp = dpg.get_value(check_box_ramp)
    open_loop = dpg.get_value(check_box_openloop)
    closed_loop = not(open_loop)
    
    if (reset):
        dpg.configure_item(input_slider_P,no_input=True)

        if not(ramp):
            lam = tau+ delay
            initial_P = (2*tau+ delay )/(2*kp*(lam+ delay))
            initial_I = tau+ delay/2
            initial_D = tau*delay/(2*tau+delay)
            dpg.set_value(input_slider_P, initial_P ) 
            dpg.set_value(input_slider_I, initial_I)
            dpg.set_value(input_slider_D,initial_D)
            

            
        
        if (ramp):
            lam = 1.5/kp
            initial_P = (2*lam+delay)/(kp*(lam+delay/2)**2)
            initial_I = 2*lam+delay
            initial_D = (lam*delay + ((delay**2) /4)) /(2*lam+ delay)
            dpg.set_value(input_slider_P, initial_P ) 
            dpg.set_value(input_slider_I, initial_I)
            dpg.set_value(input_slider_D,initial_D)


    P = dpg.get_value(input_slider_P)
    I = dpg.get_value(input_slider_I)
    D = dpg.get_value(input_slider_D)
    w = dpg.get_value(input_slider_w)

    PV_log,MV_log,SV_log = simulate(P,I,D,w,kp,tau,delay,ramp,closed_loop,history_length) 
    PV_log_dist,MV_log_dist,SV_log_dist = simulate_dist(P,I,D,w,kp,tau,delay,kp_dist,tau_dist,ramp,closed_loop,history_length) 
    


    
    dpg.configure_item(input_slider_tau,show =  not(ramp))
    
    dpg.configure_item(line_PV, x=list(range(history_length)),y=PV_log)
    dpg.configure_item(line_MV,x=list(range(history_length)),y=MV_log)
    dpg.configure_item(line_SV,x=list(range(history_length)),y=SV_log)

    dpg.configure_item(line_PV_dist,x=list(range(history_length)),y=PV_log_dist)
    dpg.configure_item(line_MV_dist,x=list(range(history_length)),y=MV_log_dist)
    dpg.configure_item(line_SV_dist,x=list(range(history_length)),y=SV_log_dist)




with dpg.theme(default_theme=True):
    dpg.add_theme_style(dpg.mvPlotStyleVar_LineWeight,3.0,category=dpg.mvThemeCat_Plots)
    

with dpg.window(label="PID tuner",width=1000,height=600) as main_window:

    

    
    
    with dpg.collapsing_header(label = "Process Model"):
    
        
        

        check_box_ramp = dpg.add_checkbox(label="ramp",callback=updateplot)
        
        input_slider_kp= dpg.add_slider_float(label="kp",min_value=1e-5,max_value=1,width= 1000,callback=updateplot)
        dpg.set_value(input_slider_kp,0.1)
        
        input_slider_tau = dpg.add_slider_float(label="tau",min_value=1,max_value=1000,width= 1000,callback=updateplot)
        dpg.set_value(input_slider_tau,10)
        input_slider_delay = dpg.add_slider_float(label="delay",min_value=0,max_value=1000,width= 1000,callback=updateplot)
        dpg.set_value(input_slider_delay,1)

    with dpg.collapsing_header(label="Distubance model"):
            

        input_slider_kp_dist= dpg.add_slider_float(label="kp_dist",min_value=0.01,max_value=10,width= 1000,callback=updateplot)
        dpg.set_value(input_slider_kp_dist,0.1)
        input_slider_tau_dist = dpg.add_slider_float(label="tau_dist",min_value=1,max_value=1000,width= 1000,callback=updateplot)
        dpg.set_value(input_slider_tau_dist,10)

    dpg.add_separator()

    with dpg.group(label="PID Parameters", horizontal=True, horizontal_spacing=500):
        dpg.add_text("PID Parameters")
        
        check_box_reset = dpg.add_checkbox(label="reset to lambda tuning", callback=updateplot)

    with dpg.group(label="PID parameters"):
        
        input_slider_P  = dpg.add_slider_float(label="P",min_value=0.1,max_value=100,width= 1000,callback=updateplot)
        dpg.set_value(input_slider_P ,1)
        input_slider_I = dpg.add_slider_float(label="I",min_value=0.01,max_value=10000,width= 1000,callback=updateplot)
        dpg.set_value(input_slider_I ,20)
        input_slider_D = dpg.add_slider_float(label="D",min_value=0,max_value=1000,width= 1000,callback=updateplot)
        dpg.set_value(input_slider_D,1)
        input_slider_w = dpg.add_slider_float(label="setpoint_weighting",min_value=0.0,max_value=1.0,width=100,callback=updateplot)
    
    dpg.add_separator()
    with dpg.group(label = "options", horizontal =True , horizontal_spacing=500):
        check_box_openloop = dpg.add_checkbox(label="openloop",callback=updateplot)

        input_slider_steps = dpg.add_slider_int(label="steps to simulate",min_value = 100 , max_value=10000,width = 100 ,callback=updateplot)
        dpg.set_value(input_slider_steps,1000)

    dpg.add_separator()
    
    
    setpoint_plot = dpg.add_plot(height= 300, width=1000,label="Setpoint response")
    disturbance_plot = dpg.add_plot(height= 300, width=1000,label = "Disturbance Response")
    
    axis_x = dpg.add_plot_axis(dpg.mvXAxis,label="step",parent = setpoint_plot)
    axis_PV = dpg.add_plot_axis(dpg.mvYAxis,label="PV",parent = setpoint_plot)
    axis_MV= dpg.add_plot_axis(dpg.mvYAxis,label="MV",parent= setpoint_plot)
    axis_SV = dpg.add_plot_axis(dpg.mvYAxis,label="SV",parent= setpoint_plot) 
    line_PV = dpg.add_line_series(x= list(range(100)),y=[0]*100, parent=axis_PV,label="PV")
    line_MV = dpg.add_line_series(x= list(range(100)),y=[0]*100, parent=axis_MV,label="MV")
    line_SV = dpg.add_line_series(x= list(range(100)),y=[0]*100, parent=axis_SV,label="SV")
    
    
   

  
    
    axis_x_dist = dpg.add_plot_axis(dpg.mvXAxis,label="step",parent = disturbance_plot)
    axis_PV_dist = dpg.add_plot_axis(dpg.mvYAxis,label="PV",parent = disturbance_plot)
    axis_MV_dist= dpg.add_plot_axis(dpg.mvYAxis,label="MV",parent= disturbance_plot)
    axis_SV_dist = dpg.add_plot_axis(dpg.mvYAxis,label="SV",parent= disturbance_plot) 
    line_PV_dist = dpg.add_line_series(x= list(range(100)),y=[0]*100, parent=axis_PV_dist,label="PV_d")
    line_MV_dist = dpg.add_line_series(x= list(range(100)),y=[0]*100, parent=axis_MV_dist,label="MV_d")
    line_SV_dist= dpg.add_line_series(x= list(range(100)),y=[0]*100, parent=axis_SV_dist,label="SV_d")

dpg.setup_viewport()
dpg.set_viewport_title("PlantPy PID tuner")
dpg.configure_viewport(0, x_pos=200, y_pos=50, width=1100, height=800)

dpg.set_primary_window(main_window,True)
dpg.start_dearpygui()
