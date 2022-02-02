# -*- coding: utf-8 -*-
"""
Created on Thu Nov  4 16:52:10 2021

@author: Sebastián Matías Pazos
"""
#importing libraries
import matplotlib.pyplot as plt
from multiprocessing import Process, Pipe
import numpy as np
import pyfirmata
import time
import sys, traceback
from PIL import Image
import scipy.io
from datetime import datetime
import os

# Global Constants and definitions
VDD_ARDUINO = 4.69  # Suggestion: Measure with voltmeter for accuracy and update value as needed
V_DIVIDER = 2       # Voltage divider implemented in the bread borad for voltage limitation.
bias_value = 1.5    # Sample bias voltage, sets the PWM output to bias the device.


#============================================================================#
# Thread controlling the arduino through PyFirmata. 
# Communicates with main thread via a pipe for device bias control and sampling
#
def monitorArduino(clkslow,outputs_dir,conn):
    # Set a log file for the thread's standard output
    sys.stdout = open(outputs_dir+'arduino_log.txt', 'w')
    # Initilize variables
    blink = False
    shuntflag = False
    msg = 0
    tia_output = 0
    
    # Initilize and configure the Arduino board. 
    print('Intializing Arduino board...')
    board = pyfirmata.Arduino('COM7') # Replace port number as needed
    it = pyfirmata.util.Iterator(board)
    # Set up analog input reporting to sample device signal
    board.analog[1].enable_reporting()
    # Set PWM output for device bias
    bias = board.digital[11]
    bias.mode = pyfirmata.PWM
    pwm_counter = bias_value/VDD_ARDUINO
    bias.write(pwm_counter)
    # Use a digital output to enable a shunt transistor, reducing PWM discharge time (FAST DEVICE RESET)
    shuntbias = board.digital[3]
    shuntbias.write = False
    
    it.start()
    time.sleep(2)
    conn.send(5)
    print('Intialization succesful, running main loop')
    # t0 = time.time()
    
    # Polling loop with main thread pipe. 
    while True:
        if conn.poll():
            msg = conn.recv()
            # Command when finish or suspend execution
            if msg == 3:      #3 = BREAK
                break
            # Command to switch off bias (RESET device)
            if msg == 9: 
                bias.write(0)
                shuntbias.write = True
                shuntflag = True
                print('Turning bias OFF')
                msg = False
            # Command to switch on bias (SET device)
            if msg == 8:
                bias.write(pwm_counter)
                shuntbias.write = False
                shuntflag = False
                print('Turning bias ON')
                msg = False
                conn.send(12)
                
    # if time.now()/t0 > 1/clkslow:
        # t0 = time.now()
        #Read the analog input and blink "alive" LED
        blink = not blink
        board.digital[13].write(blink)
        
        # Read the analog input of the Arduino and send data to main thread
        tia_output = board.analog[1].read()
        conn.send(tia_output)
        
        # Wiat for arduino sampling "clkslow". Max is 20 Hz with PyFirmata's default polling.
        time.sleep(1/clkslow)
        # print(tia_output)
        
    # Tidy-up when finished or interrupted by user
    print('Closing ports and finishing process...')
    board.exit()
    board.sp.close()
    print('Done.')
    sys.stdout.close()
    conn.send(4)    #4 = ACK
    conn.close()

#============================================================================#
# Auxiliary function to initialize output log files
#
def outputLogFiles(outputs_dir):
    sourceFile = open(outputs_dir+'nlfsrlog.txt', 'w')
    tiaFile = open(outputs_dir+'tiaoutput.txt','w')
    print('NLFSR re-seeding events logical values', file = sourceFile)
    print('Analog Input values from TIA', file = tiaFile)
    return sourceFile, tiaFile


#============================================================================#
# A function to update the live graphics of the current in the memristor.
#
def updatePlot(x,y,tn,tia_output,line1,line2,ax,fig,THRESHOLD_TIA):
    # tn += time.time()
    x.append(tn)
    y.append(tia_output*V_DIVIDER*VDD_ARDUINO)
    # set the new data
    line1.set_xdata(x)
    line1.set_ydata(y)
    line2.set_ydata([THRESHOLD_TIA, THRESHOLD_TIA])
    #line1.set_ydata(tia_output)
    #line1.set_xdata(count)
    ax.relim() 
    ax.autoscale_view(True,True,True)
    ax.set_xlim(left=tn-60,right=tn+1)
    fig.canvas.draw()
    fig.canvas.flush_events()
    return x, y
    # return


# ===========================================================================#
# This thread runs the NLFSR freely in a while loop
# Re-seeding is performed periodically signaling through a pipe from the main process.
#
def nlfsrThread(state,count,outputs_dir,conn):
    # Set a log file for the thread standard output
    sys.stdout = open(outputs_dir+'logNLFSR.txt', 'w')
    
    # Define the nlfsr register length and number of nlfsr cycles to be ran. 
    # Default values are for ~5.4 M data bits in ~3 minutes (depends on PC HW).
    cycle_count = 10
    reg_length = 19
    
    # Write NLFSR configuration to log file.
    print('NLFSR Length = {0:d}, Cycle Count = {1:d}'. \
              format(reg_length, cycle_count))
    
    # Allocate output variable for the complete output stream
    max_regsize = round(cycle_count*((2**reg_length)-1))
    nlfsr_out = np.zeros(max_regsize,dtype=np.int8)
    
    # Initilize variables
    feedback = 0
    reseed = False
    # t0 = time.time_ns()
    # try:
    print('STARTING')
    while True:
        # input()
        #t1 = time.time()
        # Update NLFSR state
        var = [int(x) for x in list('{0:0b}'.format(state).zfill(reg_length)) ]
        
        # Feedback functions provided for two example cases. Replace as needed.
        if reg_length == 18:
            # Feedback function of 18-bit, maximum period NLFSR.
            feedback = (var[17] ^ var[16] ^ (var[8] & var[5])) ^ (var[6] & var[5])   # Reg length = 18
        if reg_length == 19:
            # Feedback function of 19-bit, maximum period NLFSR.
            feedback = (var[18] ^ var[11] ^ var[8]) ^ (var[12] & var[0])   
        
        nlfsr_out[count] = var[reg_length-1]
        # print(var)
        # print(feedback)
        # input()
        #Save data to the output logs
        # tn = time.time_ns()-t0
        # time_buffer[count] = tn
        # nlfsr_out[count] = var[0]
        
        #print(var[0], tn, file = nlfsr_log)
        #print(nlfsr_out[count], time_buffer[count], file = nlfsr_log)
        # feedback = ((var[0] ^ var[1]) ^ (var[8] ^ var[9])) ^ (var[7] & var[18]);
        
        # tp = time.time_ns()
        if conn.poll():
            reseed = conn.recv()
            if reseed == 3:      #3 = BREAK
                break
            feedback = feedback ^ 1
            # print('Re-seeded!')
            
            if reseed:
                feedback = 1
                reseed = False
            else:
                feedback = 0
        
        # Apply the feedback to the NLFSR, updating its state
        state = int("".join(str(i) for i in var),2)
        if feedback==1:
            state = (state >> 1) | (feedback << reg_length-1)
        if feedback==0:
            state = (state >> 1) & (~(1 << reg_length-1))
        
        # Debugging options, save the whole state of the NLFSR in each cycle
        # This severely slows down the number generation rate.
        # te = time.time_ns() - tp
        # time_buffer[count] = te
        # print(var,file=nlfsrstates)
        # print(var)
        # input()
        
        #End condition: two complete periods of the NLFSR
        count += 1
        if count == max_regsize:
            conn.send(5)
            break
   
    # Tidy up, sync and finish execution    
    print('Done. Now saving...')
    if reseed != 3:     
        # Save NLFSR output in 3 formats: .mat, .npy and plain text.
        print('Saving .mat...')
        mdic = {"nlfsr_out": nlfsr_out, "label": "experiment"}
        scipy.io.savemat(outputs_dir+'nlfsr_results.mat', mdic)
        
        print('Saving text binary values .txt...')
        np.savetxt(outputs_dir+'nlfsrout.txt', nlfsr_out, fmt='%d')
        
        # np.save('time_instants', time_buffer, allow_pickle=True)
        print('Saving .npy...')
        np.save(outputs_dir+'nlfsr_results', nlfsr_out, allow_pickle=True)
        
        # Reshape data to generate a visual bitmap of the output bitstream.
        square = round(max_regsize**0.5)
        if square < max_regsize**0.5:
            array = np.reshape(nlfsr_out[0:square**2],(square,square))
        else:
            array = np.reshape(nlfsr_out[0:square*(square-1)],(square,square-1))
        print('Saving .bmp...')
        im = Image.fromarray(array*255).convert('1')
        im.save(outputs_dir+'trngout.bmp')
    
    # Syncronize with main thread and finish execution
    print('Finished.')
    sys.stdout.close()
    conn.send(4)    #4 = ACK
    conn.close()
    
    
# ===========================================================================#
# Main thread. Syncronizes threads, updates graphics and polls Arduino.
#        
def main():
    try:
        # Use the global variable simulationMode to set the mode:
        # simulationMode = False -> Experiment (needs a device conected)
        # simulationMode = True  -> Simulation (uses a previous acquisition of an experiment)
        # simulationMode = True
        simulationMode = False
        
        #Initialize variables and output directory to avoid data replacement
        now = datetime.now()
        datestr = now.strftime("%d-%m-%Y_%H%M%S")
        outputs_dir = 'outputs/'+datestr+'/'
        os.mkdir(outputs_dir)
        
        # Variables that impact performance.
        clkslow = 20    #Arduino polling frequency in Herz, do not change
        clkreseed = 15  #Reseeding frequency in Herz
        
        count = 0
        # waitcount = 0
        reset = False
        modulo = 10000;
        tia_output = 0
        # last_value = 0
        x = [0]
        y = [0]
        tn = 0
        
        if simulationMode == True:
            # In Simulation mode, select the File that was captured with the Arduino
            # in a previous session to be used as input to the system.
            # analogInput = np.load('G:/Mi unidad/Work/PosDoc/Trabajos/TRNG KAUST/arduino/python/outputs/Buenas/22-11-2021_154524/analogInput.npy')
            # analogInput = np.load('G:/Mi unidad/Work/PosDoc/Trabajos/TRNG KAUST/arduino/python/outputs/Buenas/22-11-2021_151252/analogInput_edited.npy')
            # analogInput = analogInput[370:4000]
            analogInput = np.load('./examples/example1.npy')
            # analogInput = np.load('./examples/example2.npy')
        
        # sample = 0
        # Threshold level for the analog input comparator        
        THRESHOLD_TIA = 2.5   # 2.5 for short meas
        THRESHOLD = THRESHOLD_TIA/V_DIVIDER/VDD_ARDUINO
        # Threshold limit that triggers device RESET (value between 0 and 1, relative to ouput TIA output dynamic range)
        SET_LIMIT = 0.4
        # Time (in seconds) to wait for device RESET/RELAXATION. Change dependending on architecture (shunt), PWM filter time constant and device characteristics
        RESET_WAIT = 3
        # Live plot intialization
        #now = datetime.now()
        fig = plt.figure()
        ax = fig.add_subplot(111)
        # Next line returns a tuple of line objects, thus the comma
        line1, = ax.plot(count, tia_output, 'r-')
        line2, = ax.plot([0,200],[THRESHOLD_TIA,THRESHOLD_TIA],'b-')
        
        #Arduino Board initialization
        #board, it, bias, tia_output = setupArduino()
        
        #Setup output logfiles
        nlfsr_log,tia_log = outputLogFiles(outputs_dir)
        print('Reseed frequency = {0:d}'. format(clkreseed), file=nlfsr_log)
        print('Sample: R3-C4-A, Gain = 1M, bias = {0:.2f}, threshold = {1:.2f}'. \
              format(bias_value, THRESHOLD), file=tia_log)
        
        #Python Shift Register initialization for NLFSR, according to length
        # state = 174762    # Seed is intercalated 1's and 0's, length=18
        state = 349525    # Seed is intercalated 1's and 0's, length=19
        
        #Wait an ad-hoc time for GUI and Arduino initialization
        time.sleep(1)
        
        # Launch NLFSR thread and setup IPC
        parent_conn, child_conn = Pipe()
        p = Process(target=nlfsrThread, args=(state,count,outputs_dir,child_conn,))
        p.start()
        nlfsr_running = True
        print('NLFSR process launched.')
        # print('parent process:', os.getppid())
        # print('process id:', os.getpid())
        
        # Launch Arduino process and setup IPC
        parent_conn_ard, child_conn_ard = Pipe()
        ard = Process(target=monitorArduino, args=(clkslow,outputs_dir,child_conn_ard,))
        ard.start()
        print('Arduino monitor process launched, waiting for response...')
        while parent_conn_ard.recv() != 5:
            print('Waiting...')
        print('Done.')
        
        # Main loop
        t0 = time.time()
        while True:
            # BETA: A moving average as a threshold condition for reseeding
            # length = len(y)
            # if len(y) > 20:
            #     THRESHOLD_TIA = np.average(y[length-20:length])
            #     THRESHOLD = THRESHOLD_TIA/V_DIVIDER/VDD_ARDUINO
            
            # Poll NLFSR process in case it finishes execution
            if parent_conn.poll():
                if parent_conn.recv() == 5:
                    print('NLFSR completed rquested periods and is shutting down. Elapsed time was:')
                    print(time.time()-t0)
                    nlfsr_running = False
                    break
            # print('Data point received')
            tn = time.time()-t0
            # print(tia_output, tn, file=tia_log)
            # print(tia_output)
            
            # time.sleep(1/clkslow)
            
            # Periodicity of the reseeding process, send commands to NLFSR thread. 
            if modulo > (tn % (1/clkreseed)):
                if tia_output > THRESHOLD:
                    parent_conn.send(True)
                    print('NLFSR re-seeded with 1!', tn, file = nlfsr_log)
                    # print('Re-seeding!')
                if tia_output < THRESHOLD:
                    parent_conn.send(False)
                    print('NLFSR re-seeded with 0!', tn, file = nlfsr_log)
            modulo = tn % (1/clkreseed)            
            
            # if tia_output > THRESHOLD and last_value < THRESHOLD:
            # sample += 1
            # if sample == 10:
                # sample = 0
                
            # Poll Arduino for incoming data (analog input)
            message = parent_conn_ard.recv()
            # t1 = time.time()
            if message != 12:
                # Use the global variable simulationMode to set the mode:
                # simulationMode = False -> Experiment (needs a device conected)
                # simulationMode = True  -> Simulation (uses a previous acquisition of an experiment)
                if simulationMode == False:
                    # In experiment mode, current data point is obtained from the arduino
                    tia_output = message
                else:
                    # In simulation mode, current data point is extracted from the read file
                    tia_output = analogInput[count]/(V_DIVIDER*VDD_ARDUINO)
                
                count = count + 1
                
                # BETA: A threshold-only based reseeding
                # if tia_output > THRESHOLD and last_value < THRESHOLD:
                #     parent_conn.send(True)
                #     print('NLFSR re-seeded! (+)', tn, file = nlfsr_log)
                #     # print('Re-seeding!')
                # else: 
                #     if tia_output < THRESHOLD and last_value > THRESHOLD:
                #     # if tia_output < THRESHOLD:
                #         parent_conn.send(False)
                #         print('NLFSR re-seeded! (-)', tn, file = nlfsr_log)
                #     #print('Re-seeding!')
                #     # else:
                #     #     parent_conn.send(False)
                #     #     # print('Do not re-seed')
                
                # Condition for Reseting the device in case the amplifier saturates.
                # Sends Reseting command to Arduino and logs the event.  
                
                if (tia_output > SET_LIMIT and reset is False):
                    print('Conductance state changed, reseting device...')
                    parent_conn_ard.send(9)
                    reset = True
                    print('Conductance state changed, reseting device...', tn,\
                          file = nlfsr_log)
                    twait = time.time()
                
                # Wait a fixed time (3 seconds default) to turn bias back on and log event. 
                # Depends on time constant of PWM.
                # Can reduce times by including a discharge path.
                if reset is True and time.time()-twait >= RESET_WAIT: 
                    # waitcount = waitcount + 1
                    # print(waitcount)
                    # if time.time()-twait > 3: #waitcount == 60:
                    print('Device reset, re-applying bias...')
                    twait = 0
                    parent_conn_ard.send(8)
                    reset = False
                    print('Device reset, re-applying bias...', tn, file = nlfsr_log)
            else:
                reset = False
            
            # Update the interactive plot
            #parent_conn_gui.send(tia_output)
            x.append(tn)
            y.append(tia_output*V_DIVIDER*VDD_ARDUINO)
            x,y = updatePlot(x,y,tn,tia_output,line1,line2,ax,fig,THRESHOLD_TIA)
            
            # Debugging lines
            # deltat = time.time() - t1
            # if deltat > 1/clkslow:
            #     if parent_conn.poll():
            #         parent_conn.recv()
            #         print(deltat)
            # last_value = tia_output
        
        # Tidy up in case user interrupt
        print("Shutdown requested...")
        print('Finishing Arduino monitor process...')
        parent_conn_ard.send(3)
        while parent_conn_ard.recv() != 4:
            print('Flushing Arduino Pipe')    
        print('Signal received for completion, terminating...')
        parent_conn_ard.close()
        child_conn_ard.close()
        ard.terminate()
        ard.join()
        if nlfsr_running:
            print('Finishing NLFSR process...')
            parent_conn.send(3)
        while parent_conn.recv() != 4:
            print('Flushing') 
        print('Signal received for completion, terminating...')
        parent_conn.close()
        child_conn.close()
        p.terminate()
        p.join()
        print('Done.')
        print('Saving Analog data...')
        
        # Save analog input data to .npy format
        ax.plot(x, y, 'r-')
        ax.plot([0,tn],[THRESHOLD_TIA,THRESHOLD_TIA],'b-')
        np.save(outputs_dir+'analogInput', y, allow_pickle=True)
        np.save(outputs_dir+'analogTimes', x, allow_pickle=True)
        
        # Log and finish
        print('Done.')
        print('Routine terminated', file = nlfsr_log)
        print('Routine terminated', file = tia_log)
        tia_log.close()
        nlfsr_log.close()

        print("Exiting.")
        return
    
    
    #Capture Ctrl+C to perform port tidy-up before interrupting execution
    except KeyboardInterrupt:
        print("Shutdown requested...")
        if nlfsr_running:
            print('Finishing NLFSR process...')
            parent_conn.send(3)
            while parent_conn.recv() != 4:
                print('Flushing') 
            print('Signal received for completion, terminating...')
            parent_conn.close()
            child_conn.close()
        p.terminate()
        p.join()
        print('Done.')
        print('Finishing Arduino monitor process...')
        parent_conn_ard.send(3)
        while parent_conn_ard.recv() != 4:
            print('Flushing')    
        print('Signal received for completion, terminating...')
        parent_conn_ard.close()
        child_conn_ard.close()
        ard.terminate()
        ard.join()
        print('Done.')
        
        np.save(outputs_dir+'analogInput', y, allow_pickle=True)
        np.save(outputs_dir+'analogTimes', x, allow_pickle=True)
        print('Routine terminated', file = nlfsr_log)
        print('Routine terminated', file = tia_log)
        tia_log.close()
        nlfsr_log.close()

        print("Exiting.")
        
    except Exception:
        traceback.print_exc(file=sys.stdout)
    sys.exit(0)

if __name__ == "__main__":
    main()


    
    