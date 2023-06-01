import numpy as np

def convert_can_data():
    '''Takes raw CAN data, in Hex, and converts it to integers.
    '''
    return np.zeros(NUM_FRAMES)

def adc_cts_to_joint_torques(adc_cts_arr):
    '''Takes the ADC counts from the hall sensors, multiplies them by the slope of the
    torque/ADC_ct linear relationship, and returns values of torque in Nm.
    '''
    torque_scalar = 0 #fill in with real slope
    return 5 * np.array(adc_cts_arr)

def convert_encoder_cts_to_angles(encoder_cts_arr):
    '''Takes the encoder data, in "counts", and divides by the resolution of the encoder
    to yield angles in radians.
    '''
    enc_res = 2**20
    return ((1/enc_res) *2*np.pi) * encoder_cts_arr