#!/usr/bin/env python

from .lac1 import LAC1
from typing import Literal
from .units import ensure_units

MAX_ADC_VALUE = 2047
MAX_ADC_CURRENT = '11.585 A'

class VLC(LAC1):
    """
    Class to interface with a SMAC VLC controller. Based on the LCA1 class with
    additional features to support the VLC controller.

    SMAC serial interface accepts instructions in the format of:

    <command>[<argument>] <CR>

    Or

    <command>[<argument>],<command>[<argument>],... <CR>

    e.g.

    SG1000,SD5000 <CR>

    Note that EF is sent as the first command to LAC-1 on initialisation, and
    EN is sent as the last command on close. This simplifies parsing of outputs.

    Note that for each cmmand sent, with EF in force, LAC-1 will output

        '\r\n>'

    When it is ready for the next command
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._max_current = ensure_units(MAX_ADC_CURRENT, 'A')

    def set_i2t_protection(
        self, nominal_current='1.25 A', peak_current='3 A', time_period='1 s'
    ):
        """
        Set the I2T protection value for the controller. The I2T protection
        mechanism provides a means to configure the actuator overloading
        characteristic in terms of peak current and time period. This can be
        used to protect the motor from overheating.

        Args:
            nominal_current (float): The nominal current value to set.
            overload (float): The overload value to set.
            interval (int): The interval value to set.
        """

        nominal_current = ensure_units(nominal_current, 'A')
        peak_current = ensure_units(peak_current, 'A')
        time_period = ensure_units(time_period, 's')

        # AL: Load accumulator
        # WW610: Set i2t_NOM
        # WW612: Set i2t_TRIP
        # WB620: Set i2tINTRVL
        interval = 0
        nom = round((nominal_current / self._max_current * MAX_ADC_VALUE).magnitude)
        peak = round((peak_current / self._max_current * MAX_ADC_VALUE).magnitude)
        trip = round(
            ((peak**2 - nom**2) * (time_period.to('ms') / (interval + 1))).magnitude
        )

        # The value of i2t_TRIP must be less than 2^31 - 1, so we need to
        # increase the interval until the value is less than this.
        while trip > 2**31 - 1:
            print('trip=', trip)
            interval += 1
            trip = round(
                ((peak**2 - nom**2) * (time_period.to('ms') / (interval + 1))).magnitude
            )
        print('trip=', trip)
        self.sendcmds(f'AL{nom},WW610,AL{trip},WL612,AL{interval},WB620')

    def phase_offset_calibration(self):
        # Set the values of registers 4 and 5 to 0, which will be used to store
        # the phase offsets for motors A and B respectively.
        #
        # AL: Load accumulator
        # AR: Allocate register
        # MF: Motor off
        # WA: Wait
        self.sendcmds('MD300,AL0,AR4,AR5,MF,WA50')

        # Read the output current of phase A and B, add the values of register
        # 4 and 5 and store the values back in registers 4 and 5 respectively.
        # Wait for 1 mm and then repeat 999 times.
        #
        # RW622: Read Phase A output current
        # AA:  Add the value in register to the accumulator
        # AR: Allocate register
        # RW624: Read Phase B output current
        # WA: Wait
        # RP: Repeat the command line
        self.sendcmds('MD301,RW622,AA@4,AR4,RW624,AA@5,AR5,WA1,RP999')

        # Load the accumlator with 0, and allocate to register 1 to use during
        # division. Load the accumulator with 2048000 and subtract the the
        # output current of phase A (register 4). Divided this value by 1000,
        # store in register 4 and write into the ADC offset for phase A.
        # Repeat the same for phase B. Display the offsets.
        #
        # AL: Load accumulator
        # AR: Allocate register
        # AS: Subtract from Accumulator
        # AD: Devide accumulator by value
        # WW606: Write word to ADC offset for phase A
        # WW608: Write word to ADC offset for phase B
        # MG: Display message
        self.sendcmds(
            'MD302,AL0,AR1,AL2048000,AS@4,AD1000,AR4,WW606,AL0,AR1,AL2048000,AS@5,AD1000,AR5,WW608,MG"Offset A/B: ":4:N,MG"/":5'
        )

        return self.sendcmds('MS300')

    def phase_motors(self, ec_counts=47800):
        ideal = int(ec_counts / 4)
        upper = int(ideal * 1.1)
        lower = int(ideal * 0.9)

        # Set the servo phasing to reversed, turn off the motor, set the
        # electronic commutation counts to 0 to allow phasing. Lock the shaft
        # by setting the commutation phase angle to 0, entering position mode,
        #  setting the torque to maximum, swtiching to torque mode, turning on
        # the motor, and setting the maxmimum voltage to approximetly 30%.
        #
        # PH: Set the phasing
        # MF: Motor off
        # EC: Set the electronic commutation counts
        # SP: Set the commutation phase angle
        # PM: Poistion mode
        # SQ: Set the torque
        # QM: Torque mode
        # MN: Motor on
        self.sendcmds('MD200,PH1,MF,EC0,SP0,PM,SQ32767,QM0,MN,SQ9502')

        # Adjust the phasing angle by increments of 90 degrees up to 270 and
        # back to 0. Load the potential absolute home value
        # ((EC/4) + (SP/65536) * EC). At each step, calculate the difference
        # in position after 400 ms and check if it is between +/- 10% of the
        # ideal encorder value. If the value is within the range, the marco
        # will jump to the end, if not, step 11 will be called to display the
        # current step size and the ideal step size for comparison. If the
        # value is outside the range for all steps, the phasing is not
        # successful and the prgoram is ended.
        #
        # AL: Load accumulator
        # AR: Allocate register
        # RL494: Read the position of the encoder and store in register
        # SP: Set the commutation phase angle
        # WA: Wait
        # AS: Subtract from Accumulator
        # IG: If Greater Than
        # IB: If Less Than
        # MJ: Jump to macro
        # NO: No operation
        # MC: Macro call
        # MG: Display message
        self.sendcmds(
            f'MD201,AL{int(ideal + (1/4) * ec_counts)},AR4,RL494,AR3,SP16383,WA400,RL494,AS@3,AR5,IG{lower},IB{upper},MJ208,NO,MC207'
        )
        self.sendcmds(
            f'MD202,AL{int(ideal + (2/4) * ec_counts)},AR4,RL494,AR3,SP32767,WA400,RL494,AS@3,AR5,IG{lower},IB{upper},MJ208,NO,MC207'
        )
        self.sendcmds(
            f'MD203,AL{int(ideal + (3/4) * ec_counts)},AR4,RL494,AR3,SP49150,WA400,RL494,AS@3,AR5,IG{lower},IB{upper},MJ208,NO,MC207'
        )
        self.sendcmds(
            f'MD204,AL{int(ideal + (2/4) * ec_counts)},AR4,RL494,AR3,SP32767,WA400,RL494,AS@3,AM-1,AR5,IG{lower},IB{upper},MJ208,NO,MC207'
        )
        self.sendcmds(
            f'MD205,AL{int(ideal + (1/4) * ec_counts)},AR4,RL494,AR3,SP16383,WA400,RL494,AS@3,AM-1,AR5,IG{lower},IB{upper},MJ208,NO,MC207'
        )
        self.sendcmds(
            f'MD206,AL{int(ideal + (0/4) * ec_counts)},AR4,RL494,AR3,SP0,WA400,RL494,AS@3,AM-1,AR5,IG{lower},IB{upper},MJ208,NO,MC207,SQ0,MG"Phasing NOK",EP '
        )
        # Display the difference between the ideal step size and the actual
        # step size
        #
        # NO: No operation
        # MG: Display message
        # RC: Return from macro call
        self.sendcmds(
            f'MD207,NO,MG"Ideal step [enc cnts]= {ideal}. Actual step= ":5,RC '
        )

        # Define the absolute home based on the phase offset, set the
        # electronic commutation counts encoder increments, stop the motor, and
        # display the phasing successful message with the ideal step size for
        # comparison.
        #
        # RA: Copy Register to Accumulator
        # DA: Define Absolute Home
        self.sendcmds(
            f'MD208,RA4,DA@0,EC{ec_counts},SQ0,MF,MG"Phasing Succesfull, stepsize ({ideal})= ":5'
        )

        input('Press enter to start phasing')
        return self.sendcmds('MS200')
    
    def parameter_save(self):
        # This command saves macros and register values into the Non-Volatile Memory of the VLC. 
        # Unlike LAC1, macros are not stored by default and will not be saved between power 
        # cycles unless this command is called.

        self.sendcmds('PS')

    def _calculate_voltage_constant(self, force):
        """
        Voltage constant between 0 - 32767 linearly corresponds to 0 - max 
        actuator force at current power supply voltage.
        """
        force = ensure_units(force, 'N')

        q = int(force / self.actuator.max_force * 32767)
        return q
    
    def _calculate_current_constant(self, force):
        """
        Current constant between 0 - 2047 linearly corresponds to 0 A - 11.585 A. 
        The current is determined from the force constant of the actuator.
        """
        force = ensure_units(force, 'N')

        q = int(force / self.actuator.force_constant / ensure_units(MAX_ADC_CURRENT, 'A') * MAX_ADC_VALUE)
        return q

    def set_max_force(self, force, mode: Literal['voltage', 'current']='voltage'):
        """
        For voltage-based torque mode, position mode and velocity mode, SQ value of
        0 - 32767 linearly corresponds to 0 - max actuator force at current power 
        supply voltage.

        For current-based torque mode, SQ value of 0 - 2047 linearly corresponds to 
        0 A - 11.585 A. The current is determined from the force constant of the 
        actuator.
        """
        force = ensure_units(force, 'N')

        if(mode == 'voltage'):
            q = self._calculate_voltage_constant(force)
        elif(mode == 'current'):
            q = self._calculate_current_constant(force)

        self.set_max_torque(q)

    def move_to_force(self, force):
        """
        Starts a velocity move and moves until the current exceeds the value 
        based on the corresponding force
        """
        force = ensure_units(force, 'N')
        
        q = self._calculate_current_constant(force)
        self.sendcmds('VM', '', 'MN', '', 'GO', '')
        self.sendcmds('RW', 548, 'IG', q, 'ST', '', 'EP', '', 'RP', '')
