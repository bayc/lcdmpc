# helper class to run AES with cosim

import matplotlib
matplotlib.use('tkagg')
import matplotlib.pyplot as plt
import pandas as pd

import os

import cosim
import config

import func_tools

class AES():


    def __init__(self,startTime,stopTime):

        self.main_dir = '/Users/jannoni/Desktop/AES/ModelDevelopment_v3/aes-cosimulation/'

        # simulation inputs
        self.starttime = startTime
        self.stoptime = stopTime
        self.timestep = 1
        self.forecast_horizon = 30  # minutes into the future
        self.forecast_type = 'persistence'

        # resource files
        self.wind_file = 'inputs/examples/buildings/building_all_data.csv'
        self.building_file = 'inputs/examples/wind/Wind.csv'
        self.vehicle_file = 'inputs/examples/vehicles/Vehicles.csv'

        # data inputs
        self.building_columns = ['Site Outdoor Air Drybulb Temperature: Environment Run Period HVAC System Timestep (C)',
                                 'Q_solar',
                                 'Q_internal']

        # output files
        self.power_file = 'outputs/datafile_ev_activepower.csv'
        self.voltage_file = 'outputs/datafile_voltages.csv'
        self.substation_file = 'outputs/datafile_substation.csv'
        self.wind_out = 'outputs/wind_'  # node name is added in agent to identify wind plants
        self.building_out = 'outputs/buildings_'  # node name is added in agent to identify buildings in feeder
        self.vehicle_out = 'outputs/vehicles_'  # node name is added in agent to identify vehicles in feeder

        # wind model
        self.wind_model = 'floris'
        self.wind_farm_file = 'inputs/examples/wind/example_input.json'
        self.wind_control = 'simple_control'  # PID control
        self.num_turbines = 100 # only used for the power_curve model

        # buildings model
        self.building_model = 'simple_commercial_building'

        # vehicles model
        self.vehicle_model = 'battery'

    def run(self):
        # run AES prototype
        bus = cosim.communication.local.LocalBus()

        input_data = dict()

        # enter main dir
        input_data['main_dir'] = self.main_dir

        # user defined locations of the building, wind, and storage nodes
        input_data['startTime'] = self.starttime
        input_data['stopTime'] = self.stoptime
        input_data['timestep'] = self.timestep  # minutes
        input_data['forecast_horizon'] = self.forecast_horizon  # minutes into the future # TODO: this will have to be generic timesteps in the long run

        # types of forecasts implemented:
        # 1. 'perfect' -> returns the next x = forecast_horizon timesteps for the controller from raw data,
        # i.e. the controller knows exactly what will happen in the next x timesteps
        # 2. 'persistence' -> what is happening now will also be happening in the next time step
        input_data['forecast_type'] = self.forecast_type

        # TODO: nodes where each of the agents live

        # input data resources files
        input_data['buildings_input'] = self.building_file
        input_data['wind_input'] = self.wind_file
        input_data['storage_input'] = self.vehicle_file

        # input columns for buildings
        input_data['building_columns'] = self.building_columns

        # output files
        input_data['power'] = self.power_file
        input_data['voltages'] = self.voltage_file
        input_data['substations'] = self.substation_file

        input_data['wind_out'] = self.wind_out
        input_data['buildings_out'] = self.building_out
        input_data['vehicles_out'] = self.vehicle_out

        # grid level control types:
        # 1. None
        # under development
        # 2. Limited Communication Distributed Model Predictive Control
        # 3. XXX
        # TODO: not used
        input_data['grid_control'] = None  # only local PID control is implemented
        input_data['num_cells'] = 1  # 1 indicates central control

        # type of model used for wind
        # Options:
        # 1. power_curve
        # 2. floris
        input_data['wind_type'] = self.wind_model

        # control types for wind
        input_data['control_wind'] = self.wind_controller

        # TODO: have an input file for each of the wind farms (right now they are all the same)
        input_data['wind_farm_file'] = self.wind_farm_file

        # inputs to the wind module - only used for the power curve model
        input_data['num_turbines'] = self.num_turbines

        # type of model used for buildings
        # Options:
        # 1. prescribed_load -> from the input file
        # 2. water_heater
        # 3. simple commercial building (HVAC and ISO model)
        # under development:
        # 4. Energy Plus
        input_data['buildings_type'] = 'simple_commercial_building'
        input_data['control_buildings'] = 'no_control'

        # type of model used for vehicles
        # Options:
        # 1. prescribed_load -> from the input file
        # 2. simple_battery
        # under development:
        # 3. Battery -> more sophisticated battery model
        # 4. HIVE
        input_data['vehicles_type'] = 'simple_battery'
        input_data['control_vehicles'] = 'no_control'

        # remove these files if they exist
        func_tools.remove_files(input_data)

        # feeder to run
        input_data['feeder_file'] = './feeder/ieee_13node/master.dss'

        # ===============================
        # run AES
        # ===============================
        # add control or no control
        config.aes(bus, input_data)